struct MyLocalizationType
    # TODO: add timestamp and perhaps orientation
    field1::Int
    field2::Float64
end

struct MyPerceptionType
    field1::Int
    field2::Float64
end

#Performs routing on current segment found from ground truth position (development only)
function routing(gt_channel, target_segment_id::Int, map::Dict{Int, VehicleSim.RoadSegment})
    #Idea 1 for finding current position
    gt_meas = fetch(gt_channel)
    pos = gt_meas.position
    
    #Idea 2 for finding current position
    #current_state = fetch(state_channel)
    #pos = current_state.q[5:6]

    while true
        fetch(shutdown_channel) && break

        fresh_gt_meas = []
        while isready(gt_channel)
            meas = take!(gt_channel)
            push!(fresh_gt_meas, meas)
        end

    path = find_shortest_path(current_segment_id, target_segment_id, map)

    println("Path: ", path)
  end
end

function h_imu(x)
    T_body_imu = VehicleSim.get_imu_transform()
    T_imu_body = VehicleSim.invert_transform(T_body_imu)
    R = T_imu_body[1:3, 1:3]
    p = T_imu_body[1:3, end]
    v_body = x[8:10]
    ω_body = x[11:13]
    ω_imu = R * ω_body
    v_imu = R * v_body + cross(p, ω_imu)
    return [v_imu; ω_imu]
end


# Finds shortest path to target using BFS. For development, current state of ground truth is used for vehicle position
function find_shortest_path(current_segment_id, target_segment_id::Int, map::Dict{Int, VehicleSim.RoadSegment})
    queue = [current_segment_id]
    visited = Set{Int}(current_segment_id)
    prev = Dict{Int, Int}()
    
    found = false
    # BFS
    while !isempty(queue)
        current = popfirst!(queue)
        if current == target_segment_id
            found = true
            break
        end
        for child in map[current].children
            if child ∉ visited
                push!(queue, child)
                push!(visited, child)
                prev[child] = current
            end
        end
    end
    
    # If no path was found, return an empty array.
    if !found
        @warn "No path found from segment $current_segment_id to $target_segment_id."
        return VehicleSim.RoadSegment[]
    end
    # Return the path as an array of VehicleSim.RoadSegment objects.
    return [map[id] for id in path_ids]
  end

function Jac_h_imu(x)
    # Initialize a 6x13 zero matrix
    H = zeros(6, 13)    
    # Populate the Jacobian with the appropriate derivatives
    H[1, 8] = 1.0  # ∂v_x / ∂x₈
    H[2, 9] = 1.0  # ∂v_y / ∂x₉
    H[3, 10] = 1.0 # ∂v_z / ∂x₁₀
    H[4, 11] = 1.0 # ∂ω_x / ∂x₁₁
    H[5, 12] = 1.0 # ∂ω_y / ∂x₁₂
    H[6, 13] = 1.0 # ∂ω_z / ∂x₁₃
    return H
end
  

function localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel)
    # Set up algorithm / initialize variables
    # process measurements
    proc_cov = Diagonal([0.05, 0.05, 0.01, 0.01, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01])
    gt_states = [zeros(13),] # ground truth states that we will try to estimate
    timesteps = []
    last_timestamp = time()

    meas_cov = Diagonal([0.2, 0.1, 0.1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001])

    #sqrt of these values *2, our mean should be within +/- these values with 95% confidence
    Σs = Matrix{Float64}[Diagonal([25,25,25,0.01,0.01,0.01,0.01,1,1,1,0.01,0.01,0.01]),]

    x_prev = zeros(13)
    zs = Vector{Float64}[]

    while true
        fetch(shutdown_channel) && break
        
        fresh_gps_meas = []
        while !isready(gps_channel)
            fetch(shutdown_channel) && break
            sleep(0.001)
        end
        
        while isready(gps_channel)
            fetch(shutdown_channel) && break
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end

        fresh_imu_meas = []
        while !isready(imu_channel)
            fetch(shutdown_channel) && break
            sleep(0.001)
        end
        while isready(imu_channel)
            fetch(shutdown_channel) && break
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end

        fresh_gt_meas = []
        while !isready(gt_channel)
            fetch(shutdown_channel) && break
            sleep(0.001)
        end
        while isready(gt_channel)
            fetch(shutdown_channel) && break
            meas = take!(gt_channel)
            push!(fresh_gt_meas, meas)
        end

        # Dynamically calculate the time step Δ
        current_timestamp = time()
        Δ = current_timestamp - last_timestamp
        last_timestamp = current_timestamp

        alpha = fresh_gps_meas[end].heading
        μs = [[fresh_gps_meas[end].lat, fresh_gps_meas[end].long, 2.65, cos(alpha/2), 0, 0, sin(alpha/2), fresh_imu_meas[end].linear_vel[1], fresh_imu_meas[end].linear_vel[2], fresh_imu_meas[end].linear_vel[3], fresh_imu_meas[end].angular_vel[1], fresh_imu_meas[end].angular_vel[2], fresh_imu_meas[end].angular_vel[3]]]
        linear_velocity = fresh_imu_meas[end].linear_vel
        angular_velocity = fresh_imu_meas[end].angular_vel
        position = [fresh_gps_meas[end].lat, fresh_gps_meas[end].long, 1.0]
        q = [cos(alpha/2), 0, 0, sin(alpha/2)]

        xₖ = VehicleSim.rigid_body_dynamics(position, q, linear_velocity, angular_velocity, Δ)
        x_prev = xₖ
        zₖ_gps = VehicleSim.h_gps(xₖ)
        zₖ_imu = h_imu(xₖ)
        zₖ = vcat(zₖ_gps, zₖ_imu)


        """
        xₖ = f(xₖ₋₁, uₖ, ωₖ, Δ), where Δ is the time difference between times k and k-1.
        A = ∇ₓf(μₖ₋₁, mₖ, 0, Δ),
        B = ∇ᵤf(μₖ₋₁, mₖ, 0, Δ),
        L = ∇ω f(μₖ₋₁, mₖ, 0, Δ),
        c = f(μₖ₋₁, mₖ, 0, Δ) - Aμₖ₋₁ - Bmₖ - L*0
        μ̂ = Aμₖ₋₁ + Bmₖ + L*0 + c
        = f(μₖ₋₁, mₖ, 0, Δ)
        Σ̂ = A Σₖ₋₁ A' + B proc_cov B' + L dist_cov L'
        C = ∇ₓ h(μ̂), 
        d = h(μ̂) - Cμ̂
        Σₖ = (Σ̂⁻¹ + C' (meas_var)⁻¹ C)⁻¹
        μₖ = Σₖ ( Σ̂⁻¹ μ̂ + C' (meas_var)⁻¹ (zₖ - d) )
        """
        A = VehicleSim.Jac_x_f(μs[end], Δ)
        b = VehicleSim.f(μs[end], Δ) - A*μs[end]
        μ_hat= A*μs[end] + b
        Σ_hat = A*Σs[end]*A' + proc_cov
        C_gps = VehicleSim.Jac_h_gps(μ_hat)
        C_imu = Jac_h_imu(μ_hat)
        C = vcat(C_gps, C_imu)
        d_gps = VehicleSim.h_gps(μ_hat)
        d_imu = h_imu(μ_hat)
        d = vcat(d_gps, d_imu) - C*μ_hat

        Σ = inv(inv(Σ_hat) + C' * inv(meas_cov) * C)
        μ = Σ*(inv(Σ_hat) *μ_hat + C'*(inv(meas_cov))*(zₖ - d))
        push!(μs, μ)
        push!(Σs, Σ)
        push!(zs, zₖ)

        push!(gt_states, xₖ)
        push!(timesteps, Δ)

        if false
            println("Hello")
            println("   Ground truth (x,y): ", fresh_gt_meas[end].position)
            println("   estimated: ", μ[1:3])
            println("   GT linear: ", fresh_gt_meas[end].velocity)
            println("   estimated linear: ", μ[8:10])
            println("   GT angular: ", fresh_gt_meas[end].angular_velocity)
            println("   estimated angular: ", μ[11:13])

        end



        localization_state = MyLocalizationType(0,0.0)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end


## Ellie Chason - start - ##

# Based off reached_target in map.jl
function find_current_segment(pos, map::Dict{Int, VehicleSim.RoadSegment})
    for (seg_id, seg) in map
        A = seg.lane_boundaries[2].pt_a
        B = seg.lane_boundaries[2].pt_b
        C = seg.lane_boundaries[3].pt_a
        D = seg.lane_boundaries[3].pt_b
        min_x = min(A[1], B[1], C[1], D[1])
        max_x = max(A[1], B[1], C[1], D[1])
        min_y = min(A[2], B[2], C[2], D[2])
        max_y = max(A[2], B[2], C[2], D[2])

        if min_x ≤ pos[1] ≤ max_x && min_y ≤ pos[2] ≤ max_y
            return seg_id
        end
    end
end

## Ellie Chason - end - ##

function perception(cam_meas_channel, localization_state_channel, perception_state_channel, shutdown_channel)
    # set up stuff
    while true
        
        fetch(shutdown_channel) && break

        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        latest_localization_state = fetch(localization_state_channel)
        
        # process bounding boxes / run ekf / do what you think is good

        perception_state = MyPerceptionType(0,0.0)
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
    end
end



function decision_making(localization_state_channel, 
    perception_state_channel, 
    target_segment_channel,
    shutdown_channel,
    map, 
    socket)
    # do some setup
    println("In decision")
    # --- begin motion planning ---
    # function to compute midpoints for a one lane road segment
    function compute_midpoints(segment)
        a1 = segment.lane_boundaries[1].pt_a
        b1 = segment.lane_boundaries[1].pt_b
        a2 = segment.lane_boundaries[2].pt_a
        b2 = segment.lane_boundaries[2].pt_b
        a_mid = (a1 + a2)/2
        b_mid = (b1 + b2)/2
        [a_mid, b_mid] #2X2 matrix
    end
    #given a list of segments
    path = [] #this will be the list of segments returned by routing function
    polyline = [] #polyline we create
    for id in path
        pt = compute_midpoints(map[id])
        push!(polyline, pt)
    end
    #now we can do PID controller on polyline
    alpha = [0.0, 0.0]
    current_segment_index = 1
    println("in decision")

    while true

        fetch(shutdown_channel) && break

        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)
        c1 = latest_localization_state[1]
        c2 = latest_localization_state[2]
        θ = VehicleSim.extract_yaw_from_quaternion(latest_localization_state[4:7])
        v = norm(latest_localization_state[8:9])

        lookahead_radius = v * ls
        current_segment = polyline[current_segment_index]
        p1 = current_segment[1]
        p2 = current_segment[2]
        a = (p2[1] - p1[1])^2 + (p2[2] - p1[2])^2
        b = 2 * ((p2[1] - p1[1]) * (p1[1] - c1) + (p2[2] - p1[2]) * (p1[2] - c2))
        c = (p1[1] - c1)^2 + (p1[2] - c2)^2 - lookahead_radius^2 

        discriminant = b^2 - 4 * a * c

        if discriminant >= 0
            sqrt_disc = sqrt(discriminant)
            t_upper = (-b + sqrt_disc) / (2 * a)
            t_lower = (-b - sqrt_disc) / (2 * a)
            valid_t = filter(t -> -0.05 ≤ t ≤ 1.1, [t_upper, t_lower])
            t = isempty(valid_t) ? -1 : first(valid_t)
        end

        center = SVector(c1, c2)
        q = SVector((t * (current_segment.p2 - current_segment.p1) + current_segment.p1)) - center

        heading = [cos(θ); sin(θ)]
        dot_value = dot(q, heading) / (norm(q) * norm(heading))
        alpha = acos(clamp(dot_value, -1, 1))  # Angle magnitude

        # Use cross product to determine sign
        cross_value = heading[1] * q[2] - heading[2] * q[1]  # 2D cross product determinant
        alpha *= sign(cross_value)

        turn = atan((2 * L * sin(alpha)) / lookahead_radius)
        result = [turn, 1.0]
        if v > 6
            result = [turn, -1.0]
        end

        # figure out what to do ... setup motion planning problem etc
        steering_angle = turn
        # if path[current_segment_index].lane_types == stop_sign
        target_vel = 0.0
        cmd = (steering_angle, target_vel, true)
        serialize(socket, cmd)
    end

end

function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)
    target_segment_channel = Channel{Int}(1)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)

    shutdown_channel = Channel{Bool}(1)
    put!(shutdown_channel, false)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    error_mon = errormonitor(@async while true
        # This while loop reads to the end of the socket stream (makes sure you
        # are looking at the latest messages)
        fetch(shutdown_channel) && break
        sleep(0.001)
        local measurement_msg
        received = false
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
                received = true
            else
                break
            end
        end
        !received && continue
        target_map_segment = measurement_msg.target_segment
        ego_vehicle_id = measurement_msg.vehicle_id
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
    end)

    tasks = []
    push!(tasks, error_mon)
    push!(tasks, @async localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel))
    push!(tasks, @async perception(cam_channel, localization_state_channel, perception_state_channel))
    push!(tasks, @async decision_making(localization_state_channel, perception_state_channel, target_segment_channel, shutdown_channel, map, socket))
    push!(tasks, @async shutdown_listener(shutdown_channel, tasks))

    for t in tasks
        wait(t)
    end
end

function shutdown_listener(shutdown_channel, tasks)
    info_string = 
        "***************
      CLIENT COMMANDS
      ***************
            -Press 'q' to shutdown threads. 
    "
    @info info_string
    while true
        sleep(0.1)
        key = get_c()

        if key == 'q'
            # terminate threads
            take!(shutdown_channel)
            println("Terminating threads")
            put!(shutdown_channel, true)
            return
        end
    end
end