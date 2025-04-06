struct MyLocalizationType
    # TODO: add timestamp and perhaps orientation
    field1::Int
    field2::Float64
end

struct MyPerceptionType
    field1::Int
    field2::Float64
end

function process_gt(
        gt_channel,
        shutdown_channel,
        localization_state_channel,
        perception_state_channel)

    while true
        fetch(shutdown_channel) && break

        fresh_gt_meas = []
        while isready(gt_channel)
            meas = take!(gt_channel)
            push!(fresh_gt_meas, meas)
        end

        # process the fresh gt_measurements to produce localization_state and
        # perception_state
        
        take!(localization_state_channel)
        put!(localization_state_channel, new_localization_state_from_gt)
        
        take!(perception_state_channel)
        put!(perception_state_channel, new_perception_state_from_gt)
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
    println("IN localization")
    # Set up algorithm / initialize variables
    # process measurements
    #TODO change these values to reflect appropriate uncertainties for each type of measurement
    proc_cov = Diagonal([0.05, 0.05, 0.01, 0.01, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01])
    gt_states = [zeros(13),] # ground truth states that we will try to estimate
    timesteps = []
    last_timestamp = time()

    #TODO change these values to reflect appropriate uncertainties for each type of measurement
    meas_cov = Diagonal([0.2, 0.1, 0.1, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
    #meas_cov_imu = Diagonal([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])

    #what should this matrix be???
    #sqrt of these values *2, our mean should be within +/- these values with 95% confidence
    Σs = Matrix{Float64}[Diagonal([25,25,25,0.01,0.01,0.01,0.01,1,1,1,0.01,0.01,0.01]),]

    x_prev = zeros(13)
    zs = Vector{Float64}[]

    while true
    # for k = 1:10
        isready(shutdown_channel) && break
        fresh_gps_meas = []
        #println("Channel size: ", length(gps_channel))
        #println("taking a meas")
        # meas = take!(gps_channel)
        # println(meas)
        while !isready(gps_channel)
            sleep(0.001)
        end
        
        while isready(gps_channel)
            isready(shutdown_channel) && break
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end

        fresh_imu_meas = []
        while !isready(imu_channel)
            sleep(0.001)
        end
        while isready(imu_channel)
            isready(shutdown_channel) && break
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end

        fresh_gt_meas = []
        while !isready(gt_channel)
            sleep(0.001)
        end
        while isready(gt_channel)
            isready(shutdown_channel) && break
            meas = take!(gt_channel)
            push!(fresh_gt_meas, meas)
        end

        # Dynamically calculate the time step Δ
        current_timestamp = time()
        Δ = current_timestamp - last_timestamp
        last_timestamp = current_timestamp

        #TODO Get a better estimate of these values. Adjust position to be from initial GPS measurement
        #TODO add in these measurements into μs (velocities can remain 0)
        alpha = fresh_gps_meas[end].heading
        # μs = Diagonal([fresh_gps_meas[end].long, fresh_gps_meas[end].lat, 1.0, cos(alpha/2), 0, 0, sin(alpha/2), fresh_imu_meas[end].linear_vel, fresh_imu_meas[end].angular_vel]) #TODO: Gloria: is this meant to be a matrix or vector
        μs = [[fresh_gps_meas[end].long, fresh_gps_meas[end].lat, 2.65, cos(alpha/2), 0, 0, sin(alpha/2), fresh_imu_meas[end].linear_vel[1], fresh_imu_meas[end].linear_vel[2], fresh_imu_meas[end].linear_vel[3], fresh_imu_meas[end].angular_vel[1], fresh_imu_meas[end].angular_vel[2], fresh_imu_meas[end].angular_vel[3]]]
        linear_velocity = fresh_imu_meas[end].linear_vel
        angular_velocity = fresh_imu_meas[end].angular_vel
        #Δ = 0.1
        position = [fresh_gps_meas[end].long, fresh_gps_meas[end].lat, 1.0]
        q = [cos(alpha/2), 0, 0, sin(alpha/2)]

        # TODO We need to figure out an appropriate amount of uncertainty (proc_cov) a couple centimeters for position, add a bit for velocities and heading
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

        # Σ = (Σ_hat \ I + C' * (meas_var \ I) * C) \ I
        Σ = inv(inv(Σ_hat) + C' * inv(meas_cov) * C)
        μ = Σ*(inv(Σ_hat) *μ_hat + C'*(inv(meas_cov))*(zₖ - d))
        push!(μs, μ)
        push!(Σs, Σ)
        push!(zs, zₖ)

        #zₖ = h_imu(xₖ)
        #C_imu = Jac_h_imu(μ_hat)
        #d_imu = h_imu(μ_hat) - C_imu*μ_hat

        # Σ = (Σ_hat \ I + C' * (meas_var \ I) * C) \ I
        # Σ = inv(inv(Σ_hat) + C_imu' * inv(meas_cov_imu) * C_imu)
        # μ = Σ*(inv(Σ_hat) *μ_hat + C_imu'*(inv(meas_cov_imu))*(zₖ - d_imu))
        # push!(μs, μ)
        # push!(Σs, Σ)
        # push!(zs, zₖ)

        push!(gt_states, xₖ)
        push!(timesteps, Δ)

        if true
            # println("Timestep ", k, ":")
            # #println("   Ground truth (x,y): ", xₖ[1:2])
            # println("   Ground truth 2 (x,y): ", fresh_gt_meas[end])
            # println("   Estimated (x,y): ", μ[1:3])
            # #println("   Ground truth v: ", xₖ[3])
            # println("   estimated q: ", μ[4:7])
            # #println("   Ground truth θ: ", xₖ[4])
            # println("   estimated linear: ", μ[8:10])
            # println("   estimated angular: ", μ[11:13])
            # println("   measurement received: ", zₖ)
            # println("   Uncertainty measure (det(cov)): ", det(Σ))

            println("   Ground truth (x,y): ", μs[2][1:3])
            println("   estimated: ", μ[1:3])

        end



        localization_state = MyLocalizationType(0,0.0)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end

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

function get_segment_center(map, seg_id)
    if !haskey(map, seg_id)
        return SVector(0.0, 0.0)  # Default if segment not found
    end
    
    seg = map[seg_id]
    # Calculate center point from lane boundaries
    if length(seg.lane_boundaries) >= 2
        lb1 = seg.lane_boundaries[1]
        lb2 = seg.lane_boundaries[end]
        pt_a = lb1.pt_a
        pt_b = lb1.pt_b
        pt_c = lb2.pt_a
        pt_d = lb2.pt_b
        return 0.25 * (pt_a + pt_b + pt_c + pt_d)
    else
        # Fallback if segment doesn't have enough lane boundaries
        return SVector(0.0, 0.0)
    end
end

function find_nearest_segment(map, position)
    # Find the nearest road segment to the given position
    # position is assumed to be a 2D vector (x, y)
    
    nearest_segment_id = -1
    min_distance = Inf
    
    for (seg_id, segment) in map
        # Calculate distances to all lane boundaries in this segment
        for boundary in segment.lane_boundaries
            # Calculate distance to line segment between pt_a and pt_b
            pt_a = boundary.pt_a
            pt_b = boundary.pt_b
            
            # Vector from pt_a to pt_b
            v_ab = pt_b - pt_a
            # Vector from pt_a to position
            v_ap = position - pt_a
            
            # Calculate projection of v_ap onto v_ab
            len_ab_squared = sum(v_ab .^ 2)
            
            # Avoid division by zero
            if len_ab_squared < 1e-10
                continue
            end
            
            # Calculate projection parameter
            t = max(0, min(1, sum(v_ap .* v_ab) / len_ab_squared))
            
            # Calculate closest point on the line segment
            closest_point = pt_a + t * v_ab
            
            # Calculate distance to the closest point
            distance = norm(position - closest_point)
            
            if distance < min_distance
                min_distance = distance
                nearest_segment_id = seg_id
            end
        end
    end
    
    return nearest_segment_id
end

function plan_route(map, current_segment_id, target_segment_id)
    # Return empty route if we're already at the target
    if current_segment_id == target_segment_id
        return [current_segment_id]
    end
    
    # A* search to find an efficient path
    # Using a combination of path length and estimated distance to target as heuristic
    function heuristic(seg_id)
        # Calculate Euclidean distance between current segment and target
        current_center = get_segment_center(map, seg_id)
        target_center = get_segment_center(map, target_segment_id)
        return norm(current_center - target_center)
    end
    
    # Priority queue for A* - using tuple of (priority, segment_id, path)
    # Priority = g + h where g = path length, h = heuristic estimate to goal
    
    # Using a simple Vector and sort it after each insertion
    # Maybe will switch to a priority queue data structure
    open_set = [(0.0 + heuristic(current_segment_id), current_segment_id, [current_segment_id])]
    
    # Track path costs (g values) and visited nodes
    g_scores = Dict{Int, Float64}()
    g_scores[current_segment_id] = 0.0
    visited = Set{Int}()
    
    while !isempty(open_set)
        # Get node with lowest f_score (priority)
        sort!(open_set, by = x -> x[1])
        (_, segment_id, path) = popfirst!(open_set)
        
        # Skip if already visited (found a better path)
        if segment_id in visited
            continue
        end
        
        # Check if we reached the target
        if segment_id == target_segment_id
            return path
        end
        
        push!(visited, segment_id)
        
        # Check if the segment has children
        if haskey(map, segment_id)
            current_g = g_scores[segment_id]
            
            for child_id in map[segment_id].children
                # Calculate edge cost - can be sophisticated based on road properties
                # Use 1.0 for standard edges for now
                # and higher costs for special segments like intersections or stop signs
                edge_cost = 1.0
                
                if haskey(map, child_id)
                    if contains_lane_type(map[child_id], intersection)
                        edge_cost = 2.0  # Intersections are more costly
                    elseif contains_lane_type(map[child_id], stop_sign)
                        edge_cost = 1.5  # Stop signs have medium cost
                    elseif contains_lane_type(map[child_id], loading_zone)
                        edge_cost = 0.5  # Prefer loading zones (target type)
                    end
                end
                
                # New path cost to this child
                new_g = current_g + edge_cost
                
                # Only consider this path if it's better than any previous path to this node
                if !haskey(g_scores, child_id) || new_g < g_scores[child_id]
                    g_scores[child_id] = new_g
                    new_path = vcat(path, [child_id])
                    f_score = new_g + heuristic(child_id)
                    push!(open_set, (f_score, child_id, new_path))
                end
            end
        end
    end
    
    # No path found
    return Int[]
end

function decision_making(localization_state_channel, 
    perception_state_channel, 
    target_segment_channel,
    shutdown_channel,
    map, 
    socket)
# do some setup
current_route = Int[]
current_segment_id = -1  # Will be determined from localization
target_segment_id = target_road_segment_id
route_index = 1


# Control parameters
default_speed = 5.0
slow_speed = 2.0
stop_distance = 10.0  # Distance to slow down when approaching target or intersection

# Tracking the last known GPS position
last_known_position = SVector(0.0, 0.0)
vehicle_heading = 0.0

# Helper function to calculate segment center
function get_segment_center(seg_id)
    if !haskey(map, seg_id)
        return SVector(0.0, 0.0)  # Default if segment not found
    end
    
    seg = map[seg_id]
    # Calculate center point from lane boundaries
    if length(seg.lane_boundaries) >= 2
        lb1 = seg.lane_boundaries[1]
        lb2 = seg.lane_boundaries[end]
        pt_a = lb1.pt_a
        pt_b = lb1.pt_b
        pt_c = lb2.pt_a
        pt_d = lb2.pt_b
        return 0.25 * (pt_a + pt_b + pt_c + pt_d)
    else
        # Fallback if segment doesn't have enough lane boundaries
        return SVector(0.0, 0.0)
    end
end

# Get direction to target from current position
function get_direction_to_next_segment(current_id, next_id)
    current_center = get_segment_center(current_id)
    next_center = get_segment_center(next_id)
    
    # Calculate vector from current to next
    direction_vector = next_center - current_center
    
    # Calculate angle in radians
    angle = atan(direction_vector[2], direction_vector[1])
    
    return angle
end

while true

    fetch(shutdown_channel) && break

    latest_localization_state = fetch(localization_state_channel)
    latest_perception_state = fetch(perception_state_channel)

    # figure out what to do ... setup motion planning problem etc
    steering_angle = 0.0
    target_vel = 0.0
    cmd = (steering_angle, target_vel, true)
    serialize(socket, cmd)
end
end

function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0), port=4444; use_gt=false)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{MyLocalizationType}(1)

    target_segment_channel = Channel{Int}(1)
    #perception_state_channel = Channel{MyPerceptionType}(1)

    shutdown_channel = Channel{Bool}(1)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)


    put!(target_segment_channel, target_map_segment)
    error_mon = errormonitor(@async while true

        # This while loop reads to the end of the socket stream (makes sure you
        # are looking at the latest messages)
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
        old_target_segment = fetch(target_segment_channel)
        if target_map_segment ≠ old_target_segment
            take!(target_segment_channel)
            put!(target_segment_channel, target_map_segment)
        end
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

    if use_gt
        @async process_gt(gt_channel,
                      shutdown_channel,
                      localization_state_channel,
                      perception_state_channel)
    end
    tasks = []
    # push!(tasks, error_mon)
    push!(tasks, @async localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel))
    # push!(@async perception(cam_channel, localization_state_channel, perception_state_channel))
    # push!(tasks, @async decision_making(localization_state_channel, perception_state_channel, map, socket))
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
            -Make sure focus is on this terminal window. Then:
            -Press 'q' to shutdown threads. 
    "
    @info info_string
    while true
        sleep(0.1)
        key = get_c()

        if key == 'q'
            # terminate threads
            println("Terminating threads")
            put!(shutdown_channel, true)     
            return
        end
    end
end