mutable struct TrackedObstacle
    id::Int
    x::SVector{4, Float64}            # [px, py, vx, vy]
    P::SMatrix{4, 4, Float64}         # Covariance
    last_seen::Float64
    confidence::Float64
end

struct ObstacleDetection
    position::SVector{3, Float64}    # estimated position
    size::SVector{3, Float64}        # estimated size (length, width, height)
    velocity::SVector{2, Float64}    # estimated velocity (x, y)
    confidence::Float64              # detection confidence [0,1]
    id::Int                          # tracking ID (if available, 0 otherwise)
end

struct LaneMarking
    points::Vector{SVector{2, Float64}}  # points defining the lane marking
    type::Symbol                         # :left, :right, :center
    confidence::Float64                  # detection confidence [0,1]
end

struct MyLocalizationType
    # TODO: add timestamp and perhaps orientation
    position::SVector{3, Float64}
    orientation::SVector{4, Float64}
end

struct MyPerceptionType
    timestamp::Float64                    # timestamp of this perception state
    obstacles::Vector{ObstacleDetection}  # detected obstacles
    lane_markings::Vector{LaneMarking}    # detected lane markings
end

function h_obs(x::SVector{4, Float64})
    return x[1:2]
end

function Jac_h_obs()
    return @SMatrix [1.0 0.0 0.0 0.0;
                     0.0 1.0 0.0 0.0]
end

function ekf_predict!(track::TrackedObstacle, dt::Float64)
    A = @SMatrix [1.0 0.0 dt  0.0;
                  0.0 1.0 0.0 dt;
                  0.0 0.0 1.0 0.0;
                  0.0 0.0 0.0 1.0]
    Q = 0.05 * I(4)
    track.x = A * track.x
    track.P = A * track.P * A' + Q
end

function ekf_update!(track::TrackedObstacle, z::SVector{2, Float64})
    H = Jac_h_obs()
    R = 0.5 * I(2)
    y = z - h_obs(track.x)
    S = H * track.P * H' + R
    K = track.P * H' * inv(S)
    track.x = track.x + K * y
    track.P = (I(4) - K * H) * track.P
end

function associate_tracks(detections::Vector{ObstacleDetection}, tracks::Vector{TrackedObstacle})
    n = length(detections)
    m = length(tracks)

    if n == 0 || m == 0
        return zeros(Int, n)  # no matches if either is empty
    end

    cost_matrix = fill(1e6, n, m)

    for i in 1:n
        for j in 1:m
            d = norm(detections[i].position[1:2] - tracks[j].x[1:2])
            cost_matrix[i, j] = d
        end
    end

    assignment = hungarian(cost_matrix)
    return assignment
end

function convert_gt_to_obstacles(gt_measurements::Vector{GroundTruthMeasurement})
    obstacles = ObstacleDetection[]
    for gt in gt_measurements
        push!(obstacles, ObstacleDetection(
            gt.position,
            gt.size,
            gt.velocity[1:2],  # drop z for 2D comparison
            1.0,               # confidence = 1.0 for GT
            gt.vehicle_id      # ID match is possible
        ))
    end
    return obstacles
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
        # Transform GT messages into a comparable structure
        gt_detections = convert_gt_to_obstacles(fresh_gt_meas)

        # Send ground-truth detections into a new channel for evaluation
        if isready(gt_eval_channel)
            take!(gt_eval_channel)
        end
        put!(gt_eval_channel, gt_detections)
        
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
    @info "IN localization"
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

        fetch(shutdown_channel) && break

        # for k = 1:10
        isready(shutdown_channel) && break
        fresh_gps_meas = []
        #@info("Channel size: ", length(gps_channel))
        #@info("taking a meas")
        # meas = take!(gps_channel)
        # @info(meas)
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
        
        # process measurements
        take!(localization_state_channel)

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
            # @info("Timestep ", k, ":")
            # #@info("   Ground truth (x,y): ", xₖ[1:2])
            # @info("   Ground truth 2 (x,y): ", fresh_gt_meas[end])
            # @info("   Estimated (x,y): ", μ[1:3])
            # #@info("   Ground truth v: ", xₖ[3])
            # @info("   estimated q: ", μ[4:7])
            # #@info("   Ground truth θ: ", xₖ[4])
            # @info("   estimated linear: ", μ[8:10])
            # @info("   estimated angular: ", μ[11:13])
            # @info("   measurement received: ", zₖ)
            # @info("   Uncertainty measure (det(cov)): ", det(Σ))

            
            @info "   Ground truth (x,y): $(μs[2][1:3])"
            @info @info "   estimated: $(μ[1:3])"

        end



        localization_state = MyLocalizationType(μ[1:3], μ[4:7])
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end

function perspective_projection(point_3d, focal_length)
    # Simple perspective projection
    x = focal_length * point_3d[1] / point_3d[3]
    y = focal_length * point_3d[2] / point_3d[3]
    return SVector{2, Float64}(x, y)
end

function pixel_to_world(localization_state, cam_meas, box)
    # Create camera transformation matrix
    cam_id = cam_meas.camera_id
    
    # Get camera transform
    T_body_cam = VehicleSim.get_cam_transform(cam_id)
    T_cam_camrot = VehicleSim.get_rotated_camera_transform()
    T_body_camrot = VehicleSim.multiply_transforms(T_body_cam, T_cam_camrot)
    
    # Get world to body transform
    R = VehicleSim.Rot_from_quat(localization_state.orientation)
    T_world_body = [R localization_state.position; 0 0 0 1]
    
    # Get world to camera transform
    T_world_camrot = T_world_body * [T_body_camrot; 0 0 0 1]
    
    # Extract bounding box coordinates
    top, left, bottom, right = box
    
    # Convert to metric coordinates in camera frame
    pixel_len = cam_meas.pixel_length
    focal_len = cam_meas.focal_length
    image_width = cam_meas.image_width
    image_height = cam_meas.image_height
    
    # Convert pixel coordinates to camera coordinates
    cam_left = (left - image_width/2) * pixel_len
    cam_right = (right - image_width/2) * pixel_len
    cam_top = (top - image_height/2) * pixel_len
    cam_bottom = (bottom - image_height/2) * pixel_len
    
    # Assume a fixed depth for objects
    depth = 20.0  
    
    # Project to 3D points in camera frame
    p1 = SVector{3, Float64}(cam_left * depth / focal_len, cam_top * depth / focal_len, depth)
    p2 = SVector{3, Float64}(cam_right * depth / focal_len, cam_bottom * depth / focal_len, depth)
    
    # Center of the bounding box
    p_center = (p1 + p2) / 2
    
    # Convert to world coordinates
    p_center_homogeneous = T_world_camrot * [p_center; 1]
    p_world = SVector{3, Float64}(p_center_homogeneous[1:3])
    
    # Calculate approximate size
    width = abs(cam_right - cam_left) * depth / focal_len
    height = abs(cam_bottom - cam_top) * depth / focal_len
    
    # Assume rectangular object
    size = SVector{3, Float64}(width, height, (width + height) / 2)
    
    return p_world, size
end
## Ellie Chason - start - ##

function compute_bbox(seg::VehicleSim.RoadSegment)
    xs = Float64[]
    ys = Float64[]
    for lb in seg.lane_boundaries
        push!(xs, lb.pt_a[1])
        push!(ys, lb.pt_a[2])
        push!(xs, lb.pt_b[1])
        push!(ys, lb.pt_b[2])
    end
    return (minimum(xs), maximum(xs), minimum(ys), maximum(ys))
end

function point_in_bbox(pos::SVector{2,Float64}, bbox::Tuple{Float64,Float64,Float64,Float64})
    x, y = pos[1], pos[2]
    xmin, xmax, ymin, ymax = bbox
    return (xmin ≤ x ≤ xmax) && (ymin ≤ y ≤ ymax)
end

function find_current_segment(localization_state::MyLocalizationType, all_segs::Dict{Int, VehicleSim.RoadSegment})
    pos = SVector{2,Float64}(Float64(localization_state.field1), localization_state.field2)
    for (id, seg) in all_segs
        bbox = compute_bbox(seg)
        if point_in_bbox(pos, bbox)
            return id
        end
    end
    return nothing  # Return nothing if no segment is found.
end

## Ellie Chason - end - ##

function perception(cam_meas_channel, localization_state_channel, perception_state_channel, shutdown_channel)
    tracks = TrackedObstacle[]
    next_track_id = 1
    
    while true
        fetch(shutdown_channel) && break

        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end
        
        # Skip if no measurements
        if isempty(fresh_cam_meas)
            sleep(0.01)
            continue
        end
        
        latest_localization_state = fetch(localization_state_channel)
        current_time = fresh_cam_meas[end].time

    
        # Convert bounding boxes to world positions
        detections = ObstacleDetection[]
        for cam_meas in fresh_cam_meas
            for box in cam_meas.bounding_boxes
                pos, size = pixel_to_world(latest_localization_state, cam_meas, box)
                push!(detections, ObstacleDetection(pos, size, SVector(0.0, 0.0), 0.8, 0)) # 0 is placeholder id
            end
        end

        # Predict all tracks
        for track in tracks
            dt = current_time - track.last_seen
            ekf_predict!(track, dt)
        end

        # Data association
        assignment = associate_tracks(detections, tracks)

        assigned_tracks = Set{Int}()
        for i in 1:length(detections)
            j = assignment[i]
            if j != 0
                ekf_update!(tracks[j], detections[i].position[1:2])
                tracks[j].last_seen = current_time
                assigned_tracks |= Set([j])
                detections[i] = ObstacleDetection(detections[i].position, detections[i].size, tracks[j].x[3:4], detections[i].confidence, tracks[j].id)
            else
                pos = detections[i].position[1:2]
                new_track = TrackedObstacle(next_track_id, SVector(pos[1], pos[2], 0.0, 0.0), I(4), current_time, 0.8)
                detections[i] = ObstacleDetection(detections[i].position, detections[i].size, new_track.x[3:4], detections[i].confidence, next_track_id)
                push!(tracks, new_track)
                next_track_id += 1
            end
        end

        # Prune old tracks
        tracks = [t for t in tracks if current_time - t.last_seen < 1.0]

        # Create perception state
        perception_state = MyPerceptionType(
            current_time,  # Use latest measurement time
            detections,
            Vector{LaneMarking}()  # Lane detection not implemented here
        )
        
        # Update perception state channel
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
        
        sleep(0.01)  # Avoid busy waiting
    end
end

function decision_making(localization_state_channel, 
        perception_state_channel, 
        target_segment_channel,
        shutdown_channel,
        map, 
        socket)
    # do some setup
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
    gt_eval_channel = Channel{Vector{ObstacleDetection}}(1)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)
    target_segment_channel = Channel{Int}(1)
    shutdown_channel = Channel{Bool}(1)
    put!(shutdown_channel, false)
    localization_state_channel = Channel{MyLocalizationType}(1)
    #perception_state_channel = Channel{MyPerceptionType}(1)

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
    else
        errormonitor(@async localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel))

        errormonitor(@async perception(cam_channel, 
                      localization_state_channel, 
                      perception_state_channel, 
                      shutdown_channel))
    end



    errormonitor(@async decision_making(localization_state_channel, 
                           perception_state_channel, 
                           target_segment_channel, 
                           shutdown_channel,
                           map, 
                           socket))

    #Perception testing
    
errormonitor(@async begin
    while true
        @info "TEST PERCEPTION"
        sleep(1.0)  
        fetch(shutdown_channel) && break

        # Add debug info to track channel readiness
        @info "Channel status: gt_channel($(isready(gt_channel))), perception_state_channel($(isready(perception_state_channel))), gt_eval_channel($(isready(gt_eval_channel)))"
        
        # Process ground truth data if available
        if isready(gt_channel)
            @info "Processing ground truth data"
            gt_meas = GroundTruthMeasurement[]
            while isready(gt_channel)
                push!(gt_meas, take!(gt_channel))
            end
            
            # Add try-catch to catch potential errors in convert_gt_to_obstacles
            try
                gt_detections = convert_gt_to_obstacles(gt_meas)
                @info "Converted $(length(gt_detections)) ground truth detections"
                
                # Take existing value if channel is ready to avoid overflow
                if isready(gt_eval_channel)
                    take!(gt_eval_channel)
                end
                put!(gt_eval_channel, gt_detections)
            catch e
                @error "Error converting ground truth measurements: $e"
            end
        end

        # Perception evaluation
        try
            if isready(perception_state_channel)
                @info "Perception state channel is ready"
                perception = fetch(perception_state_channel)
                @info "Fetched perception with $(length(perception.obstacles)) obstacles"
                
                if isready(gt_eval_channel)
                    @info "Ground truth eval channel is ready"
                    gt = fetch(gt_eval_channel)
                    est = perception.obstacles

                    @info "Comparing perception to ground truth:"
                    @info "   # Perceived: $(length(est)), # GT: $(length(gt))"

                    
                    if !isempty(est) && !isempty(gt)
                        try
                            dists = Float64[]
                            for e in est
                                e_dists = [norm(e.position - g.position) for g in gt]
                                if !isempty(e_dists)
                                    push!(dists, minimum(e_dists))
                                end
                            end
                            
                            if !isempty(dists)
                                avg_error = sum(dists) / length(dists)
                                @info "   Avg nearest neighbor error: $(round(avg_error, digits=2)) meters"
                            else
                                @info "   No valid distance measurements"
                            end
                        catch e
                            @error "Error calculating distances: $e"
                        end
                    else
                        @info "   Either perceived or ground truth obstacles are empty"
                    end
                else
                    @info "Ground truth eval channel is not ready"
                end
            else
                @info "Perception state channel is not ready"
            end
        catch e
            @error "Error in perception evaluation: $e"
        end
    end
end)

    shutdown_listener(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel)
end

function shutdown_listener(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel)
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
            take!(shutdown_channel)
            put!(shutdown_channel, true)
            break
        end
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
            -Press 'q' to shutdown threads. 
    "
    @info info_string
    while true
        sleep(0.1)
        key = get_c()

        if key == 'q'
            # terminate threads
            @info("Terminating threads")
            put!(shutdown_channel, true)

            
            return
        end
    end
end