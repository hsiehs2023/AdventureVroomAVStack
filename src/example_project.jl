using VehicleSim

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
    velocity::SVector{3, Float64}
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

function ekf_update!(track::TrackedObstacle, z)
    z_svec = SVector{2, Float64}(z)
    H = Jac_h_obs()
    R = 0.5 * I(2)
    y = z_svec - h_obs(track.x)
    S = H * track.P * H' + R
    K = track.P * H' * inv(S)
    track.x = track.x + K * y
    track.P = (I(4) - K * H) * track.P
end

function cluster_detections(detections::Vector{ObstacleDetection}; threshold=2.0)
    n = length(detections)
    if n <= 1
        return detections
    end
    
    # Create clusters
    clusters = Vector{Vector{Int}}()
    assigned = falses(n)
    
    for i in 1:n
        if assigned[i]
            continue
        end
        
        # Start a new cluster
        cluster = [i]
        assigned[i] = true
        
        # Find all detections close to this one
        for j in (i+1):n
            if assigned[j]
                continue
            end
            
            if norm(detections[i].position - detections[j].position) < threshold
                push!(cluster, j)
                assigned[j] = true
            end
        end
        
        push!(clusters, cluster)
    end
    
    # Merge detections in each cluster
    merged_detections = Vector{ObstacleDetection}()
    for cluster in clusters
        if length(cluster) == 1
            push!(merged_detections, detections[cluster[1]])
        else
            # Average position, size and velocity
            avg_pos = sum(detections[i].position for i in cluster) / length(cluster)
            avg_size = sum(detections[i].size for i in cluster) / length(cluster)
            avg_vel = sum(detections[i].velocity for i in cluster) / length(cluster)
            
            # Take maximum confidence
            max_conf = maximum(detections[i].confidence for i in cluster)
            
            # Use existing ID if any detection has one, otherwise 0
            ids = [detections[i].id for i in cluster if detections[i].id != 0]
            id = isempty(ids) ? 0 : first(ids)
            
            push!(merged_detections, ObstacleDetection(
                avg_pos, avg_size, avg_vel, max_conf, id
            ))
        end
    end
    
    return merged_detections
end

function associate_tracks(detections::Vector{ObstacleDetection}, tracks::Vector{TrackedObstacle}; threshold=2.0)
    n = length(detections)
    m = length(tracks)

    if n == 0 || m == 0
        return zeros(Int, n)  # no matches if either is empty
    end

    cost_matrix = fill(1e6, n, m)

    for i in 1:n
        for j in 1:m
            d = norm(detections[i].position[1:2] - tracks[j].x[1:2])
            cost_matrix[i, j] = d < threshold ? d : 1e6  # only consider matches within threshold
        end
    end

    # Hungarian returns (assignment, cost) tuple 
    assignment = first(hungarian(cost_matrix))
    return assignment
end

function jacobian_projection_analytic(point_3d::SVector{3,Float64}, focal_length::Float64)
    X, Y, Z = point_3d
    fx = focal_length
    J = @SMatrix [
        fx/Z    0     -fx*X/(Z^2);
         0     fx/Z   -fx*Y/(Z^2)
    ]
    return J
end

function numeric_jacobian(f, x::SVector{3,Float64}; ε=1e-6)
    n = length(x)
    m = length(f(x))
    J = zeros(m, n)
    for i in 1:n
        dx = zero(x)
        dx = dx + ε * (i == 1 ? SVector(1.0,0.0,0.0) : (i == 2 ? SVector(0.0,1.0,0.0) : SVector(0.0,0.0,1.0)))
        J[:, i] = (f(x + dx) - f(x - dx)) / (2ε)
    end
    return J
end

function test_projection_jacobian()
    point = SVector{3, Float64}(3.0, 4.0, 10.0)
    f = 800.0

    f_proj = p -> perspective_projection(p, f)

    J_analytic = jacobian_projection_analytic(point, f)
    J_numeric = numeric_jacobian(f_proj, point)

    println("Analytic Jacobian:")
    println(J_analytic)
    println("Numeric Jacobian:")
    println(J_numeric)
    println("Difference:")
    println(J_analytic - J_numeric)
end

function process_gt(
    gt_channel,
    shutdown_channel,
    localization_state_channel,
    perception_state_channel)


    function convert_gt_to_obstacles(gt_measurements)
        obstacles = ObstacleDetection[]
        for gt in gt_measurements
            # Extract position, checking for valid values
            position = if all(isfinite.(gt.position))
                gt.position
            else
                SVector{3, Float64}(0.0, 0.0, 0.0)
            end
            
            size = if isdefined(gt, :size) && all(isfinite.(gt.size))
                gt.size
            else
                SVector{3, Float64}(4.0, 2.0, 1.5)  # Default car size
            end
            
            # Extract velocity, checking for valid values
            velocity = if isdefined(gt, :velocity) && all(isfinite.(gt.velocity[1:2]))
                gt.velocity[1:2]
            else
                SVector{2, Float64}(0.0, 0.0)
            end
            
            # Create the obstacle detection
            obstacle = ObstacleDetection(
                position,
                size,
                velocity,
                1.0,  # Confidence = 1.0 for ground truth
                gt.vehicle_id  # Use vehicle ID as tracking ID
            )
            
            push!(obstacles, obstacle)
        end
        
        return obstacles
    end

    # Define the gt_eval_channel if it doesn't exist
    gt_eval_channel = Channel{Vector{ObstacleDetection}}(1)
    put!(gt_eval_channel, Vector{ObstacleDetection}())

    try
        while true
            if fetch(shutdown_channel)
                break
            end

            fresh_gt_meas = []
            
            while isready(gt_channel)
                meas = take!(gt_channel)
                push!(fresh_gt_meas, meas)
            end

            if !isempty(fresh_gt_meas)
                # Transform GT messages into obstacle detections
                try
                    
                    gt_detections = convert_gt_to_obstacles(fresh_gt_meas)
                
                    
                    # Send ground-truth detections into the eval channel for evaluation
                    if isready(gt_eval_channel)
                        take!(gt_eval_channel)
                    end
                    put!(gt_eval_channel, gt_detections)
                    
                    # Create a new localization state from ground truth
                    
                    if !isempty(fresh_gt_meas)
                        ego_gt = fresh_gt_meas[1]  # Just use the first one for simplicity
                        
                        # Extract position and orientation
                        position = if isdefined(ego_gt, :position) && all(isfinite.(ego_gt.position))
                            ego_gt.position
                        else
                            SVector{3, Float64}(0.0, 0.0, 0.0)
                        end
                        
                        orientation = if isdefined(ego_gt, :orientation) && all(isfinite.(ego_gt.orientation))
                            ego_gt.orientation
                        else
                            SVector{4, Float64}(1.0, 0.0, 0.0, 0.0)  # Identity quaternion
                        end
                        
                        new_localization_state = MyLocalizationType(
                            position,
                            orientation
                        )
                        
                        if isready(localization_state_channel)
                            take!(localization_state_channel)
                        end
                        put!(localization_state_channel, new_localization_state)
                    end
                    
                    # Create a new perception state with the obstacles
                    
                    new_perception_state = MyPerceptionType(
                        time(),
                        gt_detections,  # Use the converted detections
                        Vector{LaneMarking}()  # No lane markings for now
                    )
                    
                    if isready(perception_state_channel)
                        take!(perception_state_channel)
                    end
                    put!(perception_state_channel, new_perception_state)
                catch e
                
                end
            end
            
            sleep(0.01)  # Small sleep to avoid busy-waiting
        end
    catch e
    end
    
end


#Performs routing on current segment found from ground truth position (development only)
function routing(localization_state_channel, target_segment_id::Int64, map::Dict{Int, VehicleSim.RoadSegment})
    #Idea 1 for finding current position
    # gt_meas = fetch(gt_channel)
    gt_meas = fetch(localization_state_channel)
    pos = gt_meas.position
    
    #Idea 2 for finding current position
    #current_state = fetch(state_channel)
    #pos = current_state.q[5:6]

    current_segment_id = find_current_segment_routing(pos, map)
    println("current_segment_id ", current_segment_id)

    #println("Current Segment: ", current_segment_id)
    #println("Target Segment: ", target_segment_id)

    path = find_shortest_path(current_segment_id, target_segment_id, map)

    #println("Path: ", path)

    return path
end

#Performs routing on current segment found from ground truth position (development only)
function routing(localization_state_channel, target_segment_id::Int64, map::Dict{Int, VehicleSim.RoadSegment}, current_segment_id)

    println("Current Segment: ", current_segment_id)
    println("Target Segment: ", target_segment_id)

    path = find_shortest_path(current_segment_id, target_segment_id, map)

    return path
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
function find_shortest_path(current_segment_id, target_segment_id::Int64, map::Dict{Int, VehicleSim.RoadSegment})
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
    # Reconstruct the path
    path_ids = Int[]
    seg_id = target_segment_id
    while seg_id != current_segment_id
        push!(path_ids, seg_id)
        seg_id = prev[seg_id]
    end
    push!(path_ids, current_segment_id)
    reverse!(path_ids)
    println("pathids ", path_ids)

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
    time_counter = 0

    while true
        fetch(shutdown_channel) && break
        fresh_gps_meas = []
        while !isready(gps_channel)
            sleep(0.001)
            fetch(shutdown_channel) && break
        end
        
        while isready(gps_channel)
            fetch(shutdown_channel) && break
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end

        fresh_imu_meas = []
        while !isready(imu_channel)
            sleep(0.001)
            fetch(shutdown_channel) && break
        end
        while isready(imu_channel)
            fetch(shutdown_channel) && break
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end

        fresh_gt_meas = []
        while !isready(gt_channel)
            sleep(0.001)
            fetch(shutdown_channel) && break
        end
        while isready(gt_channel)
            fetch(shutdown_channel) && break
            meas = take!(gt_channel)
            push!(fresh_gt_meas, meas)
        end

        if isready(localization_state_channel)
            take!(localization_state_channel)
        end

        # Dynamically calculate the time step Δ
        current_timestamp = time()
        Δ = current_timestamp - last_timestamp
        last_timestamp = current_timestamp

        try
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

        if false #For testing error
            error = μ[1:3] - fresh_gt_meas[end].position
            println(error)
            # Append to a CSV file
            open("errors.csv", "a") do io
                writedlm(io, [time_counter error'], ',')  # Transpose error to make it 1 row
            end
            time_counter += 1

        end



        localization_state = MyLocalizationType(μ[1:3], μ[4:7], μ[8:10])
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    catch e
        @error "[localize] ERROR before writing localization_state: $e"
        for (i, frame) in enumerate(Base.catch_backtrace())
            println(stderr, "[$i] $(Base.show_backtrace_entry(frame))")
        end
    end
    end 
end


function perspective_projection(point_3d, focal_length)
    # Simple perspective projection
    x = focal_length * point_3d[1] / point_3d[3]
    y = focal_length * point_3d[2] / point_3d[3]
    return SVector{2, Float64}(x, y)
end

function pixel_to_world(localization_state, cam_meas, box)
    #@info "[pixel_to_world] Starting conversion with box: $box"
    
    try
        # Create camera transformation matrix
        cam_id = cam_meas.camera_id
        #@info "[pixel_to_world] Camera ID: $cam_id"
        
        # Get camera transform 
        local T_body_cam, T_cam_camrot, T_body_camrot
        try
            T_body_cam = VehicleSim.get_cam_transform(cam_id)
            T_cam_camrot = VehicleSim.get_rotated_camera_transform()
            T_body_camrot = VehicleSim.multiply_transforms(T_body_cam, T_cam_camrot)
            #@info "[pixel_to_world] Camera transforms created successfully"
        catch e
            #@error "[pixel_to_world] Error getting camera transforms: $e"
            # Provide default transforms to continue processing
            T_body_cam = [I(3) zeros(3); zeros(1,3) 1]
            T_cam_camrot = [I(3) zeros(3); zeros(1,3) 1]
            T_body_camrot = [I(3) zeros(3); zeros(1,3) 1]
        end
        
        # Get world to body transform
        local T_world_body  
        try
            #@info "[pixel_to_world] Creating world transform with orientation: $(localization_state.orientation)"
            local R
            if all(isfinite.(localization_state.orientation)) && any(localization_state.orientation .!= 0)
                R = VehicleSim.Rot_from_quat(localization_state.orientation)
            else
                #@warn "[pixel_to_world] Using identity rotation matrix due to invalid quaternion"
                R = Matrix{Float64}(I, 3, 3)
            end
            
            local pos
            if all(isfinite.(localization_state.position)) 
                pos = localization_state.position
            else
                #@warn "[pixel_to_world] Using zero position due to invalid position"
                pos = SVector{3, Float64}(0.0, 0.0, 0.0)
            end
            
            T_world_body = [R pos; 0 0 0 1]
            #@info "[pixel_to_world] World transform created successfully"
        catch e
            #@error "[pixel_to_world] Error creating world transform: $e"
            T_world_body = [I(3) zeros(3); zeros(1,3) 1]
        end
        
        # Get world to camera transform
        T_body_camrot_h = [T_body_camrot; 0 0 0 1]  # make 4×4
        T_world_camrot = T_world_body * T_body_camrot_h
        
        # Extract bounding box coordinates
        local top, left, bottom, right
        try
            top, left, bottom, right = box
            #@info "[pixel_to_world] Extracted box coordinates: top=$top, left=$left, bottom=$bottom, right=$right"
        catch e
            #@error "[pixel_to_world] Error extracting box coordinates: $e"
            # Use default values
            top, left, bottom, right = 0.0, 0.0, 100.0, 100.0
        end
        
        # Convert to metric coordinates in camera frame
        # Declare these variables in the outer scope
        local pixel_len = 0.001  # Default value
        local focal_len = 0.64   # Default value
        local image_width = 640  # Default value
        local image_height = 480 # Default value
        local cam_left, cam_right, cam_top, cam_bottom
        
        try
            
            pixel_len = cam_meas.pixel_length
            focal_len = cam_meas.focal_length
            image_width = cam_meas.image_width
            image_height = cam_meas.image_height
            
            #@info "[pixel_to_world] Camera params: pixel_len=$pixel_len, focal_len=$focal_len, width=$image_width, height=$image_height"
            
            # Convert pixel coordinates to camera coordinates
            cam_left = (left - image_width/2) * pixel_len
            cam_right = (right - image_width/2) * pixel_len
            cam_top = (top - image_height/2) * pixel_len
            cam_bottom = (bottom - image_height/2) * pixel_len
            
            #@info "[pixel_to_world] Camera coords: left=$cam_left, right=$cam_right, top=$cam_top, bottom=$cam_bottom"
        catch e
            #@error "[pixel_to_world] Error converting to camera coordinates: $e"
            # Use default values
            cam_left, cam_right, cam_top, cam_bottom = -1.0, 1.0, -1.0, 1.0
        end
        
        # Assume a fixed depth for objects 
        box_height = abs(bottom - top)
        
        depth = max(focal_len * 8.0 / (box_height * pixel_len), 10.0)
        #@info "[pixel_to_world] Estimated depth: $depth meters for box height: $box_height pixels"
        
        # Project to 3D points in camera frame 
        try
            p1 = SVector{3, Float64}(cam_left * depth / focal_len, cam_top * depth / focal_len, depth)
            p2 = SVector{3, Float64}(cam_right * depth / focal_len, cam_bottom * depth / focal_len, depth)
            
            # Center of the bounding box
            p_center = (p1 + p2) / 2
            #@info "[pixel_to_world] 3D camera point: $p_center"
            
            # Convert to world coordinates
            p_center_homogeneous = T_world_camrot * [p_center; 1]
            p_world = SVector{3, Float64}(p_center_homogeneous[1:3])
            #@info "[pixel_to_world] 3D world point: $p_world"
            
            # Calculate approximate size
            width = abs(cam_right - cam_left) * depth / focal_len
            height = abs(cam_bottom - cam_top) * depth / focal_len
            
            # Assume rectangular object with some depth
            size = SVector{3, Float64}(width, height, (width + height) / 2)
            #@info "[pixel_to_world] Object size: $size"
            
            return p_world, size
        catch e
            #@error "[pixel_to_world] Error projecting to 3D: $e"
            # Return default values as fallback
            return SVector{3, Float64}(0.0, 0.0, 20.0), SVector{3, Float64}(2.0, 2.0, 2.0)
        end
    catch e
        #@error "[pixel_to_world] Uncaught error in pixel_to_world: $e"
        Base.show_backtrace(stderr, catch_backtrace())
        # Return default values as fallback
        return SVector{3, Float64}(0.0, 0.0, 20.0), SVector{3, Float64}(2.0, 2.0, 2.0)
    end
end

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

# Based off reached_target in map.jl
function find_current_segment_routing(pos, map::Dict{Int, VehicleSim.RoadSegment})
    for (seg_id, seg) in map
        A = seg.lane_boundaries[1].pt_a
        B = seg.lane_boundaries[1].pt_b
        C = seg.lane_boundaries[2].pt_a
        D = seg.lane_boundaries[2].pt_b
        min_x = min(A[1], B[1], C[1], D[1])
        max_x = max(A[1], B[1], C[1], D[1])
        min_y = min(A[2], B[2], C[2], D[2])
        max_y = max(A[2], B[2], C[2], D[2])

        if min_x ≤ pos[1] ≤ max_x && min_y ≤ pos[2] ≤ max_y
            return seg_id
        end
    end
    println("UH OHHHH")
end

function perception(cam_meas_channel, localization_state_channel, perception_state_channel, shutdown_channel)
    @info "[perception] Running perception loop"
    tracks = TrackedObstacle[]
    next_track_id = 1
    last_positions = Dict{Int, Tuple{SVector{3, Float64}, Float64}}() # id -> (position, timestamp)
    
    try
        while true
            fetch(shutdown_channel) && break
            current_time = time()
            fresh_cam_meas = []

            # Wait for camera measurements with a short timeout
            if !isready(cam_meas_channel)
                sleep(0.01)
                continue
            end

            # Collect all available camera measurements
            while isready(cam_meas_channel)
                push!(fresh_cam_meas, take!(cam_meas_channel))
            end

            @info "[perception] Got $(length(fresh_cam_meas)) fresh camera measurements"

            if isempty(fresh_cam_meas)
                continue
            end

            detections = ObstacleDetection[]
            
            try
                latest_localization_state = fetch(localization_state_channel)
                
                for cam_meas in fresh_cam_meas
                    @info "[perception] Processing camera measurement with $(length(cam_meas.bounding_boxes)) boxes"
                    for box in cam_meas.bounding_boxes
                        try
                            pos, size = pixel_to_world(latest_localization_state, cam_meas, box)
                            
                            # Default velocity is zero
                            velocity = SVector{2, Float64}(0.0, 0.0)
                            
                            # Create detection with reasonable confidence
                            push!(detections, ObstacleDetection(pos, size, velocity, 0.8, 0))
                        catch e
                            @error "[perception] Failed to process box: $e"
                            
                        end
                    end
                end
                
                @info "[perception] Created $(length(detections)) initial obstacle detections"
            catch e
                @error "[perception] Failed to process camera measurements: $e"
                continue  # Skip this iteration if we can't process the measurements
            end

            # Only proceed with clustering if we have detections
            if !isempty(detections)
                detections = cluster_detections(detections)
                @info "[perception] After clustering: $(length(detections)) detections"

                # Apply EKF prediction to existing tracks
                for track in tracks
                    dt = current_time - track.last_seen
                    ekf_predict!(track, dt)
                end

                # Associate detections with existing tracks
                assignment = associate_tracks(detections, tracks)
                @info "[perception] Assignment result: $assignment"
                
                # Process each detection
                assigned_tracks = Set{Int}()
                
                for i in 1:length(detections)
                    track_idx = i <= length(assignment) ? assignment[i] : 0
                    
                    if track_idx != 0 && track_idx <= length(tracks)
                        # Update existing track with this detection
                        ekf_update!(tracks[track_idx], detections[i].position[1:2])
                        tracks[track_idx].last_seen = current_time
                        tracks[track_idx].confidence = max(tracks[track_idx].confidence, detections[i].confidence)
                        push!(assigned_tracks, track_idx)
                        
                        # Calculate velocity from EKF state
                        velocity = tracks[track_idx].x[3:4]
                        
                        # Update detection with track info
                        detections[i] = ObstacleDetection(
                            detections[i].position, 
                            detections[i].size, 
                            velocity,  
                            tracks[track_idx].confidence, 
                            tracks[track_idx].id)
                            
                        # Store position and time for velocity calculation
                        last_positions[tracks[track_idx].id] = (detections[i].position, current_time)
                    else
                        # Create new track
                        pos = detections[i].position[1:2]
                        x₀ = SVector{4, Float64}(pos[1], pos[2], 0.0, 0.0)
                        P₀ = Diagonal([0.5^2, 0.5^2, 5.0^2, 5.0^2])
                        
                        new_track = TrackedObstacle(
                            next_track_id,
                            x₀,
                            P₀,
                            current_time,
                            0.8
                        )
                        
                        # Update detection with new track ID
                        detections[i] = ObstacleDetection(
                            detections[i].position, 
                            detections[i].size, 
                            SVector{2, Float64}(0.0, 0.0),
                            detections[i].confidence, 
                            next_track_id)
                        
                        # Store position for future velocity calculation
                        last_positions[next_track_id] = (detections[i].position, current_time)
                        
                        push!(tracks, new_track)
                        next_track_id += 1
                    end
                end

                # Remove old tracks that haven't been seen recently
                tracks = [t for t in tracks if (current_time - t.last_seen < 2.0) || (t.confidence > 0.3)]
                @info "[perception] After cleanup: $(length(tracks)) active tracks"
            end

            # Create perception state with current detections
            perception_state = MyPerceptionType(
                current_time,
                detections,
                Vector{LaneMarking}()
            )

            # Update perception channel
            if isready(perception_state_channel)
                take!(perception_state_channel)
            end
            put!(perception_state_channel, perception_state)
            
            sleep(0.01)
        end
    catch e
        @error "[perception] CRASHED with error: $e"
        Base.show_backtrace(stderr, catch_backtrace())
    end
end


function decision_making(use_gt, localization_state_channel, 
    perception_state_channel, 
    target_segment_channel,
    shutdown_channel,
    map, 
    socket, gt_channel)
    # do some setup
    @info "In decision"
    sleep(0.5)
    try
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

    function compute_midpoint_target(segment)
        a1 = segment.lane_boundaries[length(segment.lane_boundaries)-1].pt_a
        b1 = segment.lane_boundaries[length(segment.lane_boundaries)-1].pt_b
        a2 = segment.lane_boundaries[end].pt_a
        b2 = segment.lane_boundaries[end].pt_b
        a_mid = (a1 + a2)/2
        b_mid = (b1 + b2)/2
        [a_mid, b_mid] #2X2 matrix
    end

    # ----- obstacle detection functions -----
    function is_obstacle_ahead(obstacles, ego_pos, ego_heading, lookahead_distance, path_width=3.5)
        closest_obstacle_distance = Inf
        obstacle_found = false
        
        for obstacle in obstacles
            # Skip invalid obstacles
            if !all(isfinite.(obstacle.position))
                continue
            end
            
            # Get 2D positions
            obs_pos_2d = SVector(obstacle.position[1], obstacle.position[2])
            ego_pos_2d = SVector(ego_pos[1], ego_pos[2])
            
            # Vector from ego to obstacle
            relative_pos = obs_pos_2d - ego_pos_2d
            distance = norm(relative_pos)
            
            # Project onto heading direction
            projection = dot(relative_pos, ego_heading)
            
            # Only consider obstacles ahead of us and within a reasonable distance
            # Add minimum distance threshold to ignore very close detections (likely false positives)
            if 8.0 < projection < lookahead_distance
                # Calculate lateral distance from our path
                lateral_vector = relative_pos - projection * ego_heading
                lateral_dist = norm(lateral_vector)
                
                # If obstacle is within our path width, it's ahead of us
                if lateral_dist < path_width && distance < closest_obstacle_distance
                    # Add confidence threshold to filter out low-confidence detections
                    # (assuming obstacles have a confidence field)
                    if hasfield(typeof(obstacle), :confidence) && obstacle.confidence < 0.6
                        continue
                    end
                    
                    obstacle_found = true
                    closest_obstacle_distance = distance
                    @info "[obstacle_detection] Obstacle ahead at distance $distance, lateral offset $lateral_dist"
                end
            end
        end
        
        return obstacle_found, closest_obstacle_distance
    end
    # ----- end obstacle detection functions -----

    target_segment = fetch(target_segment_channel) # Good testing target ids are 80 (road above the origin) and 27 (road we start on)
    path = nothing
    polyline = [] #polyline we create
    pt = nothing
    alpha = nothing
    current_segment_index = 1
    println(target_segment)
    path = routing(localization_state_channel, target_segment, map) #this will be the list of segments returned by routing function
    for i in 1:length(path)-1
        pt = compute_midpoints(path[i])
        push!(polyline, pt)
    end
    pt = compute_midpoint_target(path[end])
    push!(polyline, pt)

    #now we can do PID controller on polyline
    alpha = [0.0, 0.0]
    current_segment_index = 1

    # --- State tracking for stop sign ---
    at_stop_sign = false
    stop_timer_started = false
    stop_start_time = 0.0
    required_stop_time = 10.0  # seconds to wait at stop sign
    
    # --- Obstacle tracking ---
    obstacle_detected = false
    obstacle_distance = Inf
    
    # --- Obstacle avoidance parameters ---
    stop_distance = 20.0        # Stop completely at this distance 
    caution_distance = 80.0     # Start slowing down at this distance 
    detection_distance = 100.0  # Maximum detection range 
    
    while true
        t = -1.0
        if use_gt
            latest_localization_state = take!(localization_state_channel)
        else
            latest_localization_state = fetch(localization_state_channel)
        end

        # Get latest perception state with obstacles
        latest_perception_state = fetch(perception_state_channel)
        current_time = time()
        
        @info "[decision] Perception state has $(length(latest_perception_state.obstacles)) obstacles"
        if !isempty(latest_perception_state.obstacles)
            for (i, obs) in enumerate(latest_perception_state.obstacles)
                @info "[decision] Obstacle $i: pos=$(obs.position), vel=$(obs.velocity)"
            end
        end
        
        # Extract ego vehicle state
        c1 = latest_localization_state.position[1]
        c2 = latest_localization_state.position[2]
        ego_pos = SVector(c1, c2)
        θ = VehicleSim.extract_yaw_from_quaternion(latest_localization_state.orientation)
        ego_heading = SVector(cos(θ), sin(θ))
        v = norm(latest_localization_state.velocity)
        
        # Check for obstacles ahead with increased detection distance
        if !isempty(latest_perception_state.obstacles)
            obstacle_ahead, distance = is_obstacle_ahead(
                latest_perception_state.obstacles,
                ego_pos,
                ego_heading,
                detection_distance,  
                4.0                  
            )
            
            obstacle_detected = obstacle_ahead
            obstacle_distance = distance
        else
            obstacle_detected = false
            obstacle_distance = Inf
        end
        
        # Pure Pursuit
        ls = 0.1 #lookahead time
        L = 13

        lookahead_radius = 10.0
        increase_lookahead_step = 1.0
        furthest_view = 20.0
        current_segment = polyline[current_segment_index]

        p1 = current_segment[1]
        p2 = current_segment[2]
        a = (p2[1] - p1[1])^2 + (p2[2] - p1[2])^2
        b = 2 * ((p2[1] - p1[1]) * (p1[1] - c1) + (p2[2] - p1[2]) * (p1[2] - c2))
        q = nothing

        while true
            fetch(shutdown_channel) && return
            c = (p1[1] - c1)^2 + (p1[2] - c2)^2 - lookahead_radius^2 

            discriminant = b^2 - 4 * a * c
            center = SVector(c1, c2)

            if discriminant >= 0
                sqrt_disc = sqrt(discriminant)
                t_upper = (-b + sqrt_disc) / (2 * a)
                t_lower = (-b - sqrt_disc) / (2 * a)
                valid_t = filter(t -> -0.05 ≤ t , [t_upper, t_lower])
                t = isempty(valid_t) ? -5 : first(valid_t)
                q = SVector((t * (p2 - p1) + p1)) - center
            else 
                q = center
            end

            if lookahead_radius >= furthest_view
                lookahead_radius = 10.0
            elseif t == -1 || t == -5
                lookahead_radius += increase_lookahead_step
            else
                break
            end
        end

        heading = [cos(θ); sin(θ)]
        dot_value = dot(q, heading) / (norm(q) * norm(heading))
        alpha = acos(clamp(dot_value, -1, 1))  # Angle magnitude

        # Use cross product to determine sign
        cross_value = heading[1] * q[2] - heading[2] * q[1]  # 2D cross product determinant
        alpha *= sign(cross_value)

        # Calculate base steering angle using Pure Pursuit
        base_steering = atan((2 * L * sin(alpha)) / lookahead_radius)
        
        # Set default command values
        steering_angle = base_steering
        target_vel = 5.0
        
        # Handle obstacles 
        if obstacle_detected
            
            if obstacle_distance < stop_distance
                # Full stop 
                target_vel = 0.0
                cmd = (steering_angle, target_vel, true)
                serialize(socket, cmd)
                sleep(2.0)
                @info "[obstacle_avoidance] STOP - obstacle at distance: $obstacle_distance"
            elseif obstacle_distance < caution_distance
                # Gradual deceleration 
                decel_factor = (obstacle_distance - stop_distance) / (caution_distance - stop_distance)
                target_vel = 4.0 * decel_factor
                cmd = (steering_angle, target_vel, true)
                serialize(socket, cmd)
                @info "[obstacle_avoidance] SLOW - obstacle at distance: $obstacle_distance, speed: $target_vel"
            else
                # Detected but still far - slightly reduced speed
                target_vel = 4.5
                cmd = (steering_angle, target_vel, true)
                serialize(socket, cmd)
                @info "[obstacle_avoidance] CAUTION - obstacle at distance: $obstacle_distance"
            end
        else
            cmd = (steering_angle, target_vel, true)
            @info "[obstacle_avoidance] No obstacles detected"
        end
        
        # Set command based on obstacle detection
        #cmd = (steering_angle, target_vel, true)
        
        # Handle stop signs (existing logic)
        lanes = path[current_segment_index].lane_types
    
        if VehicleSim.stop_sign in lanes
            if !at_stop_sign && t >= 0.85
                target_speed = 0.0
                println("Stop sign detected")
                cmd = (steering_angle, target_speed, true)
                at_stop_sign = true
                serialize(socket, cmd)
                sleep(2.0)
            else
                # Keep obstacle detection adjustments unless we're at a stop sign
                cmd = (steering_angle, target_vel, true)
            end
        end
    
        # Segment transition logic
        if 0.9 ≤ t && current_segment_index < length(path)
            current_segment_index += 1
            at_stop_sign = false
            println("Moving to segment ", current_segment_index)
        elseif 0.2 ≤ t && current_segment_index >= length(path)
            @info "arrived at target"
            println("Arrived at destination")
            cmd = (steering_angle, 0.0, true)
            serialize(socket, cmd)
            sleep(3.0)
            fetch(shutdown_channel) && return
            target_segment = fetch(target_segment_channel)
            @info "Getting new route"
            println(target_segment)
            current_seg = path[current_segment_index].id
            path = nothing
            path = routing(localization_state_channel, target_segment, map, current_seg)
            println(path)
            polyline = []
            pt = nothing
            for i in 1:length(path)-1
                pt = compute_midpoints(path[i])
                push!(polyline, pt)
            end
            pt = compute_midpoint_target(path[end])
            push!(polyline, pt)
            alpha = [0.0, 0.0]
            current_segment_index = 1
            println(polyline)
        end
        
        # Send command to vehicle
        serialize(socket, cmd)
        
        # Small sleep to avoid busy-waiting
        sleep(0.01)
    end
catch e
    println("ERROR: $e")
    return nothing
end
    
end

function test_target_change(target_segment_channel, shutdown_channel)
    while true
        fetch(shutdown_channel) && return
        @info "Spinning"
        sleep(3.0)
        take!(target_segment_channel)
        put!(target_segment_channel, 40)
    end
end

function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end


function my_client(host::IPAddr=IPv4(0); use_gt=false, port=4444)
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
    perception_state_channel = Channel{MyPerceptionType}(1)
    shutdown_channel = Channel{Bool}(1)

    # Define the gt_eval_channel if it doesn't exist
    gt_eval_channel = Channel{Vector{ObstacleDetection}}(1)

    # Initialize shutdown with false
    put!(shutdown_channel, false)
    # Initialize the perception state channel with default empty perception state
    initial_perception = MyPerceptionType(
        time(),
        Vector{ObstacleDetection}(),
        Vector{LaneMarking}()
    )
    put!(perception_state_channel, initial_perception)
    
    # Initialize the localization state channel with a default value
    initial_localization = MyLocalizationType(
        SVector{3, Float64}(0.0, 0.0, 0.0),
        SVector{4, Float64}(1.0, 0.0, 0.0, 0.0), # Identity quaternion
        SVector{3, Float64}(0.0, 0.0, 0.0)
    )
    put!(localization_state_channel, initial_localization)

    # Initialize gt_eval_channel with empty vector
    println(Vector{ObstacleDetection}())
    put!(gt_eval_channel, Vector{ObstacleDetection}())

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    put!(target_segment_channel, target_map_segment)

    # Define a shared function to convert ground truth to obstacles that can be used by both
    # process_gt and the testing loop
    function shared_convert_gt_to_obstacles(gt_measurements, ego_vehicle_id)
        obstacles = ObstacleDetection[]
        for gt in gt_measurements
            # Skip the ego vehicle - important!
            if gt.vehicle_id == ego_vehicle_id
                continue
            end
            
            # Extract position with validation
            position = if all(isfinite.(gt.position))
                gt.position
            else
                SVector{3, Float64}(0.0, 0.0, 0.0)
            end
            
            # Extract size with validation
            size = if isdefined(gt, :size) && all(isfinite.(gt.size))
                gt.size
            else
                SVector{3, Float64}(4.0, 2.0, 1.5)  # Default car size
            end
            
            # Extract velocity with validation
            velocity = if isdefined(gt, :velocity) && all(isfinite.(gt.velocity[1:2]))
                gt.velocity[1:2]
            else
                SVector{2, Float64}(0.0, 0.0)
            end
            
            # Create the obstacle detection with high confidence since it's ground truth
            obstacle = ObstacleDetection(
                position,
                size,
                velocity,
                1.0,  # Confidence = 1.0 for ground truth
                gt.vehicle_id  # Use vehicle ID as tracking ID
            )
            
            push!(obstacles, obstacle)
        end
        
        return obstacles
    end

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
        old_target_segment = fetch(target_segment_channel)
        if target_map_segment != old_target_segment
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

    tasks = []
    push!(tasks, error_mon)
    if use_gt
        @info "Using gt measurements"
        gt_task = @async begin
            try
                while true
                    if fetch(shutdown_channel)
                        break
                    end
    
                    fresh_gt_meas = []
                    
                    while isready(gt_channel)
                        meas = take!(gt_channel)
                        push!(fresh_gt_meas, meas)
                    end
    
                    if !isempty(fresh_gt_meas)
                        try
                            # Convert GT to obstacles, making sure to filter out ego vehicle
                            gt_detections = shared_convert_gt_to_obstacles(fresh_gt_meas, ego_vehicle_id)
                            
                            @info "[gt_processing] Created $(length(gt_detections)) obstacle detections from ground truth"
                            
                            # Send to evaluation channel
                            if isready(gt_eval_channel)
                                take!(gt_eval_channel)
                            end
                            put!(gt_eval_channel, gt_detections)

                            # Create localization state from ego vehicle GT
                        ego_gt = nothing
                        for gt in fresh_gt_meas
                            if gt.vehicle_id == ego_vehicle_id
                                ego_gt = gt
                                break
                            end
                        end
                        
                        if ego_gt !== nothing
                            new_localization_state = MyLocalizationType(
                                ego_gt.position,
                                ego_gt.orientation,
                                ego_gt.velocity
                            )
                            
                            if isready(localization_state_channel)
                                take!(localization_state_channel)
                            end
                            put!(localization_state_channel, new_localization_state)
                        end
                        
                        # Create perception state with obstacles
                        new_perception_state = MyPerceptionType(
                            time(),
                            gt_detections,  # Use the filtered detections
                            Vector{LaneMarking}()
                        )

                        # Make sure perception state is updated
                        if isready(perception_state_channel)
                            take!(perception_state_channel)
                        end
                        put!(perception_state_channel, new_perception_state)
                        
                        @info "[gt_processing] Updated perception with $(length(gt_detections)) obstacles"
                    catch e
                        @error "[gt_processing] Error processing GT: $e"
                    end
                end
                
                sleep(0.01)
            end
        catch e
            @error "[gt_task] Error: $e"
        end
    end
    push!(tasks, gt_task)

    else
        @info "Using sensor measurements"
        loc_task = @async begin
            try
                localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel)
            catch e
                @error "Localization task error: $e"
                for (i, frame) in enumerate(stacktrace(catch_backtrace()))
                    println(stderr, "[$i] $(Base.show_backtrace_entry(frame))")
                end
            end
        end
        push!(tasks, loc_task)

        sleep(2.0)

        perc_task = @async begin
            try
                perception(cam_channel, localization_state_channel, perception_state_channel, shutdown_channel)
            catch e
                @error "Perception task error: $e"
                for (i, frame) in enumerate(stacktrace(catch_backtrace()))
                    println(stderr, "[$i] $(Base.show_backtrace_entry(frame))")
                end
            end
        end  
        push!(tasks, perc_task)

    end

    # Run the decision making task
    dec_task = @async begin
        try
            pos_state_channel = nothing
            if use_gt
                pos_state_channel = gt_channel
            else
                pos_state_channel = localization_state_channel
            end
            decision_making(use_gt, pos_state_channel, 
                           perception_state_channel, 
                           target_segment_channel, 
                           shutdown_channel,
                           map_segments, 
                           socket, gt_channel)
        catch e
            @error "Decision making task error: $e"
            for (i, frame) in enumerate(stacktrace(catch_backtrace()))
                println(stderr, "[$i] $(Base.show_backtrace_entry(frame))")
            end
        end
    end
    push!(tasks, dec_task)
    @info "here5"

    #push!(tasks, @async perception(cam_channel, localization_state_channel, perception_state_channel))
   # Start the shutdown listener
   shutdown_task = @async begin
    try
        shutdown_listener(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel)
    catch e
        @error "Shutdown listener error: $e"
    end
end
push!(tasks, shutdown_task)
    # push!(tasks, @async test_target_change(target_segment_channel, shutdown_channel))

    for t in tasks
        wait(t)
    end
end

function shutdown_listener(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel)
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