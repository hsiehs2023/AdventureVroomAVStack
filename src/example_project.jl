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

function associate_tracks(detections::Vector{ObstacleDetection}, tracks::Vector{TrackedObstacle}; threshold=5.0)
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
    #@info "IN localization"
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
    
        #@info "[localize] Waiting for gps_channel..."
        while !isready(gps_channel)
            sleep(0.001)
            fetch(shutdown_channel) && break
        end
        #@info "[localize] Got GPS measurement"
        
        fresh_gps_meas = []
        while isready(gps_channel)
            fetch(shutdown_channel) && break
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end
    
        #@info "[localize] Waiting for imu_channel..."
        while !isready(imu_channel)
            sleep(0.001)
            fetch(shutdown_channel) && break
        end
        #@info "[localize] Got IMU measurement"
    
        fresh_imu_meas = []
        while isready(imu_channel)
            fetch(shutdown_channel) && break
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end
        
        #@info "[localize] Try to get localization"
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
       # @info "[localize] Got localization"

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

        #NEVER GET TO TRY BLOCK

        #TODO Get a better estimate of these values. Adjust position to be from initial GPS measurement
        #TODO add in these measurements into μs (velocities can remain 0)
        try
            #@info "[localize] Starting state estimation..."
            # everything between IMU and put!(localization_state_channel, ...)
            alpha = fresh_gps_meas[end].heading
            μs = [[fresh_gps_meas[end].long, fresh_gps_meas[end].lat, 2.65, cos(alpha/2), 0, 0, sin(alpha/2),
                    fresh_imu_meas[end].linear_vel[1], fresh_imu_meas[end].linear_vel[2], fresh_imu_meas[end].linear_vel[3],
                    fresh_imu_meas[end].angular_vel[1], fresh_imu_meas[end].angular_vel[2], fresh_imu_meas[end].angular_vel[3]]]
        
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

                
                #@info "   Ground truth (x,y): $(μs[2][1:3])"
                #@info "   estimated: $(μ[1:3])"

            end

            localization_state = MyLocalizationType(μ[1:3], μ[4:7])
            #@info "[localize] About to write localization_state to channel"
            if isready(localization_state_channel)
                take!(localization_state_channel)
            end
            put!(localization_state_channel, localization_state)
            #@info "[localize] Wrote localization_state!"
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
        depth = max(focal_len * 1.5 / (box_height * pixel_len), 1)
        #@info "[pixel_to_world] Using depth: $depth"
        
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

function perception(cam_meas_channel, localization_state_channel, perception_state_channel, shutdown_channel)
    @info "[perception] Running perception loop"
    tracks = TrackedObstacle[]
    next_track_id = 1
    
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
                #@info "[perception] Fetching localization state..."
                latest_localization_state = fetch(localization_state_channel)
                #@info "[perception] Got localization: $(latest_localization_state)"
            
                for cam_meas in fresh_cam_meas
                    @info "[perception] Processing camera measurement with $(length(cam_meas.bounding_boxes)) boxes"
                    for box in cam_meas.bounding_boxes
                        @info "[perception] Processing box: $box"
                        try
                            pos, size = pixel_to_world(latest_localization_state, cam_meas, box)
                            @info "[perception] Computed world position: $pos, size: $size"
                            push!(detections, ObstacleDetection(pos, size, SVector(0.0, 0.0), 0.8, 0))
                        catch e
                            @error "[perception] Failed to process box: $e"
                            Base.show_backtrace(stderr, catch_backtrace())
                        end
                    end
                end
                
                @info "[perception] Created $(length(detections)) initial obstacle detections"
            catch e
                @error "[perception] Failed to process camera measurements: $e"
                Base.show_backtrace(stderr, catch_backtrace())
                continue  # Skip this iteration if we can't process the measurements
            end

            detections = cluster_detections(detections)
            @info "[perception] After clustering: $(length(detections)) detections"

            # Apply EKF prediction to existing tracks
            for track in tracks
                dt = current_time - track.last_seen
                ekf_predict!(track, dt)
            end

            # Associate detections with existing tracks
            # This function returns a vector where assignment[i] is the track index for detection i
            assignment = associate_tracks(detections, tracks)
            assignment = [j > length(tracks) ? 0 : j for j in assignment]
            
            # Debug the assignment to understand its structure
            @info "[perception] Assignment result: $assignment"
            
            # Count non-zero assignments to report how many are matched with existing tracks
            num_matched = count(x -> x != 0, assignment)
            @info "[perception] Associated $num_matched detections with existing tracks"

            assigned_tracks = Set{Int}()
            
            # Process each detection - either update existing track or create new one
            for i in 1:length(detections)
                # Get the assigned track index for this detection
                if i <= length(assignment)
                    track_idx = assignment[i]
                else
                    @warn "[perception] No assignment for detection $i"
                    track_idx = 0  # Default to creating a new track
                end
                
                # Check if the track index is valid and the track exists
                if track_idx != 0 && track_idx <= length(tracks)
                    # Update existing track with this detection
                    ekf_update!(tracks[track_idx], detections[i].position[1:2])
                    tracks[track_idx].last_seen = current_time
                    tracks[track_idx].confidence = max(tracks[track_idx].confidence, detections[i].confidence)
                    push!(assigned_tracks, track_idx)
                    
                    # Update detection with track info
                    detections[i] = ObstacleDetection(
                        detections[i].position, 
                        detections[i].size, 
                        tracks[track_idx].x[3:4],  # Use velocity from track
                        tracks[track_idx].confidence, 
                        tracks[track_idx].id)
                else
                    # Create new track for this detection
                    pos = detections[i].position[1:2]
                    # EKF mean: use detection position, assume zero velocity
                    x₀ = SVector(pos[1], pos[2], 0.0, 0.0)
 
                    # EKF covariance: moderate confidence in position, high uncertainty in velocity
                    P₀ = Diagonal([0.5^2, 0.5^2, 5.0^2, 5.0^2])  # variances
                    new_track = TrackedObstacle(
                        next_track_id,
                        x₀,
                        P₀,
                        current_time,
                        0.8
                    )
                    # Add logging for visibility
                    @info "[init] New EKF track $next_track_id"
                    @info "   μ₀ = $x₀"
                    @info "   Σ₀ = \n$P₀"

                    # Update detection with new track ID
                    detections[i] = ObstacleDetection(
                        detections[i].position, 
                        detections[i].size, 
                        SVector(0.0, 0.0),  # New track has no velocity estimate yet
                        detections[i].confidence, 
                        next_track_id)
                    
                    push!(tracks, new_track)
                    next_track_id += 1
                end
            end

            # Remove old tracks that haven't been seen recently
            tracks = [t for t in tracks if (current_time - t.last_seen < 2.0) || (t.confidence > 0.3)]
            @info "[perception] After cleanup: $(length(tracks)) active tracks"

            # Create perception state with current detections
            perception_state = MyPerceptionType(
                current_time,
                detections,  # These now have track IDs and velocities where available
                Vector{LaneMarking}()  # No lane markings for now
            )

            @info "[perception] Created perception state with $(length(detections)) detections"

            # Update perception channel with new state
            if isready(perception_state_channel)
                take!(perception_state_channel)  # Remove old state
            end
            put!(perception_state_channel, perception_state)  # Add new state

            #@info "[perception] Updated perception state channel"
            
            sleep(0.01)  # Short sleep to avoid busy-waiting
        end
    catch e
        @error "[perception] CRASHED with error: $e"
        Base.show_backtrace(stderr, catch_backtrace())
    end
    @info "[perception] Wrote perception with $(length(detections)) obstacles at time $(current_time)"
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

        # figure out what to do .. setup motion planning problem etc
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

    test_point = SVector(3.0, 4.0, 10.0)
     f = 800.0
 
     f_proj = p -> perspective_projection(p, f)
     J_analytic = jacobian_projection_analytic(test_point, f)
     J_numeric = numeric_jacobian(f_proj, test_point)
 
     diff = J_analytic - J_numeric
     max_diff = maximum(abs.(diff))
 
     @info "[jacobian-test] Analytic Jacobian:\n$J_analytic"
     @info "[jacobian-test] Numeric Jacobian:\n$J_numeric"
     @info "[jacobian-test] Difference:\n$diff"
     if max_diff < 1e-5
         @info "[jacobian-test] PASSED: max error $max_diff"
     else
         @error "[jacobian-test] FAILED: max error $max_diff exceeds tolerance"
     end

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
        SVector{4, Float64}(1.0, 0.0, 0.0, 0.0) # Identity quaternion
    )
    put!(localization_state_channel, initial_localization)
    
    # Initialize gt_eval_channel with empty vector
    put!(gt_eval_channel, Vector{ObstacleDetection}())

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    put!(target_segment_channel, target_map_segment)

    # Define a shared function to convert ground truth to obstacles that can be used by both
    # process_gt and the testing loop
    function shared_convert_gt_to_obstacles(gt_measurements)
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
                @info "Received CameraMeasurement with $(length(meas.bounding_boxes)) boxes"
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
    end)

    tasks = []
    
    if use_gt
    
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
                        # Transform GT messages into obstacle detections
                        try
                            
                            gt_detections = shared_convert_gt_to_obstacles(fresh_gt_meas)
                         
                            # Send ground-truth detections into the eval channel for evaluation
                            if isready(gt_eval_channel)
                                take!(gt_eval_channel)
                            end
                            put!(gt_eval_channel, gt_detections)
                            
                            # Create a new localization state from ground truth
                            # This assumes the first GT measurement is for the ego vehicle
                            if !isempty(fresh_gt_meas)
                                ego_gt = fresh_gt_meas[1]  # Just use the first one for simplicity
                                
                                # Extract position and orientation, handling potential missing fields
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
                            # Exception handling without logging
                        end
                    end
                    
                    sleep(0.01)  # Small sleep to avoid busy-waiting
                end
            catch e
                # Exception handling without logging
            end
        end
        push!(tasks, gt_task)
    else
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
            decision_making(localization_state_channel, 
                           perception_state_channel, 
                           target_segment_channel, 
                           shutdown_channel,
                           map_segments, 
                           socket)
        catch e
            @error "Decision making task error: $e"
            for (i, frame) in enumerate(stacktrace(catch_backtrace()))
                println(stderr, "[$i] $(Base.show_backtrace_entry(frame))")
            end
        end
    end
    push!(tasks, dec_task)
    
    # Perception testing task
    test_task = @async begin
        last_timestamp = 0.0
    
        while true
            sleep(1.0)
    
            fetch(shutdown_channel) && break
    
            # --- Update GT Eval Channel ---
            if isready(gt_channel)
                gt_meas = GroundTruthMeasurement[]
                while isready(gt_channel)
                    push!(gt_meas, take!(gt_channel))
                end
    
                try
                    gt_detections = shared_convert_gt_to_obstacles(gt_meas)
    
                    if isready(gt_eval_channel)
                        take!(gt_eval_channel)
                    end
                    put!(gt_eval_channel, gt_detections)
                catch e
                    @error "Error converting ground truth measurements in test loop: $e"
                end
            end
    
            # --- Perception Evaluation ---
            try
                new_perception = nothing
    
                # Wait for a perception state with a newer timestamp
                while true
                    if isready(perception_state_channel)
                        maybe_new = fetch(perception_state_channel)
                        if maybe_new.timestamp > last_timestamp
                            new_perception = maybe_new
                            last_timestamp = new_perception.timestamp
                            break
                        end
                    end
                    sleep(0.01)
                    fetch(shutdown_channel) && break
                end
    
                if new_perception !== nothing
                    @info "[test] Read NEW perception state with timestamp $(new_perception.timestamp) and $(length(new_perception.obstacles)) obstacles"
    
                    # --- Compare to ground truth ---
                    if isready(gt_eval_channel)
                        try
                            gt = fetch(gt_eval_channel)
                            est = new_perception.obstacles
    
                            @info "Comparing perception to ground truth:"
                            @info "   # Perceived: $(length(est)), # GT: $(length(unique(obstacle.id for obstacle in gt)))"
    
                            if !isempty(est) && !isempty(gt)
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
                                end
                            end
                        catch e
                            @error "Error comparing to ground truth: $e"
                        end
                    end
                end
            catch e
                @error "Error in perception test loop: $e"
            end
        end
    end
    push!(tasks, test_task)

    # Start the shutdown listener
    shutdown_task = @async begin
        try
            shutdown_listener(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel)
        catch e
            @error "Shutdown listener error: $e"
        end
    end
    push!(tasks, shutdown_task)
    
    # Wait for all tasks to complete
    for task in tasks
        wait(task)
    end
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
    push!(tasks, @async localize(gps_channel, imu_channel, localization_state_channel, shutdown_channel, gt_channel))
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