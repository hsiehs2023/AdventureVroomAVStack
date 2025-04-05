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

struct MyPerceptionType
    timestamp::Float64                    # timestamp of this perception state
    obstacles::Vector{ObstacleDetection}  # detected obstacles
    lane_markings::Vector{LaneMarking}    # detected lane markings
end

struct MyLocalizationType
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

function localize(
        gps_channel, 
        imu_channel, 
        localization_state_channel, 
        shutdown_channel)
    # Set up algorithm / initialize variables
    while true

        fetch(shutdown_channel) && break

        fresh_gps_meas = []
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end
        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end
        
        # process measurements
        take!(localization_state_channel)
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

function perception(cam_meas_channel, localization_state_channel, perception_state_channel, shutdown_channel)
    # Obstacle tracking data structure
    tracked_obstacles = Dict{Int, ObstacleDetection}()
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
        
        # Process camera measurements
        detected_obstacles = Vector{ObstacleDetection}()
        
        for cam_meas in fresh_cam_meas
            # Process each bounding box
            for box in cam_meas.bounding_boxes
                # Convert pixel coordinates to world coordinates
                position, size = pixel_to_world(latest_localization_state, cam_meas, box)
                
                # tracking: assign new ID for now (better tracking would match with previous detections)
                obstacle = ObstacleDetection(
                    position,
                    size,
                    SVector{2, Float64}(0.0, 0.0),  # Zero velocity for now (would estimate from tracking)
                    0.8,  # 80% confidence
                    next_track_id
                )
                
                next_track_id += 1
                push!(detected_obstacles, obstacle)
            end
        end
        
        # Create perception state
        perception_state = MyPerceptionType(
            fresh_cam_meas[end].time,  # Use latest measurement time
            detected_obstacles,
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

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)
    target_segment_channel = Channel{Int}(1)
    shutdown_channel = Channel{Bool}(1)
    put!(shutdown_channel, false)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    put!(target_segment_channel, target_map_segment)

    errormonitor(@async while true
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
        @async localize(gps_channel, 
                    imu_channel, 
                    localization_state_channel, 
                    shutdown_channel)

        @async perception(cam_channel, 
                      localization_state_channel, 
                      perception_state_channel, 
                      shutdown_channel)
    end



    @async decision_making(localization_state_channel, 
                           perception_state_channel, 
                           target_segment_channel, 
                           shutdown_channel,
                           map, 
                           socket)
end

function shutdown_listener(shutdown_channel)
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
end