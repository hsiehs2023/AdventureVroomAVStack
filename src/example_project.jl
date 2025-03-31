struct MyLocalizationType
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
    
    # Extract GPS position from localization state
    # In a real implementation, we would need to buffer and process the GPS measurements
    if latest_localization_state != nothing
        if typeof(latest_localization_state) <: MyLocalizationType
            # If we're using our custom localization type
            # This can contain processed GPS data already
            current_segment_id = latest_localization_state.segment_id
            last_known_position = latest_localization_state.position
            vehicle_heading = latest_localization_state.heading
        else
            # For simplicity, assume the first field is a position vector
            last_known_position = SVector(latest_localization_state.field1, latest_localization_state.field2)
            vehicle_heading = 0.0  # Heading would come from IMU or processed GPS
            
            # Find current road segment based on GPS position
            estimated_segment_id = find_nearest_segment(map, last_known_position)
            current_segment_id = estimated_segment_id
        end
    end
    
    # If segment changed or no route, recalculate route
    if current_segment_id > 0 && (isempty(current_route) || (current_segment_id != current_route[route_index] && !(route_index < length(current_route) && current_segment_id == current_route[route_index+1])))
        current_route = plan_route(map, current_segment_id, target_segment_id)
        route_index = 1
        @info "New route planned: $current_route"
    end
    
    # Simple logic for steering and velocity
    steering_angle = 0.0
    target_vel = default_speed
    
    if !isempty(current_route) && route_index < length(current_route)
        next_segment_id = current_route[route_index + 1]
        
        # Check if we've reached the next segment
        if current_segment_id == next_segment_id
            route_index += 1
            if route_index < length(current_route)
                next_segment_id = current_route[route_index + 1]
            end
        end
        
        # Get current and next segment to determine direction
        if haskey(map, current_segment_id) && haskey(map, next_segment_id)
            current_seg = map[current_segment_id]
            next_seg = map[next_segment_id]
            
            # Calculate desired heading to the next segment
            desired_heading = get_direction_to_next_segment(current_segment_id, next_segment_id)
            
            # Calculate steering based on the difference between current and desired heading
            # This is a simple proportional controller
            heading_error = desired_heading - vehicle_heading
            # Normalize angle to [-π, π]
            while heading_error > π
                heading_error -= 2π
            end
            while heading_error < -π
                heading_error += 2π
            end
            
            # Apply proportional control with a gain
            steering_angle = 0.5 * heading_error
            
            # Limit steering angle
            steering_angle = max(-0.5, min(0.5, steering_angle))
            
            # Adjust speed based on segment type
            if contains_lane_type(next_seg, intersection)
                # Approaching intersection - slow down
                target_vel = slow_speed
            elseif contains_lane_type(next_seg, stop_sign)
                # Approaching stop sign - slow down
                target_vel = slow_speed
            elseif next_segment_id == target_segment_id
                # Approaching final destination
                target_vel = slow_speed
            end
        end
        
        # Basic obstacle avoidance using perception
        if latest_perception_state != nothing
            if typeof(latest_perception_state) <: MyPerceptionType
                # Using our custom perception type
                if latest_perception_state.is_path_blocked
                    target_vel = 0.0  # Stop if path is blocked
                elseif latest_perception_state.min_distance < stop_distance
                    # Slow down proportionally to obstacle distance
                    target_vel = max(0.0, target_vel * (latest_perception_state.min_distance / stop_distance))
                end
            else
                # Simplified - assumes field2 might contain distance to nearest obstacle
                obstacle_distance = latest_perception_state.field2
                if obstacle_distance < stop_distance
                    target_vel = max(0, target_vel * (obstacle_distance / stop_distance))
                end
            end
        end
    else
        # We've reached the end of the route or no route found
        if current_segment_id == target_segment_id
            # We've reached the destination - stop
            target_vel = 0.0
            @info "Reached target destination!"
        else
            # No valid route found - slow down
            target_vel = slow_speed
            @info "No valid route found from segment $current_segment_id to $target_segment_id"
        end
    end
    
    cmd = (steering_angle, target_vel, true)
    serialize(socket, cmd)
    
    # Sleep a bit to prevent tight loop
    sleep(0.01)
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