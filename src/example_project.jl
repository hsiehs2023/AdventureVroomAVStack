struct MyLocalizationType
    field1::Int
    field2::Float64
end

struct MyPerceptionType
    field1::Int
    field2::Float64
end

function localize(gps_channel, imu_channel, localization_state_channel)
    # Set up algorithm / initialize variables
    while true
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

        localization_state = MyLocalizationType(0,0.0)
        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, localization_state)
    end 
end

function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    # set up stuff
    while true
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
    map, 
    target_road_segment_id, 
    socket)
# do some setup
while true
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


function my_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    #localization_state_channel = Channel{MyLocalizationType}(1)
    #perception_state_channel = Channel{MyPerceptionType}(1)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

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

    @async localize(gps_channel, imu_channel, localization_state_channel)
    @async perception(cam_channel, localization_state_channel, perception_state_channel)
    @async decision_making(localization_state_channel, perception_state_channel, map, socket)
end