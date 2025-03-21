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

## Ellie Chason - start - ##

function compute_bbox(seg::RoadSegment)
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

function find_current_segment(localization_state::MyLocalizationType, all_segs::Dict{Int, RoadSegment})
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

# -- Sung-Lin --
"""
Process raw camera measurements to extract bounding boxes.
"""
function process_bounding_boxes(cam_meas_channel)
    # TODO: Extract bounding boxes from camera measurements
    processed_detections = []
    return processed_detections
end

"""
Predict the future state of an object based on the motion model.
Uses the state transition function f(x) from EKF implementation.
"""
function predict_object_state(obj_state, Δt)
    #TODO: implement state transition function
    # [p1, p2, θ, v, l, w, h] -> [p1 + Δt*v*cos(θ), p2 + Δt*v*sin(θ), θ, v, l, w, h]
    # Update covariance using Jacobian
    return obj_state
end

"""
Project 3D object state to 2D bounding box in image coordinates.
This is the measurement function h(x) from the EKF implementation.
"""
function project_state_to_bbox(obj_state, localization_state, camera_params)
    # TODO: Implement the measurement function that projects 3D state to 2D bbox
    # 1. Calculate 3D bounding box corners
    # 2. Transform to camera coordinates
    # 3. Project to image plane
    # 4. Calculate bounding box extremes
    return [0.0, 0.0, 0.0, 0.0]
end

"""
Update track state using Kalman filter update step with new detection.
"""
function update_track_with_detection(track, detection)
    # TODO: Implement Kalman filter update step
    # 1. Calculate innovation (measurement residual)
    # 2. Calculate Kalman gain
    # 3. Update state and covariance
    return track 
end

"""
Initialize a new track from a detection.
"""
function initialize_new_track(detection, next_id)
    # TODO: Implement new track initialization
    # 1. Estimate initial state from detection
    # 2. Set initial covariance (high uncertainty)
    # 3. Return new ObjectState
    return nothing
end

# Emily
function collision_constraint(Body1, Body2)
    # based on shape of vehicle (which I don't fully understand at this point) ensure that bodies do not
    # overlap
end

# Emily
function straight_lane_constraint(X, outer_bound, inner_bound)
    # represent lane boundaries as halfspaces and require vehicle to remain within them
    # we will need different lane constraint functions depending on shape of lane boundaries (curved vs. straight)
end

# Gloria
# function generateRoute(current_segment_id, target_road_segment_id, map)
#     curr_segment = map[current_segment]
#     target_segment = map[target_road_segment_id]
#     to_visit = Queue(RoadSegment)
#     path = []

#     # TODO: implement simple BFS followed by A* search
#     # while curr_segment != target_segment
#     #     # dequeue the current path (array of integer segment ids)
#     #     # enqueue all children of the current segment into to_visit
#     #     # 
#     # end

#     return path
# end

function decision_making(localization_state_channel, 
        perception_state_channel, 
        map, 
        target_road_segment_id, 
        socket)
    # do some setup
    while true
        latest_localization_state = fetch(localization_state_channel)
        latest_perception_state = fetch(perception_state_channel)

        # Routing
        # Verification check that current_segment_id = where our localization says we are
        # generateRoute(current_segment_id, target_road_segment_id, map)

        # Trajectory Generation
        # Use ipopt/ HW2 structure to express geometric costs and constraints


        # PID Control
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