using StaticArrays
using LinearAlgebra

function mocked_localization_state()
    AdventureVroomAVStack.MyLocalizationType(
        SVector(10.0, 0.0, 1.5),
        SVector(1.0, 0.0, 0.0, 0.0)
    )
end

function mocked_camera_measurement()
    time = 123.456
    camera_id = 1
    focal_length = 800.0
    pixel_length = 0.005
    image_width = 640
    image_height = 480
    bounding_boxes = [SVector{4, Int64}(100, 120, 200, 220)]

    return AdventureVroomAVStack.CameraMeasurement(
        time,
        camera_id,
        focal_length,
        pixel_length,
        image_width,
        image_height,
        bounding_boxes
    )
end

@testset "Perception Module" begin
    # Prepare channels
    cam_channel = Channel{AdventureVroomAVStack.CameraMeasurement}(1)
    loc_channel = Channel{AdventureVroomAVStack.MyLocalizationType}(1)
    perc_channel = Channel{AdventureVroomAVStack.MyPerceptionType}(1)
    shutdown_channel = Channel{Bool}(1)
    
    # Put test data
    put!(cam_channel, mocked_camera_measurement())
    put!(loc_channel, mocked_localization_state())
    put!(shutdown_channel, false)
    
    # Start perception processing
    t = @async AdventureVroomAVStack.perception(cam_channel, loc_channel, perc_channel, shutdown_channel)
    
    sleep(0.2)  # wait for async task to process
    
    # Shut down perception task to avoid infinite loop
    take!(shutdown_channel)
    put!(shutdown_channel, true)

    wait(t)  # Wait for perception to exit

    perception_result = take!(perc_channel)
    
    @test length(perception_result.obstacles) == 1
    @test perception_result.obstacles[1].confidence ≈ 0.8
    @test norm(perception_result.obstacles[1].position) > 0  # sanity check
    
    # Clean shutdown
    take!(shutdown_channel)
    put!(shutdown_channel, true)
    wait(t)
end