@testset "router1" begin
    start_segment = 32
    end_segment =  53
    known_route = [32, 30, 28, 26, 24, 16, 49, 51, 53]

    map = AdventureVroomAVStack.VehicleSim.city_map()

    my_route = AdventureVroomAVStack.plan_route(map, start_segment, end_segment)
    
    my_route_ids = []
    for segment in my_route
        push!(my_route_ids, segment.id)
    end
    @test my_route_ids == known_route
end

@testset "router2" begin
    start_segment = 93
    end_segment =  59
    known_route = [93, 95, 97, 99, 101, 5, 13, 20, 49, 51, 53, 55, 57, 59]

    map = AdventureVroomAVStack.VehicleSim.city_map()

    my_route = AdventureVroomAVStack.plan_route(map, start_segment, end_segment)
    
    my_route_ids = []
    for segment in my_route
        push!(my_route_ids, segment.id)
    end
    @test my_route_ids == known_route
end

@testset "router3" begin
    start_segment = 27
    end_segment =  84
    known_route = [27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 2, 84]

    map = AdventureVroomAVStack.VehicleSim.city_map()

    my_route = AdventureVroomAVStack.plan_route(map, start_segment, end_segment)
    
    my_route_ids = []
    for segment in my_route
        push!(my_route_ids, segment.id)
    end
    @test my_route_ids == known_route
end