using Test
using EmergencyBrakingSystem
using AutomotiveDrivingModels
using AutoViz
using AutomotiveSensors
using AutomotivePOMDPs
using Parameters
using StaticArrays
using Reel

@testset "EmergencyBrakingSystem.jl" begin

ped_v = 5.0 / 3.6
ped_theta = π/2
ped_right_side = true
hitpoint = 0
ped_t_collision = 1.0
ego_y = 0.0
ped_x = 100.0
ped_y = 0.0


obstacle_right_offset = -VehicleDef().width/2 - 1.0
obstacle_1 = ConvexPolygon([VecE2(44, obstacle_right_offset), VecE2(44, obstacle_right_offset-2), VecE2(48.5, obstacle_right_offset-2), VecE2(48.5, obstacle_right_offset)],4)
obstacles = [obstacle_1]


# unavoidable situation
ego_v = 50.0 / 3.6
ego_x = ped_x - ego_v * ped_t_collision - VehicleDef().length/2;
(scenes, timestep, env, ego_vehicle, sensor, sensor_observations, risk, ttc, collision_rate, brake_request, prediction, collision) = EmergencyBrakingSystem.evaluate_scenario(ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles);
@test collision[end] == true

# avoidable situation
ego_v = 30.0 / 3.6
ego_x = ped_x - ego_v * ped_t_collision - VehicleDef().length/2;
(scenes, timestep, env, ego_vehicle, sensor, sensor_observations, risk, ttc, collision_rate, brake_request, prediction, collision) = EmergencyBrakingSystem.evaluate_scenario(ego_x, ego_y, ego_v, ped_x, ped_y, ped_v, ped_theta, obstacles);
@test collision[end] == false

end








