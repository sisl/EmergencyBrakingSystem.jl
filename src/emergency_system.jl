

@with_kw mutable struct EmergencySystem <: DriverModel{LatLonAccel}
    
    timestep::Float64 = 0.05
    t_current::Float64 = 0
    tick::Int64 = 0


    ego_vehicle::Vehicle = Vehicle(VehicleState(VecSE2(0., 0., 0.), 0.), VehicleDef(), 1)
    a::LatLonAccel = LatLonAccel(0.0, 0)

    env::CrosswalkEnv = CrosswalkEnv(CrosswalkParams())
    sensor::AutomotiveSensors.GaussianSensor = AutomotiveSensors.GaussianSensor(AutomotiveSensors.LinearNoise(10, 0., 0.), 
                                                               AutomotiveSensors.LinearNoise(10, 0., 0.), 0, 0, MersenneTwister(1)) 

    sensor_observations::Vector{Vehicle} = []

    update_tick_emergency_system::Int64 = 1

    state_brake_system::Int32 = 0

    a_request::Float64 = 0
    a_current::Float64 = 0
    t_brake_trigger::Float64 = 0

    brake_request::Bool = false
    brake_release::Bool = false

    THRESHOLD_COLLISION_RATE::Float64 = 0.6
    THRESHOLD_TIME_TO_REACT::Float64 = 0.99

    TS_BREAK::Float64 = 0.2
    TD_BREAK::Float64 = 0.2
    AX_MAX::Float64 = -10.0
    SAFETY_DISTANCE_LON::Float64 = 1.0


    collision_rate::Float64 = 0.0
    risk::Float64 = 0.0
    ttc::Float64 = 0.0

    obstacles::Vector{ConvexPolygon}
    prediction_obstacle::Array{Float64} = []

end

  
function AutomotiveDrivingModels.observe!(model::EmergencySystem, scene::Scene, roadway::Roadway, egoid::Int)


    model.ego_vehicle = scene[findfirst(egoid, scene)]
    for i=2:scene.n
        object = scene[findfirst(i, scene)]
        if ( collision_checker(model.ego_vehicle.state, object.state, model.ego_vehicle.def, object.def) == true )
            println("collision with object: ", i, " ego.state: ", model.ego_vehicle.state, " object.state: ", object.state)
        end
    end

    model.sensor_observations = measure(model.sensor, model.ego_vehicle, scene, roadway, model.obstacles)
    
    ################ Emergency System #####################################################
    (ttb, stop_distance) = get_stop_distance_vehicle(model, model.ego_vehicle.state.v, 10.0, 1.0)
    
    model.risk = 0
    model.collision_rate = 0
    model.brake_request = false
    model.prediction_obstacle = []

    emergency_system_brake_request = false

    if ( model.tick % model.update_tick_emergency_system == 0 )

    	phi_dot = 0.0; t0 = 0.0; x0 = 0.0; y0 = 0.0; phi0 = 0.0; TS = 0.01; T = 3.0
        ego_data = caclulate_CTRA_prediction_simple(t0, x0, 0., y0, 0., phi0, 0., model.ego_vehicle.state.v, 0., TS, T, phi_dot, model.a_current)

        # calculate for every observation the risk
        for object in model.sensor_observations

            # calculation of the object data in the Frenet frame
            object_posF = Frenet(proj(object.state.posG, get_lane(model.env.roadway, model.ego_vehicle.state), model.env.roadway, move_along_curves=false),model.env.roadway)
            delta_s = object_posF.s - model.ego_vehicle.state.posF.s - model.ego_vehicle.def.length/2
            delta_d = object_posF.t - model.ego_vehicle.state.posF.t
            delta_theta = object_posF.ϕ - model.ego_vehicle.state.posF.ϕ

            # object is only in front of the ego vehicle relevant
            if (delta_s > 0 && model.ego_vehicle.state.v > 0.1)  

                # in general, trajectory should come from a prediction model!!
                # here a simple linear single track model is used! 
                object_data = caclulate_CTRA_prediction_simple(t0, delta_s, 0.1, delta_d, 0.1, 
                                                                delta_theta, 0.1, object.state.v, 0.1, TS, T, 0., 0.)

                # calculate the point/time where the two trajectories are closest
                (idx, ti) = get_conflict_zone(ego_data, object_data)

                # calculate collision probability based on pedestrian prediction
                (ttc, collision_rate, prediction_obstacle) = calculate_collision(ego_data, object_data, idx)

                # for visualization, only consider critical objects 
                if ( collision_rate > 0.2 )
                    model.collision_rate = collision_rate
                    model.prediction_obstacle = prediction_obstacle
 
                    ttc_m = mean(ttc)
                    ttc_std = std(ttc)
                    
                    model.ttc = ttc_m-2*ttc_std  # be safer
                    model.risk = min(ttb / model.ttc, 1.0)

                #   print(" collision_rate: ", model.collision_rate, " ttc_m: ", ttc_m, " ttc_std: ", ttc_std)
                #   println("Risk: ", model.risk, " ttc_min: ", model.ttc)
                end

                # intervention necessary: time to react < threashold AND collision risk < threashold
                if ( model.risk > model.THRESHOLD_TIME_TO_REACT && model.collision_rate >= model.THRESHOLD_COLLISION_RATE )
                    emergency_system_brake_request = true
                end 
            end
        end

        if ( emergency_system_brake_request == true )
            model.brake_request = true
        else
            model.brake_request = false
        end
    end
    
    # state machine for the brake system (limitation of required deceleration request)
    model = brake_system_state_machine(model, model.ego_vehicle.state)
    model.a = LatLonAccel(0., model.a_current)

    model.t_current = model.t_current + model.timestep 
    model.tick += 1
    
end


function AutomotiveDrivingModels.propagate(veh::Vehicle, action::LatLonAccel, roadway::Roadway, Δt::Float64)

    # new velocity
    v_ = veh.state.v + action.a_lon*Δt
    v_ = clamp(v_, 0., v_)

    # lateral offset
    delta_y = action.a_lat * Δt   # a_lat corresponds to lateral velocity --> a_lat == v_lat
    if v_ <= 0.
        delta_y = 0.
    end
    y_ = veh.state.posG.y + delta_y
    y_ = clamp(y_, -1.0, 1.0)

    s_new = v_ * Δt

    # longitudional distance based on required velocity and lateral offset
    delta_x = sqrt(s_new^2 - delta_y^2 )
    delta_x = clamp(delta_x, 0., delta_x)
    x_ = veh.state.posG.x + delta_x

    return VehicleState(VecSE2(x_, y_, veh.state.posG.θ), roadway, v_)
end


function AutomotiveDrivingModels.get_name(model::EmergencySystem)
    return "Emergency Braking System"
end

AutomotiveDrivingModels.rand(model::EmergencySystem) = model.a



"""
    is_crash(scene::Scene)
return true if the ego car is in collision in the given scene, do not check for collisions between
other participants
"""
function is_crash(scene::Scene)
    ego = scene[findfirst(1, scene)]
    @assert ego.id == 1
    if ego.state.v ≈ 0
        return false
    end
    for veh in scene
        if veh.id != 1
            if AutomotivePOMDPs.is_colliding(ego, veh)
                return true
            end
        end
    end
    return false
end

"""
    brake_system_state_machine(model::EmergencySystem, ego::VehicleState)
State machine to model the behavior of the brake system. Modification of the brake.state and the required deceleration
"""
function brake_system_state_machine(model::EmergencySystem, ego::VehicleState)
    

    if model.brake_release == true
        model.state_brake_system = 0
    end

    if model.state_brake_system == 0 
        #println("standby")
        model.a_current = 0
        if ( model.brake_request == true )
            model.state_brake_system = 1
        end
    end
    
    if model.state_brake_system == 1
        #println("brake_request")
        model.a_current = 0
        model.a_request = model.AX_MAX
        model.t_brake_trigger = model.t_current + model.TD_BREAK
        model.state_brake_system = 2
    end
    
    if model.state_brake_system == 2
        #println("brake_delay")
        model.a_current = 0
        if (model.t_current >= model.t_brake_trigger)
          model.state_brake_system = 3    
        end
    end
    
    if model.state_brake_system == 3
        #println("brake_ramp")
        model.a_current = model.a_current - 50 * model.timestep 
        if ( model.a_request >=  model.a_current )
            model.state_brake_system = 4
        end
    end
    if model.state_brake_system == 4
        #println("brake_max")
        if (ego.v > 0)
            model.a_current = model.a_request
        else
            model.a_current = 0.0
            model.state_brake_system = 0
        end
    end

    return model
end



"""
    get_stop_distance_vehicle(model::EmergencySystem, delta_v::Float64, ax_max::Float64, friction_coefficient::Float64)
Caclulation of the stop distance of the vehicle, based on the delay, ramp up phase and the maximum deceleration
return: time to brake and the distance to stop
"""
function get_stop_distance_vehicle(model::EmergencySystem, delta_v::Float64, ax_max::Float64, friction_coefficient::Float64)

    a = min(ax_max,friction_coefficient*10);
  
    v_s = delta_v - (a / 2) *  model.TS_BREAK;      # velocity after ramp up phase     
    s_s = v_s *  model.TS_BREAK;                    # distance during ramp up phase
    s_v = (v_s * v_s) / (2 * a);                    # distance to stop with full acceleration

    distance_to_stop = model.TD_BREAK * delta_v + s_s + s_v;
    distance_to_stop = distance_to_stop + model.SAFETY_DISTANCE_LON

    if ( distance_to_stop < 0.0) 
        distance_to_stop = 0
    end
    ttb = distance_to_stop / delta_v
  
    return ttb, distance_to_stop
  end



@with_kw mutable struct Trajectory
    a::Float32 = 0
    phi_dot::Float64 = 0
    X::Matrix{Float64} = Array{Float64}(undef, 0,9)
end


"""
    caclulate_CTRA_prediction_simple(t0::Float64, x0::Float64, x_s::Float64, y0::Float64, y_s::Float64,
                            phi0::Float64, phi_s::Float64, v0::Float64, v_s::Float64, TS::Float64, T::Float64, phi_dot::Float64, a::Float64)
Simple linear prediction for an object observation. A single track model with increasing standard deviation for the relevant states is used (time based). 
Calculation of the trajectory based on the current state and the current input (a and phi_dot). 
return: array with {t, x, y, phi, v, x_std, y_std, phi_std, v_std}
"""
function caclulate_CTRA_prediction_simple(t0::Float64, x0::Float64, x_s::Float64, y0::Float64, y_s::Float64,
                            phi0::Float64, phi_s::Float64, v0::Float64, v_s::Float64, TS::Float64, T::Float64, phi_dot::Float64, a::Float64)

    trajectory = Trajectory()
    trajectory.a = a;
    trajectory.phi_dot = phi_dot;

    time = 0:TS:(T-TS)
    X = Matrix(undef, length(time), 9)
    x = [t0 x0 y0 phi0 v0 x_s y_s phi_s v_s]
    X[1,:] = x

    STD_X = 0.2
    STD_Y = 0.1
    STD_NN = 0.1
    for i=2:length(time)
        phi = x[4]
        v = x[5]
        x = x + TS *[1 v*cos(phi) v*sin(phi) trajectory.phi_dot trajectory.a STD_X STD_Y STD_NN STD_NN ]
        X[i,:] = x
    end

    trajectory.X = X
    return trajectory
end

"""
    calculate_collision(object_1::Trajectory, object_2::Trajectory, critical_index::Int)
Calculates for the object prediction the collision risk and the time to collision in the Frenet frame
return: time to collision distribution, collision rate and predictions ot the object at the critical point
"""
function calculate_collision(object_1::Trajectory, object_2::Trajectory, critical_index::Int)
   
    time = object_1.X[:,1]
    ego_state_t0 = object_1.X[1,:]
    
    ttc = Float64[]
    collisions = 0

    ellipses = [] 

    if (critical_index < length(time) )

        ego = object_1.X[critical_index,:]
        ego_def = VehicleDef()
        ego_state = VehicleState(VecSE2(ego[2],ego[3],ego[4]),ego[5])
        
        object = object_2.X[critical_index,:]
        object_state = VehicleState(VecSE2(object[2],object[3],object[4]),object[5])

        # get objects position on ellipse
        ellipses = get_object_confidence_interval(object[2], object[6], object[3], object[7], object[4])
        object_positions = size(ellipses)[1]

        # collision check for every object position on the ellipse 
        for i = 1:object_positions
        
            object_state = VehicleState(VecSE2(ellipses[i,1],ellipses[i,2],object[4]),object[5])
            collision = AutomotivePOMDPs.collision_checker(ego_state, object_state, ego_def, AutomotivePOMDPs.PEDESTRIAN_DEF)
            if (collision)
                collisions = collisions + 1
                ttc_tmp = (ellipses[i,1]-ego_state_t0[2] + AutomotivePOMDPs.PEDESTRIAN_DEF.width/2) / ego_state_t0[5]
                push!(ttc, ttc_tmp)
            end
        end
        collision_rate = collisions/object_positions
    else
        collision_rate = 0
        push!(ttc, 100.0)
    end
    return ttc, collision_rate, ellipses
end


"""
    get_object_confidence_interval(x::Float64, x_sigma::Float64, y::Float64, y_sigma::Float64, theta::Float64)
Returns object positions based on the object state (prediction) for the 99% confidence interval error ellipse
"""
function get_object_confidence_interval(x::Float64, x_sigma::Float64, y::Float64, y_sigma::Float64, theta::Float64)

    # get the 99% confidence interval error ellipse
    confidence = 0.99
    k = (-2*log(1-confidence))^0.5
    
    x_s = k*x_sigma;
    y_s = k*y_sigma;
    
    # calculation of the ellipse
    theta_grid = LinRange(0.0, 2*pi, 20)

    ellipse_x  = x_s * cos.(theta_grid);
    ellipse_y  = y_s * sin.(theta_grid)

    # rotation via theta
    R = [ cos(theta) sin(theta); -sin(theta) cos(theta) ];
    ellipse = [ellipse_x ellipse_y] * R;

    # transform to x/y-position
    ellipse = [(ellipse[:,1] .+ x) (ellipse[:,2] .+ y)]
    return ellipse
end


"""
    get_conflict_zone(object_1::Trajectory, object_2::Trajectory)
Get the point/time where two trajectories are closest to each other 
"""
function get_conflict_zone(object_1::Trajectory, object_2::Trajectory)
    time = object_1.X[:,1]
    min_di = 500.
    min_di_idx = 500
    for i = 1:length(time)
        di = sqrt((object_1.X[i,2] - object_2.X[i,2])^2 + (object_1.X[i,3] - object_2.X[i,3])^2 )
        if ( di <= min_di )  
            min_di = di
            min_di_idx = i
        end
    end
    return min_di_idx, time[min_di_idx]
end
