
@with_kw struct PredictionOverlay <: SceneOverlay
    prediction::Array{Float64} = Array{Float64}()
    color::Colorant = RGBA(1., 0.0, 0.0, 0.2)
end

function AutoViz.render!(rendermodel::RenderModel, overlay::PredictionOverlay, scene::Scene, roadway::R) where R

    predictions = overlay.prediction
    ego = scene[findfirst(1, scene)]
    for i=1:size(predictions)[1]
      pos = predictions[i,:]
      ped = Vehicle(VehicleState(VecSE2(pos[1], pos[2], 1.57), 0.), AutomotivePOMDPs.PEDESTRIAN_DEF, 1)
      add_instruction!(rendermodel, render_vehicle, (ego.state.posG.x+ped.state.posG.x+ego.def.length/2-AutomotivePOMDPs.PEDESTRIAN_DEF.width/2, ego.state.posG.y+ped.state.posG.y, ped.state.posG.θ, ped.def.length, ped.def.width, overlay.color))
    end
    return rendermodel
end


function animate_record(scenes::Vector{Scene},dt::Float64, env::CrosswalkEnv, ego_vehicle::Vector{Vehicle}, sensor::GaussianSensor, sensor_o::Vector{Vector{Vehicle}}, 
                             risk::Vector{Float64}, ttc::Vector{Float64}, collision_rate::Vector{Float64}, brake_request::Vector{Bool}, prediction::Vector{Array{Float64}}, cam=FitToContentCamera(0.0))
     
    duration = length(scenes)*dt::Float64
    fps = Int(1/dt)

    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        text_to_visualize = [string("v-ego: ", round(ego_vehicle[frame_index].state.v*3.6,digits=1), "km/h", " TTC: ", round(ttc[frame_index],digits=1), "s", 
                                " Collision: ",  collision_rate[frame_index], " Risk: ",  round(risk[frame_index],digits=1), 
                                " Brake request: ",  brake_request[frame_index])]
       
        sensor_overlay = GaussianSensorOverlay(sensor=sensor, o=sensor_o[frame_index])
        occlusion_overlay = OcclusionOverlay(obstacles=env.obstacles)
        prediction_overlay = PredictionOverlay(prediction=prediction[frame_index])
        text_overlay = TextOverlay(text=text_to_visualize,pos=VecE2(75.,8.),incameraframe=true,color=colorant"white",font_size=20)
        return render(scenes[frame_index], env, [prediction_overlay, sensor_overlay, occlusion_overlay, text_overlay, IDOverlay()], cam=cam)

    end
    return duration, fps, render_rec
end


@with_kw mutable struct ObservationCallback

    sensor_observations::Vector{Vector{Vehicle}}
    ego_vehicle::Vector{Vehicle}
    ego_a::Vector{Float64}
    collision::Vector{Bool}

    collision_rate::Vector{Float64}
    ttc::Vector{Float64}
    risk::Vector{Float64}
    brake_request::Vector{Bool}
    prediction_obstacle::Vector{Array{Float64}}

end

function AutomotiveDrivingModels.run_callback(
        callback::ObservationCallback,
        scenes::Vector{Scene},
        actions::Union{Nothing, Vector{Frame{A}}},
        roadway::R,
        models::Dict{I,M},
        tick::Int) where {A<:EntityAction,I,R,M<:DriverModel}

    push!(callback.sensor_observations, models[1].sensor_observations)
    push!(callback.ego_vehicle, models[1].ego_vehicle)
    push!(callback.ego_a, models[1].a_current)
    collision = is_crash(scenes[tick])
    push!(callback.collision, collision)

    push!(callback.collision_rate, models[1].collision_rate)
    push!(callback.ttc, models[1].ttc)
    push!(callback.risk, models[1].risk)
    push!(callback.brake_request, models[1].brake_request)
    push!(callback.prediction_obstacle, models[1].prediction_obstacle)

    return collision
end

