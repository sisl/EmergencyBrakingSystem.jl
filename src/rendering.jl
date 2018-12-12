
@with_kw struct PretictionOverlay <: SceneOverlay
    prediction::Array{Float64} = Array{Float64}()
    color::Colorant = RGBA(1., 0.0, 0.0, 0.2)
end

function AutoViz.render!(rendermodel::RenderModel, overlay::PretictionOverlay, scene::Scene, roadway::R) where R

    predictions = overlay.prediction
    ego = scene[findfirst(1, scene)]
    for i=1:size(predictions)[1]
      pos = predictions[i,:]
      ped = Vehicle(VehicleState(VecSE2(pos[1], pos[2], 1.57), 0.), AutomotivePOMDPs.PEDESTRIAN_DEF, 1)
      add_instruction!(rendermodel, render_vehicle, (ego.state.posG.x+ped.state.posG.x+ego.def.length/2-AutomotivePOMDPs.PEDESTRIAN_DEF.width/2, ego.state.posG.y+ped.state.posG.y, ped.state.posG.θ, ped.def.length, ped.def.width, overlay.color))
    end
    return rendermodel
end


function animate_record(rec::SceneRecord,dt::Float64, env::CrosswalkEnv, ego_vehicle::Vector{Vehicle}, sensor::GaussianSensor, sensor_o::Vector{Vector{Vehicle}}, 
                             risk::Vector{Float64}, ttc::Vector{Float64}, collision_rate::Vector{Float64}, brake_request::Vector{Bool}, prediction::Vector{Array{Float64}}, cam=FitToContentCamera(0.0))
     
    duration =rec.nframes*dt::Float64
    fps = Int(1/dt)

    function render_rec(t, dt)
        frame_index = Int(floor(t/dt)) + 1
        text_to_visualize = [string("v-ego: ", ego_vehicle[frame_index].state.v*3.6, " km/h", " TTC: ", ttc[frame_index], 
                                " Collision rate: ",  collision_rate[frame_index], " Risk: ",  risk[frame_index], 
                                " Brake request: ",  brake_request[frame_index])]
       
        sensor_overlay = GaussianSensorOverlay(sensor=sensor, o=sensor_o[frame_index])
        occlusion_overlay = OcclusionOverlay(obstacles=env.obstacles)
        prediction_overlay = PretictionOverlay(prediction=prediction[frame_index])
        text_overlay = TextOverlay(text=text_to_visualize,pos=VecE2(20.,10.),incameraframe=true,color=colorant"white",font_size=15)
        return render(rec[frame_index-nframes(rec)], env, [prediction_overlay, sensor_overlay, occlusion_overlay, text_overlay, IDOverlay()], cam=cam)

    end
    return duration, fps, render_rec
end

@with_kw mutable struct ObservationCallback
    ego_vehicle::Vector{Vehicle}
    risk::Vector{Float64}
    collision_rate::Vector{Float64}
    ttc::Vector{Float64}
    brake_request::Vector{Bool}
    prediction::Vector{Array{Float64}}
    sensor_observations::Vector{Vector{Vehicle}}
    collision::Vector{Bool}
    a::Vector{Float64}
end

function AutomotiveDrivingModels.run_callback(
        callback::ObservationCallback,
        rec::EntityQueueRecord{S,D,I},
        roadway::R,
        models::Dict{I,M},
        tick::Int) where {S,D,I,R,M<:DriverModel}

    push!(callback.ego_vehicle, models[1].ego_vehicle)
    push!(callback.risk, models[1].risk)
    push!(callback.collision_rate, models[1].collision_rate)
    push!(callback.ttc, models[1].ttc)
    push!(callback.brake_request, models[1].brake_request)
    push!(callback.prediction, models[1].prediction)
    push!(callback.sensor_observations, models[1].sensor_observations)
    push!(callback.a, models[1].a_current)

    collision = is_crash(rec[0])
    push!(callback.collision, collision)

    return collision
end

