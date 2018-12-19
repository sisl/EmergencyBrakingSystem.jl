module EmergencyBrakingSystem

using Random
using AutomotiveDrivingModels
using AutoViz
using AutomotiveSensors
using AutomotivePOMDPs
using Parameters
using StaticArrays
using Reel


export 
    EmergencySystem,
    generate_scenario,
    evaluateScenarioMetric,
    evaluateScenariosMetric

include("rendering.jl")
include("evaluation_scenario.jl")
include("emergency_system.jl")


end
