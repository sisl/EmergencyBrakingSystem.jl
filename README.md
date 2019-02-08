# detailed documentation follows soon ...

# EmergencyBrakingSystem

[![Build Status](https://travis-ci.org/sisl/EmergencyBrakingSystem.jl.svg?branch=master)](https://travis-ci.org/sisl/EmergencyBrakingSystem.jl)
[![Coveralls](https://coveralls.io/repos/github/sisl/EmergencyBrakingSystem.jl/badge.svg?branch=master)](https://coveralls.io/github/sisl/EmergencyBrakingSystem.jl?branch=master)

# EmergencyBrakingSystem: Emergency Braking System for Pedestrians

contact: Markus Schratter, [m.schratter@gmx.at](m.schratter@gmx.at) Maxime Bouton, [boutonm@stanford.edu](boutonm@stanford.edu)


## CPAN25 50km/h
![](docs/CPAN25_50kmh_emergency_brake_intervention.gif)
<img src="docs/CPAN25_50kmh_emergency_brake_intervention.gif" width="30%">


## False Positive Test
![](docs/CPAN25_50kmh_FP_test.gif)
A pedestrian corsses the road, but there is no brake intervention required.

## Installation

To install this package and its dependency run the following in the julia REPL:
```julia
using Pkg
Pkg.add(PackageSpec(url="https://github.com/sisl/Vec.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/Records.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveDrivingModels.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutoViz.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveSensors.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutoUrban.jl"))
Pkg.add(PackageSpec(url="https://github.com/JuliaPOMDP/RLInterface.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotivePOMDPs"))
Pkg.add(PackageSpec(url="https://github.com/sisl/EmergencyBrakingSystem.jl"))
```


## TODOs



## Dependencies

- AutomotiveDrivingModels.jl
- POMDPs.jl

## Code to run

Run `test.ipynb` for a visualization of the different scenarios.

## Folder structure
