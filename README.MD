
Script for calculating movements parameters using python3.
Main goal to calculate parameters, mostly timings, using only acceleration, deceleration, velocity, how it usually be with servo drives
It is designed to both be easy to use from the command line but also be modular to be integrated and customized from other python scripts.


## Documentation

The documentation of calculation is not hosted [here](https://127.0.0.1).

##  Current state of code
Currently code can calculate movements with zero and non-zero start and end velocity, you can use slowing to slow down system
in cases when you need while moving. 
I use this calculation in example App to get timings in 2D Cartesian Portal for Pick and Place application
in industrial case packing systems and so on.