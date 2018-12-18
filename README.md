# Quadcopter-hovering-control
A PID controller which can help quadcopter hovering. 

## Background
At present, most search/personal usage quadcopter(drone) uses cheap but low accuracy IMU. However, in this situation, it's really hard for drones to achiever hovering without RC controller. This project is to using additional sensor to help drones hovering automatically.

## Algorithm
Cascade PID controller. One is for velocity control, the other one is for movement control.

## Sensor
px4flow optical flow sensor
