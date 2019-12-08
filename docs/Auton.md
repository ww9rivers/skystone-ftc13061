# Autonomous Mode

Many basic services could be build for an improved autonomous drive.

## Vuforia Object Recognition

Vuforia is a library licensed to FTC teams for the robot to recognize game objects, in 2019, for
example, the skystones and regular stones.

Object recognition is implemented in the ObjectDetector module.

## VuMark Navigation

VuMark navigation is (built on top of Vuforia?) used to recognize FTC navigation markers, to
help orient the robot.

Navigation is the module implementing this function.   

## Encoder Driving

Encoder driving allows the robot to be driven in specified directions with more accurate
control.

## IMU Gyroscope Orientation

The REV Extension Hub for FTC robot has a built-in IMU (Inertial measurement unit), which
helps the robot measure its own motions.

REVIMU is the module for interfacing with the IMU.

## Touch Sensor

A touch sensor is a simple digital device that provides a basic On/Off switch when its status
is read.

## Color / Distance Sencor

The REV color and distance sensor is a device integrating both functions.

## Ultrasound Distance Sensor

FTC robot kits do not usually contain this sensor.

## TensorFlow Lite

TensorFlow Lite is a machine learning tool kit open sourced by Google， which is used by the
object recognition and navigation code in the FRC Robot Controller.