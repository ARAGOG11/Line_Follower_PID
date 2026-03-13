# PID Line Follower Robot

This project implements a line-following robot using a PID control algorithm on Arduino.
PID control implementation inspired by common Arduino line follower approaches,
with custom tuning and motor control adjustments.

## Overview
The robot detects a line using infrared sensors and continuously corrects its path using PID control to maintain stable tracking.

## Hardware
- Arduino UNO
- IR Sensor Array
- L293D Motor Driver
- DC Gear Motors
- Robot Chassis

## Control Algorithm
The PID controller adjusts motor speeds based on line position error.

Error → PID correction → motor speed adjustment

## Code
`line_follower_pid.ino`
