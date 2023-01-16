# arduino-line-follower

This repository contains the code for an the code for an Arduino line follower with 2 motors built as a project for the Robotics Introduction Course taken in the 3rd year at the Faculty of Mathematics and Informatics of Bucharest.

## Components

- Arduino Uno
- Light sensors
- Motor driver
- 2X Motors
- LiPo battery

## How it works

The robot has to follow a black line agains a white background to complete a circuit. The circuit contains turns and twists that the robot has to follow.

Internally, with the data from the light sensor, a PID controll system is used to decide the speed of each of the 2 motors in order to properly take all the turns. 

When the arduino starts it firsts performs left to right sweeps while calibrating the sensor. The sweeps ensure that each sensor "sees" both the white background and the black line in order to save the minimum and maximum luminosity values

## Image


## Video 



https://user-images.githubusercontent.com/38132701/212675247-779ef3a6-4b43-4410-89d6-6d6842793395.MOV

