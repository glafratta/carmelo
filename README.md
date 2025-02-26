# CloCk: Closed-loop control with Core Knowledge
The purpose of this library is to showcase a framework for multi-step ahead plannig using pure input control. The navigation problem is broken down into several unique closed-loop input controllers, called Tasks. Each tasks produces a unique control behaviour (go straight, turn left/right 90 degrees) in response to a disturbance object. A supervising module, called the Configurator, can simulate sequences of Tasks  in game engine [Box2D](https://github.com/erincatto/box2d), extracts plans in the discrete and continuous domain, and queue them for execution.

## Hardware
The indoor robot is equipped with 
* 360 Parallax Continuous Rotation Servo motors (see [here](https://github.com/berndporr/alphabot/blob/main/alphabot.cpp) for wiring)
* A1 SLAMTEC LIDAR (see [here](https://github.com/berndporr/rplidar_rpi) for wiring)
* Raspberry Pi model 3b+

## Prerequisites
- Bullseye release of Raspberry Pi OS
### Development packages

* G++ compiler
* CMake
* PiGPIO library
* OpenCV
* Boost
* XOrg
* LibGLU1

`sudo apt install g++ cmake libpigpio-dev libopencv-dev libboost-all-dev xorg-dev libglu1-mesa-dev`

### Compile from source

* [LIDAR API](https://github.com/berndporr/rplidar_rpi)
* [Motors API](https://github.com/berndporr/alphabot)
* [Cpp Timer](https://github.com/berndporr/cppTimer)
* [Box2D v2.4.1](https://github.com/glafratta/box2d)
  ** if not installed automatically, go to `box2d/build` and run `sudo make install`

## Optional: tweak constants
There are some constants pertaining to our robot's kinematic model (e.g. speed, dimensions...). If you use a different platform than alphabet, you can tweak the robot's model by changning constants in `src/const.h`.

## Build
```
cd CloCK
cmake .
sudo make install
```

## Run
### Navigation demo (Raspberry Pi)
* `sudo ./targetless` : this program demonstrates planning over a 1m distance horizon for a control goal that is not a target location but rather an objective to drive straight for the longest time with the least amount of disturbances
* `sudo ./target`: this program demonstrates target seeking behaviour, where the target is imaginary and located at x=1.0m, y=0m.


Run with options `0`: for turning debug options off. In debug mode, LIDAR coordinates, Box2D objects and robot trajectories are dumped into the `/tmp` folder. Additional option to turn planning on/off by adding `0` or `1`. 
Examples:
- `./target 0` : runs with debugging off (0) and planning on (default)
- `./target 1 0` : runs with debugging on (1) and planning off (0)



