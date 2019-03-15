# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


## 1 Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. Other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## 2 The implementation

### 2.1 Path planning
Given that the map waypoints, and the perfect sensor fusion data was provided (seen on [line 224](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L224)), the task was to find a few sparsely spaced waypoints with which a valid trajectory can be generated. These waypoints can be used as anchor points for a spline, that guarantees that the final path goes through all waypoints with a smooth trajectory.

In case there was no previous path data, the car's current status was used as the basis (seen on [line 262](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L262)), otherwise the waypoint generation was continued from where the last planned path ends (seen on [line 272](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L272)), and so 3 points were generated with the easy use of the 's' axis for lane keeping, and the 'd' axis for lane changes, in Frenet coordinates (seen on [line 285](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L285)).

Then, to account for proper speed control, another set of points needed to be identified along the same path, with a spacing that gives the right speed, given the simulation execution frequency. Thanks to the help of external libraries, a spline was defined (seen on [line 306](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L306)).
In order to have a continuous path, that is not changing between each cycle, the previous path points were kept (seen on [line 315](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L315)), and only a small amount was generated to make up for the already passed ones. The amount of path points needed to be limited though, as having a too long previous path can hinder the ability to react to sudden events quickly enough.

### 2.2 Prediction & Behavior planning
So far, the code is able to follow the empty road in a given lane with a smooth trajectory, but high level decisions  needed to be incorporated too, to account for the surrounding traffic. This means the sensible combination of speed & lane selection, that gives a collision free, efficient and comfortable means of travel.

First, I created some auxiliary functions, that provide relevant information about the surrounding travel. The `get_vehicle` function (seen on [line 34](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L34)) gives us the vehicle that is closest to the Ego car in any selected lane, either forward or backward, within a selectable range. Then, I wrote the `behavior` function, which is responsible for setting a reference speed and a reference lane for the path planner (seen on [line 83](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L83)). It assigns a cost to each lane, based on the following factors:

* if there's a vehicle ahead,
  * the cost is higher, the slower the vehicle is (seen on [line 107](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L107))
  * the cost is higher, the closer the vehicle is (seen on [line 108](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L108))
* the cost is lowered for the ego lane, to discourage too many lane changes (seen on [line 119](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L119))
* the cost is increased extensively, when another vehicle is closely behind in the other lanes (seen on [line 124](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L124))

The resulting behavior is collision free and picks lanes with optimum speed.

Then the behavior planning consisted on changing lanes, if the neighboring lane means a lowered cost (seen on [line 133](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L133), and setting the speed according to the speed of the followed vehicle, or the speed limit on an empty lane (seen on [line 147](https://github.com/rdhelli/CarND-Path-Planning-Project/blob/8818185bdb349ef0092e41d5c3cdf810b4b24829/src/main.cpp#L147))
