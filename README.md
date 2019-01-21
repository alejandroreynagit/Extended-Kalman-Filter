# Extended Kalman Filter
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Writeup - Daniel Alejandro Reyna Torres

In this project, it is presented an (extended) kalman filter to estimate the state of a moving car with noisy lidar and radar measurements. The project implements C++ to write a program able to read and analyse randa and lidar measurements to track and predict the car position.

---

## The Project

The goals / steps of this project are the following:

* Use the UDACITY simulator to drive and test the (extended) kalman filter
* Compute RMSE and compare to a minimum given value
* Leave out radar/lidar feedback and compare results
* Test the kalman filter on both track 1 and track 2
* Summarize the results with a written report

---

#### 1. Build and Run

Assuming we have 'cmake', 'make' and 'uWebSocketIO' already installed, open a terminal:

* Clone this repository

* On the main project terminal:
    * Make a build directory: 
    
        ```
        mkdir build && cd build
        ```
    * Compile: 
        ```
        cmake .. && make
        ```
    * Run it: 
        ```
        ./ExtendedKF
        ```
* Using the Udacity provided simulator, select `Project 1/2 EKF and UKF`   in the main menu screen. Once the scene is loaded you can hit the START button to observe how the object moves and how measurement markers are positioned in the data set.

#### 2. Kalman Filter Alogorithm

The Kalman filter is an estimation algorithm used to estimate the state of a system given noisy and uncertain measurements. In this project, measurements are acquired through radar and lidar sensors.

Kalman filter basically work in a predict/update loop: [Check this site for more detail](https://classroom.udacity.com/nanodegrees/nd013/parts/edf28735-efc1-4b99-8fbb-ba9c432239c8/modules/49d8fda9-69c7-4f10-aa18-dc3a2d790cbe/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/a2a6c61f-afdd-47ae-9e9f-dcbe9916e772)

![](output/kf_loop.png)

First, we have a prior information of the object to track, we use this information to predict the state of that object until the next measurement arrives, this is the PREDICTION step. Then, we gather information from sensors (like lidar and radar) and use this information to courrect our belief about the state of the object, this correspond to the UPDATE step. 

In case we have multiple sensors, each sensor will have its own predcition update scheme. For the radar and lidar sensors, the prediction step would be the same, however, the measurement update would be different since both sensor "see" the world differently.

The Extended Kalman Filter come to the scenario when the system is not linear, this is the case when working with radar measurements. It linearise the system distribution around the mean of the current state and then use this result in the prediction and update states of the Kalman Filter algorithm.

#### 3. Testing

The udacity simulator presents the following initial screen:

![](output/Sim_ini.png)

We start the project and run the test on Dataset 1. Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and the prediction are the green triangles:

![](output/Sim_run.png)

Finally, I tested on Dataset 2, but also leaving out the radar on one run and then the same with lidar. Results are the following:



#### 4. Final Results

The final step and at some point hard, because of libraries and communication setups, was to run the simulator to see how well the car was driving around the given track and that measures such as RMSE, position and velocity were properly caluclated and displayed.

Overall picture of testing on track 1:

![](output/Sim_end.png)

At the end of the process, the kalman fuilter is able to predict the car position around the track with decent RMSE values as compared with those in the rubric!

Here's a [link to my video result](output/Results.mov).

---

## Discussion

Very interesting way to track the car position but also useful to track other objects. Programming in C++ could be quite challenging if this is not an every-dat programming language for anybody. Final results shown how powerful is the Kalman filter.


Thank you for reading this report.

_Daniel_