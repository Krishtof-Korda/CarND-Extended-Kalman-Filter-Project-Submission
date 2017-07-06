## Korda's Extended Kalman Filter Project for Udacity Self-Driving Car Nanodegree

[YouTube Video](https://youtu.be/9hsFGzllfVM)

[![alt text](https://img.youtube.com/vi/9hsFGzllfVM/0.jpg)](https://youtu.be/9hsFGzllfVM)



---

The goals / steps of this project are the following:

* Implement an Extended Kalman Filter for tracking a pedestrian.
* Familiarize myself with C++ coding, classes, compiling, etc.


## [Rubric Points](https://review.udacity.com/#!/rubrics/748/view) 

This project used a simulator to stream positional and velocity sensor data from Lidar and Radar returns of a pedestrian walking. The Kalman filter then provided its estimate of the position and velocity of the pedestrian after each measurement. If the measurement was Lidar a standard Kalman filter was used. If the measurement was Radar, then the Extended Kalman filter was used. The Extended Kalman filter simply converts the predicted state value form x, y position and velocity into range, angle, and range rate in order to execute a parameter update. This is done because the measurement space of Radar is range, angle, and range rate. 

I tested the Extended Kalman Filter with the fusion of Lidar and Radar sensors. The fused sensors have positional root mean squared errors on the order of 0.09, while the velocity errors are on the order of 0.45. Here is an image of the output visualized with green being the filter estimation, and red and blue being the lidar and radar sensor data. Later I tested the filter with only radar or only lidar to see how the different sensors contribute to the errors.  

### This is with fusion of Lidar and Radar:
![alt text][1]

  [1]: ./images/LaserRadarUpdates.jpeg 


I also tested the filter with Lidar data only which led to an increase in positional and velocity error of 51% and 41% respectively. While this may seem high, it can be seen later that the radar only fares much worse. The Lidar sensor alone provides a very good estimation of position and velocity using the linearized Kalman model. 


### This is with Lidar only:
![alt text][2]

  [2]: ./images/LaserOnlyUpdates.jpeg 

Lastly I tested the filter with Radar data only which led very poor positional and velocity errors. The position and velocity errors increased by 256% and 78% respectively. I expected the velocity errors to remain fairly good for this based on what we learned in the lessons. Radar was touted as more accurate in velocity than Lidar. My guess is that since the Kalman filter uses both velocity and position in its calculations, the extremely high positional errors skewed the velocity estimation. Now we can see why the fusion of Lidar and Radar provides the best solution. 


### This is with Radar only:
![alt text][3]

  [3]: ./images/RadarOnlyUpdates.jpeg 
  

This project was an excellent introduction to C++ and Kalman filters. I learned very much from the templated code that was provided. I feel much more comfortable with C++, classes, and Kalman filters.

Thank you again to the content creators at Udacity for expanding my horizons. 
