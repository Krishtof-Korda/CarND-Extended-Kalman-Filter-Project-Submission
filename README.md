## Korda's Extended Kalman Filter Project for Udacity Self-Driving Car Nanodegree

[YouTube Video](https://youtu.be/j6WeUe2YN6k)

[![alt text](https://img.youtube.com/vi/j6WeUe2YN6k/0.jpg)](https://youtu.be/j6WeUe2YN6k)



---

The goals / steps of this project are the following:

* Implement an Extended Kalman Filter for tracking a pedestrian.
* Familiarize myself with C++ coding, classes, compiling, etc.


## [Rubric Points](https://review.udacity.com/#!/rubrics/748/view) 

I tested the Extended Kalman Filter with the fusion of Lidar and Radar sensors.

### This is with fusion of Lidar and Radar:
![alt text][1]

  [1]: ./images/BestRMSE.jpeg 

I also tested the filter with Lidar data only which led to very good positional errors, but slightly worse velocity errors. This is to be expected since Lidar is very accurate in position but does not provide any velocity information. 
### This is with Lidar only:
![alt text][2]

  [2]: ./images/LaserOnlyUpdates.jpeg 

Lastly I tested the filter with Radar data only which led very poor positional and velocity errors. I expected the velocity errors to remain fairly good for this. However, since the positional information was very bad it affected the velocity calculations as well. Now we can see why the fusion of Lidar and Radar provides the best solution. 
### This is with Radar only:
![alt text][3]

  [3]: ./images/RadarOnlyUpdates.jpeg 
  
