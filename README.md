### Unscented-Kalman-Filter ###

Submitted by - Vishal Rangras

The goals for this project are to:

1. Initialize the UKF.
2. Tune the process noise.
3. Generate the Augmented Sigma Points.
4. Predict Sigma Points.
5. Predict State Mean Vector and Process Covariance Matrix.
6. Update the State using Laser and Radar Measurement.
7. Compute Cross Relation Matrix and Kalman Gain.
8. Compute NIS for state prediction after update step.
9. Calculate RMSE for px, py, vx and vy.

**[Rubric](https://review.udacity.com/#!/rubrics/783/view) Points**

### Building the Project and Execution ###

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./UnscentedKF`
5. See the results in Simulator

### Results ###

[image1]: ./img/Dataset-01.png "Dataset-01"
[image2]: ./img/Dataset-02.png "Dataset-02"

![alt text][image1]
![alt text][image2]