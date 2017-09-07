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

|            | R 1    | R 2    | R 3    | R 4    | R 5    | R 6    |
|------------|--------|--------|--------|--------|--------|--------|
| P_(1,1)    | 1      | 1      | 1      | 0.15   | 0.15   | 0.15   |
| P_(2,2)    | 1      | 1      | 1      | 0.15   | 0.15   | 0.15   |
| std_a_     | 30     | 3      | 3      | 3      | 3      | 3      |
| std_yawdd_ | 30     | 3      | 0.3    | 0.3    | 0.3    | 0.3    |
| radar      | true   | true   | true   | true   | false  | true   |
| laser      | true   | true   | true   | true   | true   | false  |
| px         | 0.0965 | 0.0775 | 0.0800 | 0.0786 | 0.2356 | 2.0519 |
| py         | 0.1202 | 0.0909 | 0.0878 | 0.0851 | 0.1590 | 1.4467 |
| vx         | 0.8101 | 0.3631 | 0.3449 | 0.3307 | 3.6168 | 2.2104 |
| vy         | 1.0176 | 0.4410 | 0.4045 | 0.3067 | 3.4144 | 3.1160 |

**P. S. :** The letter `R` stands for `Reading` in the column names of above table.

[image1]: ./img/Reading-01.PNG "Reading-01"
[image2]: ./img/Reading-02.PNG "Reading-02"
[image3]: ./img/Reading-03.PNG "Reading-03"
[image4]: ./img/Reading-04.PNG "Reading-04"
[image5]: ./img/Reading-05.PNG "Reading-05"
[image6]: ./img/Reading-06.PNG "Reading-06"

** Using default process noise values i.e. std_a_ = 30 & std_yawdd_ = 30 **
![alt text][image1]

** Using process noise values of std_a_ = 3 & std_yawdd_ = 3 **
![alt text][image2]

** Using process noise values of std_a_ = 3 & std_yawdd_ = 0.3 **
![alt text][image3]

** Using process noise values of std_a_ = 3 & std_yawdd_ = 3 **
** Initilizing Process covariance matrix using values of std_laspx_ & std_laspy_ **
![alt text][image4]

** Tracking using only Laser data while keeping Radar data off. **
![alt text][image5]

** Tracking using only Radar data while keeping Laser data off. **
![alt text][image6]