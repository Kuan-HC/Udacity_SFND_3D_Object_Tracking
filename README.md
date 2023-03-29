# SFND 3D Object Tracking

<img src="images/readme.gif" width="900"/>


## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
  * Install Git LFS before cloning this Repo.
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Rubic

### FP.1 Match 3D Objects
<b> Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences. </b>

1. Use matches to find the links between bounding boxes in previous and current frame
2. Consider a point could fall in multiple bounding boxes, use a 2D array to count how
   many points fall in previous and current frames bounding boxes
3. Pick pairs in each row with highest scores

### FP.2 Compute Lidar-based TTC
<b>Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.</b>

1. Compute the mean x distance in previous and current frame to alleviate outliers and noise
2. Use the difference between the average x distance in the previous and current frame 
3. Divide the current mean distance by the velocity to get the TTC

### FP.3 Associate Keypoint Correspondences with Bounding Boxes
<b> Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box. </b>

1. Loop through the matched points and check if they are inside the bounding box
2. Log the points that are inside the bounding box and compute their distance
3. Set a distance threshold based on the mean distance value, for example 1.2 times the mean distance value
4. Loop through the points logged in step 1 and add them to the bounding box keypoints if their distance is lower than the threshold

### FP.4 Compute Camera-based TTC
<b> Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.</b>
1. Loop through the matches from task 3 to get keypoints in both frames
2. Compute and store the distance ratio 
3. Use the median distance ration to get TTC

### FP.5 Performance Evaluation 1
<b>Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.</b>  

<img src="images/rawLidarPoints.gif" width="150"/>
<img src="images/filterLidarPoints.gif" width="150"/> 

<b>Left:</b> raw lidar points <b>Right:</b> filtered lidar points  

#### Min. Distance
|Data\Frame| 0 - 1 | 1 - 2 | 2 - 3 | 3 - 4 | 4 - 5 | 5 - 6 | 6 - 7 | 7 - 8 | 8 - 9 | 9 - 10|10 - 11|11 - 12|12 - 13|13 - 14|14 - 15|15 - 16|16 - 17|17 - 18| 
|:---      |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |:---   |
|Raw data  |7.913  |7.849  |7.793  |7.685  |7.638  |7.577  |7.555  |7.475  |7.434  |7.393  |7.205  |7.272  |7.194  |7.129  |7.042  |6.827  |6.896  |6.814  |
|Filtered  |7.949  |7.879  |7.839  |7.794  |7.729  |7.653  |7.601  |7.549  |7.496  |7.443  |7.373  |7.308  |7.222  |7.159  |7.072  |6.994  |6.932  |6.843  |
|Raw TTC   |13.03  |7.879  |9.115  |5.061  |4.954  |5.034  |7.756  |5.991  |6.518  |7.226  |3.097  |7.235  |6.335  |7.697  |6.044  |2.854  |7.073  |5.799  |
|Filtered TTC|13.03|7.575  |10.59  |10.67  |8.493  |6.957  |7.756  |8.482  |9.254  |8.860  |7.927  |7.612  |6.878  |8.043  |6.671  |7.064  |7.702  |6.164  |


The figures above show how we select the relevant lidar points for this task. Ideally, we would use the point that is closest, but this point could be  noise or outliers Therefore, we use the mean value of the lidar points in the ROI. This way, lidar points that are not from the closest part or noise are both included.

### FP.6 Performance Evaluation 2
<b>Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons. </b>

<b> Max Lidar and Camera TTC Difference (s) </b>

| Detector\Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| ---                 | :---  | :---  |:--- |:---   |:---   |:---  |
| SHITOMASI           | 23.72  | 6.64   | 3.57 | 6.36   | -     | 5.43  |
| HARRIS              | 13.73   | 16.8   | inf | 5.02   | -     | 40.15  |
| FAST                | 26.84   | 79.06   | 13.47 | 5.69   | -     | 30.75  |
| BRISK               | 13.2  | 13.28  | 7.84| 11.62  | -     | 23.01 |
| ORB                 | 5406   | 259.2   | 22.67 | inf   | -     | inf  |
| AKAZE               | 2.92  | 3.24  | 3.22| 2.49  | 3.77  |4.31  |
| SIFT                | 5.6   | 7.72   | -   | 5.62   | -     |4.39   |

The top three detector descriptor combination are Akaze-Freak, Akaze-Brisk and Akaze - Orb.
The key factor of calculating camera TTC is the matched key points. A bad key point matched could lead to unstable camera TTC estimation as shown in figures below.

<img src="images/d_1.png" width="600"/>  
<img src="images/d_2.png" width="600"/> 