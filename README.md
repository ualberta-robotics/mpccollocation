# MPC style Direct Collocation for Kinova Trajectory Planning
Repository containing code snippets in Python and Matlab for Kinova Trajectory Optimization

## The Procedure Step-By-Step

1. ### Recording robot trajectories and saving to rosbag files
    For recording the trajectories of the WAM or the Kinova, the basic command we use is the rosbag record - shown as :
    ```
    rosbag record -O bag_filename.bag /topic
    ```
    In our case it is:
    ```
    rosbag record -O /hulk/home/Compiles/pbdlib-python/pbdlib/data/RobotData /my_gen3/feedback_base
    ```

2. ### Converting the rosbag to csv
    Using a simple conversion script ```bag_to_csv.py``` convert rosbag to csv

    #### Usage
    Fairly simple. Just run ```python bag_to_csv.py``` in the folder with all the bag files
    A folder is created with an all important file named - ```_slash_my_gen3_slash_base_feedback.csv```
    For each of the folders, there will be a separate .csv file and we NEED to modify this before actually using it in MATLAB.

3. ### Processing the csv file
    The processing of the csv basically adds 3 columns each for the orientation theta x, theta y and theta z in the *extrinsic coordinate* frames. This is important because the original Euler angles of the robot are in Intrinsic Coordinates. 

    Simply run ```python Intrinsic_Extrinsic_Conv.py ``` for each of the csv files. The script assumes that the csv is in the same folder as the script. 
    Run this for all csv files you have.

    A good way will be to have the folders name *t1* to *tn* for *n* trajectories and have all csv files within each folder

4. ### CSV to MAT
    Awesome! We are almost done!

    Now open up MATLAB and navigate to the m file ```cleancsvtomat.m```. The m file assumes that you have the csv files within each forder name t1 to tn for n trajectories.
    Make sure the path of the workspace is same as that of the folders and run the cleancsvtomat.m

    *What this does is make arrays posdemos and veldemos for the trajectory points, basically filtering into smoother spline like trajectories with lesser jerks and consistent size* 

