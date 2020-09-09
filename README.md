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

    Simply run 
    
    ```
    python Intrinsic_Extrinsic_Conv.py 
    ``` 
    
    for each of the csv files. The script assumes that the csv is in the same folder as the script. 
    Run this for all csv files you have.

    A good way will be to have the folders name *t1* to *tn* for *n* trajectories and have all csv files within each folder

4. ### CSV to MAT
    Awesome! We are almost done!

    #### First Step
    Now open up MATLAB and navigate to the m file ```Extrinsic_CSV_to_MAT.m```. The m file assumes that you have the csv files within each forder name t1 to tn for n trajectories. Make changes to the number of trajectories in the for loop of the code and also the filename if you don't want the 't' foldername.
    Make sure the path of the workspace is same as that of the folders and run the Extrinsic_CSV_to_MAT.m

    *What this does is make arrays posdemos and veldemos for the trajectory points, basically filtering into smoother spline like trajectories with lesser jerks and consistent size* 

    #### Second Step
    Run ```collect_2D_data_3D.m``` to finally get the dynamics of the system along with the streamlines. You can see each coefficient of x,y,z for the given order of the system. 

    This is what we will use as the Collocation Dynamics of the system.

5. ### Editing and Running the Collocation

    Navigate to the ```collocation.py``` file and in the *qddot* value, edit the coefficients of the system dynamics according to the ones obtained in the previous section.

    x[0], x[1] and x[2] states correspond to the x, y and z states respectively (just an easy way for the model to understand)

    If on a separate system instead of *vis*, use ssh to logon as:
    
    ```ssh 192.168.1.60```
    and run 

    ```
    roslaunch kinova_joy kinova_gmm_control_no_gui.launch
    ```

    *Thanks to Jakub, this script helps in interfacing with the Kinova beautifully*

    And run our very own ```collocation.py``` in a separate terminal

    Nothing moves, right?

    We just need to give a starting message to the arm telling it's okay! You're good to go.

    On the *vis* system terminal, use the following:

    ```
    rostopic pub /kinova_selected_obj kinova_joy/selectedObject "centroid:
     x: 0.0
     y: 0.0
     z: 0.0
    axis:
     x: 0.0
     y: 0.0
     z: 0.0
    length_to_top: 0.0"
    ``` 
    you can hit *double tab* to autofill the default values to 0.0

    This sends out a dummy message to start the process.

    *And now !!! .... (drumroll)*

    *The Kinova moves to a starting point followed by a trajectory*

6. ### Time to Shine!

    Now that we have the trajectory, we can visualize it easily. The trajectories are all basically stored in *vis* when the code running is complete in the folder named *data_for_dhruv*

    The folder itself saves the trajectories in the form of time stamped txt files so its easy for you to see the most recent one.

    For the latest run txt file you want to analyse, use the ```cp``` copy command to copy to filename ```source.txt```
    Example:
    ```
    cp 2020-09-09T20\:43\:23.167961_gmm_data.txt source .txt
    ```

    Now navigate to and run ```./analysis_script.bash``` 

    This creates 4 different files each containing - 
    * Commanded Position
    * Current Position
    * Commanded Angles
    * Current Angles

    The  ```plot_traj.py```  runs the dynamics of the streamlines plotting it along the trajectories. You can modify the ```plot_traj.py``` file according to the desired dynamics. 


*This work is the combined effort of Computer Vision and Robotics Group at the University of Alberta, under the supervision of Dr. Martin Jagersand*


