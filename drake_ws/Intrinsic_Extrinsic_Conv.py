# Imports and Libs
import numpy as np
import os
import pandas as pd
from numpy import linalg as LA
import matplotlib  
matplotlib.use('TkAgg')   
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

import scipy as sc
from scipy.spatial.transform import Rotation as R

# File name for the csv
name = '_slash_my_gen3_slash_base_feedback.csv'

# Converts the csv data to Dataframe
df = pd.DataFrame(pd.read_csv(name))
saved_column_x = df.tool_pose_theta_x
saved_column_y = df.tool_pose_theta_y
saved_column_z = df.tool_pose_theta_z

#Save the angles
angles1 = np.array([np.transpose(saved_column_x), np.transpose(saved_column_y), np.transpose(saved_column_z)])
angle_values = np.transpose(angles1)
#In case you want to see the values
# print(angles1.shape)
# print(angle_values)
# print(len(df))


# Change each Euler angle from Intrinsic to Extrinsic
# Save in val
val = []

for i in range(len(df)):
    r = R.from_euler('zyx', angle_values[i], degrees=True)
    print(r.as_euler('ZYX', degrees=True))

    b = np.array(r.as_euler('ZYX', degrees=True))
    b = np.transpose(b)
    val = np.concatenate((val, b))

# Reshape the final val and insert columns in dataframe
val.shape = (len(df),3)
df.insert(184, "thetax", val[:,0], True)
df.insert(185, "thetay", val[:,1], True)
df.insert(186, "thetaz", val[:,2], True)

# Get current path
path = os.getcwd()
# print(path)

# Write the Dataframe back into csv
df.to_csv(path + r'/NN_slash_my_gen3_slash_base_feedback.csv', index = False, header=True)
