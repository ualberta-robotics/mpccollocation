#IMPORT LIBS
import numpy as np
from pydrake.solvers import mathematicalprogram
from pydrake.solvers.mathematicalprogram import Solve
from pydrake.symbolic import Variable
from pydrake.systems.primitives import SymbolicVectorSystem
import matplotlib.pyplot as plt
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogOutput
from pydrake.systems.framework import LeafSystem_
from pydrake.systems.scalar_conversion import TemplateSystem
from pydrake.systems.framework import BasicVector, LeafSystem, ContinuousState, VectorBase
from pydrake.systems.scalar_conversion import TemplateSystem
from pydrake.systems.framework import BasicVector_, LeafSystem_, PortDataType, System_, VectorBase_, ContinuousState_, SystemScalarConverter
import pydrake.systems.scalar_conversion as mut
from pydrake.systems.trajectory_optimization import DirectCollocation
from pydrake.trajectories import PiecewisePolynomial
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
file1 = open("123.txt","w") 
import timeit
import socket
import numpy as np
import struct
from numpy import linalg as LA
from scipy.signal import savgol_filter
from mpl_toolkits.mplot3d import Axes3D
from pytransform3d.rotations import *
import math
import time

from scipy.spatial.transform import Rotation as Rsc



#DEFINE THE SEND IP AND PORT
UDP_IP_SEND = "192.168.1.60"
UDP_PORT_SEND = 30034

#RECIEVE IP AND PORT
UDP_IP_RCV = "192.168.1.7"
UDP_PORT_RCV = 30033



#FUNCTION DEF
# #BOILERPLATE PLANT DEFINITION OF TO INCLUDE THE DYNAMICS
Dynamics = np.array((0., 0., 0.))
Dynamics = np.transpose(Dynamics)

@TemplateSystem.define("ex1_")
def ex1_(T):
    class Impl(LeafSystem_[T]):
        def _construct(self, converter=None):
            LeafSystem_[T].__init__(self, converter)
            # Inputs 
            self.DeclareVectorInputPort("u", BasicVector_[T](3))
            # Outputs
            self.DeclareVectorOutputPort("x", BasicVector_[T](3), self.CopyStateOut)
            # 2 positions
            self.DeclareContinuousState(3, 0, 0)

        def _construct_copy(self, other, converter=None):
            Impl._construct(self, converter=converter)

        def CopyStateOut(self, context, output):
            x = context.get_continuous_state_vector().CopyToVector()
            y = output.SetFromVector(x)

        def DoCalcTimeDerivatives(self, context, derivatives):
            global Dynamics 
            x = context.get_continuous_state_vector().CopyToVector()
            u = self.EvalVectorInput(context, 0).CopyToVector()
            q = x[:3]           
            qdot = x[3:]        

            # Plug In New Dynamics Here
            qddot = np.array([                      
                0.0094 - 0.0468*x[0] + 0.0667*x[1] - 0.0493*x[2] - 0.0226*x[0]*x[0] + 0.0417*x[0]*x[1] + 0.3267*x[0]*x[2] + 0.0692*x[1]*x[1] - 0.0087*x[1]*x[2] - 0.0079*x[2]*x[2] + u[0],
                0.0839 - 0.0471*x[0] - 0.1227*x[1] - 0.1645*x[2] - 0.0552*x[0]*x[0] + 0.0122*x[0]*x[1] + 0.0350*x[0]*x[2] - 0.2174*x[1]*x[1] + 0.3898*x[1]*x[2] - 0.0405*x[2]*x[2] + u[1],
                -0.0270 - 0.0876*x[0] - 0.0669*x[1] + 0.0345*x[2] + 0.1439*x[0]*x[0] + 0.0810*x[0]*x[1] - 0.0899*x[0]*x[2] + 0.0723*x[1]*x[1] - 0.0675*x[1]*x[2] + 0.1433*x[2]*x[2] + u[2],
            ])
            # Uncomment to see the dynamics
            # print(q)
            Dynamics = np.transpose(Dynamics)
            # Dynamics = np.concatenate((Dynamics, q))
            # print(q)
            file1.write(str(q))
            file1.write("\n\n")
            derivatives.get_mutable_vector().SetFromVector(np.concatenate((qdot, qddot)))
    return Impl

#Def to calculate the rotation mat
def returnrotmat(roll, pitch, yaw):
    rotmat = np.array([[math.cos(pitch)*math.cos(yaw), math.sin(roll)*math.sin(pitch)*math.cos(yaw)-math.cos(roll)*math.sin(yaw), math.sin(roll)*math.sin(yaw)+math.cos(roll)*math.sin(pitch)*math.cos(yaw)], 
                    [math.cos(pitch)*math.sin(yaw), math.cos(roll)*math.cos(yaw)+math.sin(roll)*math.sin(pitch)*math.sin(yaw), math.cos(roll)*math.sin(pitch)*math.sin(yaw)-math.sin(roll)*math.cos(yaw)], 
                    [-math.sin(pitch), math.sin(roll)*math.cos(pitch), math.cos(roll)*math.cos(pitch)]])

    return rotmat

#Def to return the angles
def returnxortn(rot_mat):

    yproj_basis = np.transpose(np.array([0.0, 1.0, 0.0]))

    mat_yproj = np.dot(rot_mat,yproj_basis)
    X_diff_x = mat_yproj[0]
    X_diff_z = mat_yproj[2]
    print("X = ", X_diff_x)
    print("Z = ", X_diff_z)
    ortn_x = np.arctan2(X_diff_x,X_diff_z)

    return ortn_x

def returnyortn(rot_mat):

    zproj_basis = np.transpose(np.array([0.0, 0.0, 1.0]))

    mat_zproj = np.dot(rot_mat,zproj_basis)
    Y_diff_x = mat_zproj[0]
    Y_diff_y = mat_zproj[1]
    print("X = ", Y_diff_x)
    print("Y = ", Y_diff_y)
    ortn_y = np.arctan2(Y_diff_y,Y_diff_x)

    return ortn_y

def returnzortn(rot_mat):

    xproj_basis = np.transpose(np.array([1.0, 0.0, 0.0]))

    mat_xproj = np.dot(rot_mat,xproj_basis)
    Z_diff_y = mat_xproj[1]
    Z_diff_z = mat_xproj[2]
    print("Y = ", Z_diff_y)
    print("Z = ", Z_diff_z)
    ortn_z = np.arctan2(Z_diff_z,Z_diff_y)

    return ortn_z

#STATES
#Define the object of Direct Collocation
N = 10
max_dt = 0.5
umax = 2
max_tf = N * max_dt

# Prev state for ref
# initial_state_ground = np.array([0.30, 0.40, 0.6])
# final_state_ground = np.array([0.5, -0.6, -0.10])

# initial_angle_ground = np.array([126.048, 13.714, 71.547])
# final_angle_ground = np.array([143.496, -2.811, 80.515])

# New Testing state
# initial_state_ground = np.array([0.467, 0.314, 0.555])
# final_state_ground = np.array([0.6, -0.5, -0.10])

# initial_angle_ground = np.array([121.972, -10.511, 96.892])
# final_angle_ground = np.array([143.496, -2.811, 80.515])

# Test state 2
# initial_state_ground = np.array([0.404, 0.073, 0.693])
# final_state_ground = np.array([0.6, -0.5, -0.10])

# initial_angle_ground = np.array([136.544, 17.316, 75.348])
# final_angle_ground = np.array([143.496, -2.811, 80.515])


initial_state_ground = np.array([0.369, -0.151, 0.485])
final_state_ground = np.array([0.6, -0.5, -0.10])

initial_angle_ground = np.array([141.855, -3.476, 45.592])
final_angle_ground = np.array([143.496, -2.811, 80.515])

#DEFINE THE START POSITIONS
startposx = 0.30
startposy = 0.40
startposz = 0.6

#Input Limit
input_limit = np.array(4.0)  

XFINAL = np.array([[0.,0.,0.]])
UFINAL = np.array([[0.,0.,0.]])
COSTFINAL = np.array([])


#DIRECT COLLOCATION
#Define the plant
ex1 = ex1_[None]
plant = ex1()
#Define Context
context = plant.CreateDefaultContext()

Loop_run_iter = 20

XTraj = XFINAL
C = COSTFINAL

NewSavedXTraj = []
NewSavedUTraj = []
NewSavedCost = []


#LOOP FOR THE QUADRANT PLOTS
for quad in range(1):

    XTraj = XFINAL
    UTraj = UFINAL
    C = COSTFINAL
    initial_state = initial_state_ground
    final_state = final_state_ground
    i = 0
    err = []
    Of = 95.0
    Oo = 4.50

    start_alpha, start_beta, start_gamma = math.radians(initial_angle_ground[0]), math.radians(initial_angle_ground[1]), math.radians(initial_angle_ground[2])
    end_alpha, end_beta, end_gamma = math.radians(final_angle_ground[0]), math.radians(final_angle_ground[1]), math.radians(final_angle_ground[2])

    #for i in range(Loop_run_iter):
    err_in = LA.norm((final_state[:2] - initial_state[:2]),2)

    calc_state_x_old = initial_state[0]
    calc_state_y_old = initial_state[1]
    calc_state_z_old = initial_state[2]
    calc_state_x = initial_state[0]
    calc_state_y = initial_state[1]
    calc_state_z = initial_state[2]
    counter = 0
    send_flag = 0       #Just a flag to note the actual sending of data


    R_euler_end = returnrotmat(-end_alpha, -end_beta, -end_gamma)
    end_x_angle = returnxortn(R_euler_end)
    end_y_angle = returnyortn(R_euler_end)
    end_z_angle = returnzortn(R_euler_end)

    while LA.norm((final_state[:2] - initial_state[:2]),2) >= 0.005:

        error_norm = LA.norm((final_state[:2] - initial_state[:2]),2)
        print("The error =  ", error_norm)
        Ocalc = Of + (Oo - Of)*(error_norm/err_in)
        print("The y orientation = ", Ocalc)

        print("In loop")

        #Recieves the data from Robot
        sock_rcv = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
        sock_rcv.bind((UDP_IP_RCV, UDP_PORT_RCV))

        data_rcv, addr = sock_rcv.recvfrom(1024)
        data_rcv = struct.unpack('<8d',data_rcv)
        X_initial_ortn_rcvd = data_rcv[5]
        Y_initial_ortn_rcvd = data_rcv[6]
        Z_initial_ortn_rcvd = data_rcv[7]
        print(data_rcv)


        # end_x_angle = returnxortn(end_alpha, end_beta, end_gamma)

        R_euler_start = returnrotmat(-start_alpha, -start_beta, -start_gamma)
        curr_x_angle = returnxortn(R_euler_start)
        curr_y_angle = returnyortn(R_euler_start)
        curr_z_angle = returnzortn(R_euler_start)

        angle_x = math.degrees(end_x_angle - curr_x_angle)
        angle_y = math.degrees(end_y_angle - curr_y_angle)
        angle_z = math.degrees(end_z_angle - curr_z_angle)


        R_x = matrix_from_angle(0,-angle_x)
        R_y = matrix_from_angle(1,-angle_y)
        R_z = matrix_from_angle(2,-angle_z)

        combined_rot1 = np.dot(R_x,R_euler_start)
        combined_rot2 = np.dot(R_y,combined_rot1)
        combined_rot = np.dot(R_z,combined_rot2)

        r1 = Rsc.from_matrix(combined_rot)
        calc_ortn_x = r1.as_euler('xyz', degrees=True)
        # print("The next angle = ", calc_ortn_x[0])


        # For the x ortn calc
        if counter > 1:
            intrinsic_ortn_x = calc_ortn_x[0]
            intrinsic_ortn_y = calc_ortn_x[1]
            intrinsic_ortn_z = calc_ortn_x[2]
            start_alpha, start_beta, start_gamma = math.radians(data_rcv[5]), math.radians(data_rcv[6]), math.radians(data_rcv[7])
        else:
            intrinsic_ortn_x = X_initial_ortn_rcvd
            intrinsic_ortn_y = Y_initial_ortn_rcvd
            intrinsic_ortn_z = Z_initial_ortn_rcvd


        # For the y ortn calc
        calc_state_y_old = calc_state_y
        calc_state_x_old = calc_state_x
        calc_state_z_old = calc_state_z

        print("THE X, Y, Z ORTN = ", intrinsic_ortn_x, intrinsic_ortn_y, intrinsic_ortn_z)

        if data_rcv[1] == 4.0:
            sample = [0.0, 4.0, startposx, startposy, startposz, initial_angle_ground[0], initial_angle_ground[1], initial_angle_ground[2]]
            print("Sent starting position")
            data_send = struct.pack('<8d', *sample)
            sock_send = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
            sock_send.sendto(data_send, (UDP_IP_SEND, UDP_PORT_SEND))
            # time.sleep(5)
        elif data_rcv[1] == 1.0:
            sample = [0.0, 4.0, startposx, startposy, startposz, initial_angle_ground[0], initial_angle_ground[1], initial_angle_ground[2]]
            print("Sent starting position")
            data_send = struct.pack('<8d', *sample)
            sock_send = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
            sock_send.sendto(data_send, (UDP_IP_SEND, UDP_PORT_SEND))
        else:
            print("Next iter on send")
        

        dircol = DirectCollocation(plant,
                                  context,
                                  num_time_samples=N,
                                  minimum_timestep=0.01,
                                  maximum_timestep=max_dt)

        #Constraint that all time steps have equal duration
        dircol.AddEqualTimeIntervalsConstraints()

        #Input u values to be 2x1 matrix
        u = dircol.input()

        #Add constraints to break points
        dircol.AddConstraintToAllKnotPoints(u[0] <=  umax)
        dircol.AddConstraintToAllKnotPoints(u[0] >= -umax)

        #The state
        x = dircol.state()


        starttime = timeit.default_timer()
        #print("The start time is :",starttime)

        #Box Constraints of Initial and Final States
        dircol.AddBoundingBoxConstraint(initial_state, initial_state, dircol.initial_state())
        dircol.AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())
        
        print("The time:", timeit.default_timer() - starttime)


        #Cost Function Minimize, here add the quadratic cost
        #Add quadratic input cost
        # #TODO
        R = 10
        dircol.AddRunningCost(R * u[0]**2)
        dircol.AddRunningCost(R * u[1]**2)
        dircol.AddRunningCost(R * u[2]**2)

        P = dircol.NewContinuousVariables(3, "P")

        dircol.AddQuadraticCost(
            Q=np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
            b=np.array([1., 1., 1.]),
            c=1.,
            vars=P[:3])

        #Initial Trajectory
        initial_x_trajectory = PiecewisePolynomial.FirstOrderHold([0., 20.], np.column_stack((initial_state, final_state)))  
        print(initial_x_trajectory)
        dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

        #Solve the DirCol
        result = Solve(dircol)
        print("\n\n")
        print("THE FUNCTION RETURN")
        print(result.is_success())
        print("\n\n")
        assert (result.is_success())

        #Final Trajectory
        print("\n\n")
        print("THE STATE TRAJ PRINT")
        u_trajectory = dircol.ReconstructInputTrajectory(result)
        x_trajectory = dircol.ReconstructStateTrajectory(result)


        #For the input u times 100
        times_input = np.linspace(u_trajectory.start_time(), u_trajectory.end_time(), 100)
        u_lookup = np.vectorize(u_trajectory.value)

        times_input = np.linspace(x_trajectory.start_time(), x_trajectory.end_time(), 100)
        x_lookup = np.vectorize(x_trajectory.value)

        #initial_state = np.array([x_lookup(1)[0], x_lookup(1)[1]])
        calc_state_x = x_lookup(1)[0]
        calc_state_y = x_lookup(1)[1]
        calc_state_z = x_lookup(1)[2]

        #Counter to check if the value has been calculated
        counter = counter + 1
        send_flag = 1

        initial_state = np.array((data_rcv[2], data_rcv[3], data_rcv[4]))
        print(data_rcv[1])

        #Converts and send out the data array
        #SEND OUT THE RUNNING DATA IF THE STATUS BIT IS 0
        if data_rcv[1] == 0.0:
            if calc_state_x - calc_state_x_old < 0.2:
                sample = [0.0, 0.0, x_lookup(1)[0], x_lookup(1)[1], x_lookup(1)[2], intrinsic_ortn_x , intrinsic_ortn_y, intrinsic_ortn_z]
            else:
                sample = [0.0, 0.0, startposx, startposy, startposz, intrinsic_ortn_x, intrinsic_ortn_y, intrinsic_ortn_z]

            data_send = struct.pack('<8d', *sample)
            sock_send = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
            sock_send.sendto(data_send, (UDP_IP_SEND, UDP_PORT_SEND))
            print("Sent the next iteration point")
        
        else:
            print("The status bit isnt 0")

        b = np.array((x_lookup(0)[0], x_lookup(0)[1], x_lookup(0)[2]))
        b = np.transpose(b)
        XTraj = np.concatenate((XTraj, b))
        C = np.append(C, result.get_optimal_cost())

        m = np.array((u_lookup(0)[0], u_lookup(0)[1], u_lookup(0)[2]))
        m = np.transpose(m)
        UTraj = np.concatenate((UTraj, m))

        i = i + 1
        print("Loop iteration",i)

        x_init = np.vectorize(initial_x_trajectory.value)

        # To check the z point for decreasing values
        z_old = x_lookup(1)[2]

    #LOOP ENDED
    #SEND THE STATUS -1 TO THE STOP EXECUTION
    sample = [0.0, -1.0, x_lookup(1)[0], x_lookup(1)[1], x_lookup(1)[2], intrinsic_ortn_x, intrinsic_ortn_y, intrinsic_ortn_z]
    print("Sent Ending Status ")
    data_send = struct.pack('<8d', *sample)
    sock_send = socket.socket(socket.AF_INET, # Internet
                socket.SOCK_DGRAM) # UDP
    sock_send.sendto(data_send, (UDP_IP_SEND, UDP_PORT_SEND))
    sock_rcv.close()
    sock_send.close()

    #PLOT THE ERROR GRAPH
    for newerroriter in range(i):
        t = np.reshape(x_init(newerroriter), (1,3))
        x_error = np.sqrt((t[0][0] - XTraj[newerroriter][0])**2 + (t[0][1] - XTraj[newerroriter][1])**2)
        err = np.append(err,[x_error])
    err = err[1:]
    linsp = np.linspace(0, i, (i-1))
    # plt.plot(linsp,err, color = 'green', linewidth=3.0)

    NewSavedXTraj = XTraj[1:]
    NewSavedUTraj = UTraj[1:]
    NewSavedCost = C

#PRINT THE COST AND THE TRAJ VALUES
print(NewSavedXTraj)
print(NewSavedCost)

# PLOTTING TRAJECTORY AND STREAMLINES
aa = 0.2
bb = 0.6
cc = 0.5 
zz1 = 0.3
zz2 = 1.0
w = 0.6
Y, X = np.mgrid[-cc:w:100j, aa:bb:100j]

U = 0.0094 - 0.0468*X + 0.0667*Y - 0.0493 - 0.0226*X*X + 0.0417*X*Y + 0.3267*X + 0.0692*Y*Y - 0.0087*Y - 0.0079
V = 0.0839 - 0.0471*X - 0.1227*Y - 0.1645 - 0.0552*X*X + 0.0122*X*Y + 0.0350*X - 0.2174*Y*Y + 0.3898*Y - 0.0405

fig = plt.figure(figsize=(20, 30))
gs = gridspec.GridSpec(nrows=3, ncols=2, height_ratios=[1, 1, 2])

# Varying density along a streamline
ax0 = fig.add_subplot(gs[0, 0])

ax0.streamplot(X, Y, U, V, density=[1, 1], color = 'orange')
ax0.set_title('Streamlines', color = 'white')

plt.plot(NewSavedXTraj[:,0],NewSavedXTraj[:,1], color = 'green', linewidth=3.0)
plt.show()

