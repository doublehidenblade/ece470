import sim # access all the VREP elements
import modern_robotics as mr
import numpy as np
from scipy.linalg import expm
from scipy.linalg import logm
import numpy as np
def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])
def unskew(x):
    return np.array([x[2][1], x[0][2], x[1][0]])
def ik(T, thetalist0):
    M = np.array([[ 0.   ,   -1.   ,    0.  ,     0.3876 ],
 [ 0.     , 0.   ,   -1.  ,     0.40845],
 [ 1.     ,  0.  ,     0. ,      0.20915],
 [ 0.  ,     0.    ,   0. ,      1.     ]])
    S = np.array([[ 0.   ,   0.   ,   0.   ,   0.   ,   1.  ,    0.    ],
 [ 0.,      1.  ,    1.  ,    1.   ,   0.  ,    1.    ],
 [ 1.    ,  0.  ,    0.   ,   0.   ,   0.  ,    0.    ],
 [ 0.1524, -0.152  ,-0.152  ,-0.152 , -0.  ,   -0.152 ],
 [ 0.1524 , 0. ,    -0.   ,  -0.    ,  0.152 , -0.    ],
 [-0.  ,   -0.1524 , 0.0916  ,0.3046, -0.2624 , 0.3876]])
    eomg = 0.01
    ev = 0.001
    ans = mr.IKinSpace(S, M, T, thetalist0, eomg, ev)
    print(ans)
    if (ans[1] == False): 
        return False;
    return ans[0]

sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection

if clientID!=-1:
    print ("Connected to remote API server")
    #IV
    R = np.array([
    [-0.0309, -0.8246, -0.5648],
    [0.99454, 0.0309, -0.996],
    [0.0996, -0.5648, 0.81915]]) ## "zero's x is one's (+-) y"
    p = np.array([[0.4],[0.412],[0.1655]])
    T = np.array([
    [R[0][0], R[0][1], R[0][2], p[0]],
    [R[1][0], R[1][1], R[1][2], p[1]],
    [R[2][0], R[2][1], R[2][2], p[2]],
    [0,0,0,1]])
    thetalist0 = np.array([10/180*np.pi,-25/180*np.pi,35/180*np.pi,-45/180*np.pi,-90/180*np.pi,10/180*np.pi,])
    theta1,theta2,theta3,theta4,theta5,theta6 = ik(T, thetalist0)
    print(theta1*180/np.pi,theta2*180/np.pi,theta3*180/np.pi,theta4*180/np.pi,theta5*180/np.pi,theta6*180/np.pi)
    err_code,joint1 = sim.simxGetObjectHandle(clientID,"UR3_joint1", sim.simx_opmode_blocking)
    err_code,joint2 = sim.simxGetObjectHandle(clientID,"UR3_joint2", sim.simx_opmode_blocking)
    err_code,joint3 = sim.simxGetObjectHandle(clientID,"UR3_joint3", sim.simx_opmode_blocking)
    err_code,joint4 = sim.simxGetObjectHandle(clientID,"UR3_joint4", sim.simx_opmode_blocking)
    err_code,joint5 = sim.simxGetObjectHandle(clientID,"UR3_joint5", sim.simx_opmode_blocking)
    err_code,joint6 = sim.simxGetObjectHandle(clientID,"UR3_joint6", sim.simx_opmode_blocking)
    #move 
    err_code = sim.simxSetJointTargetPosition(clientID, joint1 , theta1-np.pi/2, sim.simx_opmode_oneshot)
    err_code = sim.simxSetJointTargetPosition(clientID, joint2 , theta2+np.pi/2, sim.simx_opmode_oneshot)
    err_code = sim.simxSetJointTargetPosition(clientID, joint3 , theta3, sim.simx_opmode_oneshot)
    err_code = sim.simxSetJointTargetPosition(clientID, joint4 , theta4, sim.simx_opmode_oneshot)
    err_code = sim.simxSetJointTargetPosition(clientID, joint5 , theta5, sim.simx_opmode_oneshot)
    err_code = sim.simxSetJointTargetPosition(clientID, joint6 , theta6, sim.simx_opmode_oneshot)
else:
    print("Not connected to remote API server")
