import sim # access all the VREP elements
import modern_robotics as mr
import numpy as np
from scipy.linalg import expm
from scipy.linalg import logm
import numpy as np
# get skew matrix
def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])
    
# unskew the matrix
def unskew(x):
    return np.array([x[2][1], x[0][2], x[1][0]])

# inverse kinematic, it takes the homogenious matrix and approximate where it 
# would be, it uses M and S matrices to iterate over possible angles and when 
# possible anlgles is closed to the answer then return the joint angles
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

# it get the start point of the ball and predict the ball's location to let the 
# UR3 arm stop the ball. point 1 is the location where the camera start to sense
# the ball, and point 2 is the location after some movements then we use the point1 
# and point2 to predict the location of point3
    
def stop():
    i =0
    prev = 0
    interceptx = 0.4
    interceptz = 0.1
    point1 = None
    point2 = None
    detectx = 0.6
    while(True):
        [err,signal]=sim.simxGetStringSignal(clientID,'measuredData',sim.simx_opmode_blocking)
        coord = sim.simxUnpackFloats(signal)
        
        if len(coord)!= 0:
            if point1 is None:
                point1 = coord
            if point2 is None and coord[0] < detectx:
                point2 = coord
        if point1 is not None and point2 is not None:
            break
        i = i+1
    print('start:', point1)
    print('end:', point2)
    ratio = (point2[1]-point1[1])/(point2[0]-point1[0]+0.000000001)
    intercepty = point1[1] + ratio*(interceptx-point1[0])
    point3 = [interceptx,intercepty,interceptz]
    hit = [interceptx+0.1,intercepty,interceptz]
    robot.moveTo(point3)
    robot.moveTo(hit)
    robot.moveTo(point3)
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection

# when it is initialized, it tells the robot to go to the ready position
class Robot:
    def __init__(self):
        self.thetalist = np.array([10/180*np.pi,-25/180*np.pi,35/180*np.pi,-45/180*np.pi,-90/180*np.pi,10/180*np.pi])
        self.R = np.array([
    [-0.0309, -0.8246, -0.5648],
    [0.99454, 0.0309, -0.996],
    [0.0996, -0.5648, 0.81915]])
    # go to the ready position
        self.stepTo([0.4,0.412,0.1655])
        self.last_pos = np.array([0.4,0.412,0.1655])
    # it takes the desired location and call stepto repeatly untill it reaches the 
    # desired location        
    def moveTo(self, temp):
        endp = np.array([temp[0]-0.1,temp[1]+0.05,temp[2]-0.15])
        d = np.sqrt((self.last_pos[0]-endp[0])**2+(self.last_pos[1]-endp[1])**2+(self.last_pos[2]-endp[2])**2)
        unitv = np.array((endp-self.last_pos)/d)
        d_traveled = 0
        cur = self.last_pos
        while(d_traveled<d):
            cur = cur+0.1*unitv
            d_traveled = d_traveled + 0.1
            # we set the operating mode to be oneshot so that the UR3 can move smoothly
            self.stepTo(cur,sim.simx_opmode_oneshot)
        self.stepTo(endp)
        self.last_pos = endp
    # it takes a very close point from the last joint configuration to keep the 
    # distance small enought for IK to calculate and record the new calculated position
    def stepTo(self, p, mode = sim.simx_opmode_blocking):
        # ik things
        T = np.array([
    [self.R[0][0], self.R[0][1], self.R[0][2], p[0]],
    [self.R[1][0], self.R[1][1], self.R[1][2], p[1]],
    [self.R[2][0], self.R[2][1], self.R[2][2], p[2]],
    [0,0,0,1]])
        newthetas = ik(T, self.thetalist)
        self.thetalist = newthetas
        #move 
        err_code = sim.simxSetJointTargetPosition(clientID, joint1 , newthetas[0]-np.pi/2, mode)
        err_code = sim.simxSetJointTargetPosition(clientID, joint2 , newthetas[1]+np.pi/2, mode)
        err_code = sim.simxSetJointTargetPosition(clientID, joint3 , newthetas[2], mode)
        err_code = sim.simxSetJointTargetPosition(clientID, joint4 , newthetas[3], mode)
        err_code = sim.simxSetJointTargetPosition(clientID, joint5 , newthetas[4], mode)
        err_code = sim.simxSetJointTargetPosition(clientID, joint6 , newthetas[5], mode)
# main function
if clientID!=-1:
    err_code,joint1 = sim.simxGetObjectHandle(clientID,"UR3_joint1", sim.simx_opmode_blocking)
    err_code,joint2 = sim.simxGetObjectHandle(clientID,"UR3_joint2", sim.simx_opmode_blocking)
    err_code,joint3 = sim.simxGetObjectHandle(clientID,"UR3_joint3", sim.simx_opmode_blocking)
    err_code,joint4 = sim.simxGetObjectHandle(clientID,"UR3_joint4", sim.simx_opmode_blocking)
    err_code,joint5 = sim.simxGetObjectHandle(clientID,"UR3_joint5", sim.simx_opmode_blocking)
    err_code,joint6 = sim.simxGetObjectHandle(clientID,"UR3_joint6", sim.simx_opmode_blocking)
    print ("Connected to remote API server")
    # initialize the robot
    robot = Robot()
    #stop three balls 
    stop()
    stop()
    stop()

else:
    print("Not connected to remote API server")


