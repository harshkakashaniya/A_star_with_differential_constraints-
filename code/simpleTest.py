# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
from A_star_differential import Velocity_matrix_modified
from A_star_differential import Path_modified

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    if res==vrep.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

#retrieve motor  handles
    errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_oneshot_wait)
    errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_oneshot_wait)
    steps=Path_modified
    #steps =  [[100, 100, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]]
    #steps = [[-4, -4, 0], [-3.905, -4.0, 0.82], [-3.776, -3.862, 0.82], [-3.647, -3.724, 0.82], [-3.518, -3.586, 0.82], [-3.389, -3.448, 0.82], [-3.26, -3.31, 0.82], [-3.131, -3.172, 0.82], [-3.002, -3.034, 0.82], [-2.873, -2.896, 0.82], [-2.744, -2.758, 0.82], [-2.615, -2.62, 0.82], [-2.486, -2.482, 0.82], [-2.357, -2.344, 0.82], [-2.228, -2.206, 0.82], [-2.099, -2.068, 2.47]]
    #movement = [[10,0],[20,0],[50,0],[100,100],[50,0], [20,0],[10,0]]
    movement=Velocity_matrix_modified
    count=0
    print(len(movement))
    for i in movement:
        errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,i[0], vrep.simx_opmode_streaming)
        errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,i[1], vrep.simx_opmode_blocking)
        if count==len(movement):
            time.sleep(0)
        else :
            print(steps[count])
            count=count+1
            print(count)
            simTime = vrep.simxGetLastCmdTime(clientID)
            print(simTime/1000)
            time.sleep(0.25)

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')

print ('Program ended')
