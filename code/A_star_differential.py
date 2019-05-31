import argparse
import numpy as np
import os, sys
from numpy import linalg as LA
import math
from PIL import Image
from matplotlib import pyplot as plt
import random
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import cv2

def circle(x,y,map,x_center,y_center,radius):
    if (x-x_center)**2+(y-y_center)**2<=(radius+Robot_dia/2)**2:
        map[x,y,:]=[0,0,0]
    return map

def rectrangle(x,y,map,x_min,y_min,x_max,y_max):
    if x in range (int(x_min-Robot_dia/2),int(x_max+Robot_dia/2)+1) and y in range(int(y_min-Robot_dia/2),int(y_max+Robot_dia/2)+1):
        map[x,y,:]=[0,0,0]
    return map

def Map():
    map=255*np.array(np.ones((1110,1010,3)),dtype=np.uint8)
    for X in range (map.shape[0]):
        for Y in range(map.shape[1]):

            map=circle(X,Y,map,390,965,81/2) # 1
            map=circle(X,Y,map,438,736,81/2) # 2
            map=circle(X,Y,map,438,274,81/2) # 3
            map=circle(X,Y,map,390,45,81/2)  # 4
            map=circle(X,Y,map,149.95,830.05,159.9/2) # 5
            map=circle(X,Y,map,309.73,830.05,159.9/2) # 6
            map=rectrangle(X,Y,map,149.95,750.1,309.73,910) # 7
            map=rectrangle(X,Y,map,438,315,529,315+183) # 8
            map=rectrangle(X,Y,map,438+91,35+152+78,438+91+183,35+152+78+76) # 9
            map=rectrangle(X,Y,map,1110-636,35,1110-636+274,35+152) # 10
            map=rectrangle(X,Y,map,1110-425,0,1110,35)  # 11
            map=rectrangle(X,Y,map,1110-183,35,1110,35+76) # 12
            map=rectrangle(X,Y,map,1110-117-31-183,35,1110-31-183,35+58) # 13
            map=rectrangle(X,Y,map,1110-58,1010-313-76-55.5-117-86-67.25-117,1110,1010-313-76-55.5-117-86-67.25) # 14
            map=rectrangle(X,Y,map,438+91+183+72.5,35+152+80,438+91+183+72.5+152,35+152+80+117) # 15
            map=rectrangle(X,Y,map,1110-91,1010-313-76-55.5-117-86,1110,1010-313-76-55.5-117) # 16
            map=rectrangle(X,Y,map,1110-58,1010-313-76-55.5-117,1110,1010-313-76-55.5) # 17
            map=rectrangle(X,Y,map,1110-366,1010-313-76,1110,1010-313) # 18
            map=rectrangle(X,Y,map,1110-192-86,1010-183,1110-192,1010) # 19
            map=rectrangle(X,Y,map,1110-84-43,1010-91,1110-84,1010) # 20
            if X in range(int(Robot_dia/2)+1) or X in range(map.shape[0]-int(Robot_dia/2)-1,map.shape[0]):
                map[X,Y,:]=[0,0,0]
            if Y in range(int(Robot_dia/2)+1) or Y in range(map.shape[1]-int(Robot_dia/2)-1,map.shape[0]):
                map[X,Y,:]=[0,0,0]
    return map

def Huerastic_distance(x_src,y_src,x_dest,y_dest):
    return np.sqrt((x_src-x_dest)**2+(y_src-y_dest)**2)

def Step(x,y,theta,RPM_L,RPM_R):
    instants=50
    for i in range(instants):
        x=x+Wheel_Dia*math.pi*(RPM_L+RPM_R)*math.cos(theta)*time/(120*instants)
        y=y+Wheel_Dia*math.pi*(RPM_L+RPM_R)*math.sin(theta)*time/(120*instants)
        theta=theta+Wheel_Dia*math.pi*(RPM_R-RPM_L)*time/(60*Wheel_distance*instants)
    return x , y ,theta


def NodeCheck(Master_mat,X,Y):
    flag_n=True
    threshold=2 # less than buffer
    explore_limit=5000
    if len(Master_mat)<=explore_limit:
        for i in range(len(Master_mat)):
            if X<Master_mat[i,2]+threshold and X>Master_mat[i,2]-threshold and Y<Master_mat[i,3]+threshold and Y>Master_mat[i,3]-threshold:
                flag_n=False
    else :
        l=len(Master_mat)
        for i in range(1,explore_limit):
            if X<Master_mat[l-i,2]+threshold and X>Master_mat[l-i,2]-threshold and Y<Master_mat[l-i,3]+threshold and Y>Master_mat[l-i,3]-threshold:
                flag_n=False
    return flag_n

def boundry_check(x,y):
    flag=0
    X=int(x)
    Y=int(y)
    if int(x) in range(0,1110) and int(y) in range(0,1010):
        flag=1
    return flag

def Explore_choice(Master_mat,Parent_X,Parent_Y,Parent_Theta,Parent_Cost,Parent_index,RPM_L,RPM_R,weight,flag):
    x_new,y_new,theta_new=Step(Parent_X,Parent_Y,Parent_Theta,RPM_L,RPM_R)
    if(boundry_check(x_new,y_new)==1 and Map[int(x_new),int(y_new),0]==255 and flag==True and NodeCheck(Master_mat,x_new,y_new)==True):
        #cost=Parent_Cost+weight
        Map[int(x_new),int(y_new),:]=[0,0,200]
        cost=Parent_Cost+Huerastic_distance(Parent_X,Parent_Y,x_new,y_new)
        Distance=Huerastic_distance(x_new,y_new,Goal[0],Goal[1])
        stack=np.mat([len(Master_mat),Parent_index,x_new,y_new,theta_new,cost,cost+Distance,(RPM_L*2*22)/(7*60),(RPM_R*2*22)/(7*60)])
        Master_mat=np.vstack((Master_mat,stack))
    if int(x_new) in range(Goal[0]-buffer,Goal[0]+buffer) and int(y_new) in range(Goal[1]-buffer,Goal[1]+buffer):
        flag=False
    return Master_mat,flag

def pointexplore(index,Master_mat,Map,flag):
    Parent_index=Master_mat[index,0]
    Parent_X=Master_mat[index,2]
    Parent_Y=Master_mat[index,3]
    Parent_Theta=Master_mat[index,4]
    Parent_Cost=Master_mat[index,5]
    RPM_L,RPM_R,weight=RPM1,0,cost_of_step[0,0]
    Master_mat,flag=Explore_choice(Master_mat,Parent_X,Parent_Y,Parent_Theta,Parent_Cost,Parent_index,RPM_L,RPM_R,weight,flag)
    RPM_L,RPM_R,weight=RPM2,0,cost_of_step[0,1]
    Master_mat,flag=Explore_choice(Master_mat,Parent_X,Parent_Y,Parent_Theta,Parent_Cost,Parent_index,RPM_L,RPM_R,weight,flag)
    RPM_L,RPM_R,weight=RPM2,RPM1,cost_of_step[0,2]
    Master_mat,flag=Explore_choice(Master_mat,Parent_X,Parent_Y,Parent_Theta,Parent_Cost,Parent_index,RPM_L,RPM_R,weight,flag)
    RPM_L,RPM_R,weight=RPM1,RPM1,cost_of_step[0,3]
    Master_mat,flag=Explore_choice(Master_mat,Parent_X,Parent_Y,Parent_Theta,Parent_Cost,Parent_index,RPM_L,RPM_R,weight,flag)
    RPM_L,RPM_R,weight=RPM2,RPM2,cost_of_step[0,4]
    Master_mat,flag=Explore_choice(Master_mat,Parent_X,Parent_Y,Parent_Theta,Parent_Cost,Parent_index,RPM_L,RPM_R,weight,flag)
    RPM_L,RPM_R,weight=RPM1,RPM2,cost_of_step[0,5]
    Master_mat,flag=Explore_choice(Master_mat,Parent_X,Parent_Y,Parent_Theta,Parent_Cost,Parent_index,RPM_L,RPM_R,weight,flag)
    RPM_L,RPM_R,weight=0,RPM1,cost_of_step[0,6]
    Master_mat,flag=Explore_choice(Master_mat,Parent_X,Parent_Y,Parent_Theta,Parent_Cost,Parent_index,RPM_L,RPM_R,weight,flag)
    RPM_L,RPM_R,weight=0,RPM2,cost_of_step[0,7]
    Master_mat,flag=Explore_choice(Master_mat,Parent_X,Parent_Y,Parent_Theta,Parent_Cost,Parent_index,RPM_L,RPM_R,weight,flag)
    Master_mat[index,6]=math.inf

    return Master_mat,flag

def backtrack(Master_mat):
    back_mat=[Master_mat[-1,0]]
    Found=1
    while(Found!=0):
        pointer=int(back_mat[-1])
        Found=Master_mat[pointer,1]
        print(Found)
        back_mat=np.append([back_mat],[Found])
    return back_mat

# function returns matrix with final path
def backtrackmat(back_mat,Node,Map):
    backtrackmat=np.array(np.zeros((len(back_mat),3)))
    Velocity_matrix=np.array(np.zeros((len(back_mat),2)))
    for a in range(len(back_mat)):
        i=len(back_mat)-a-1
        value=int(back_mat[a])
        backtrackmat[i,0]=Node[value,2]
        backtrackmat[i,1]=Node[value,3]
        backtrackmat[i,2]=Node[value,4]
        Velocity_matrix[i,0]=Node[value,7]
        Velocity_matrix[i,1]=Node[value,8]

    return backtrackmat,Velocity_matrix,Map

def Step_plot(Path,Velocity_matrix,Map):
    instants=50
    for i in range(1,len(Path)):
        theta=Path[i-1,2]
        x=Path[i-1,0]
        y=Path[i-1,1]
        RPM_L=Velocity_matrix[i,0]*60*7/(2*22)
        RPM_R=Velocity_matrix[i,1]*60*7/(2*22)
        for j in range(instants):
            x=x+Wheel_Dia*math.pi*(RPM_L+RPM_R)*math.cos(theta)*time/(120*instants)
            y=y+Wheel_Dia*math.pi*(RPM_L+RPM_R)*math.sin(theta)*time/(120*instants)
            theta=theta+Wheel_Dia*math.pi*(RPM_R-RPM_L)*time/(60*Wheel_distance*instants)
            Map[int(x),int(y),:]=[0,200,0]

        Map_path=cv2.rotate(Map.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
        scale_p = cv2.resize(Map_path,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
        cv2.imshow('map_p',scale_p)
        cv2.waitKey(10)
    return Map

######----------------------------------------------------------
# Initialize
RPM1= 15 # in RPM
RPM2= 20 # in RPM
#Goal=[100,500] # in cm
#Start=[100,100] # in cm
Wheel_Dia=7.6 # in cm
Wheel_distance=23 # in cm
Robot_dia=35.4 # in cm
Allowable_clerance=2 # in cm
time=2 # in sec
theta=0 # in degrees
sampling_frequency=1 # samples in a sec.
buffer=3 # in cm
####-----------------------------------------------------------
print('Map is getting generated')
Map=Map()
error=0
print("Input co-ordinates has origin at bottom left" )
print("Input co-ordinates of x between 20 & 1090")
print("Input co-ordinates of y between 20 & 990")
Startx=float(input('Value of Start x \n'))
Starty=float(input('Value of Start y \n'))
Endx=float(input('Value of Goal x \n'))
Endy=float(input('Value of Goal y \n'))
Startx=int(Startx)
Starty=int(Starty)
Endx=int(Endx)
Endy=int(Endy)
# Error conditions
if Startx not in range(1110) and Starty not in range(1010):
    print('Start point out of bound')
    error=1

if Endx not in range(1110) or Endy not in range(1010):
    print('End point out of bound')
    error=1

if (error==0):
    if (Map[Startx,Starty,0]==0):
        print('Start point is in obstacle')
        error=1
    if (Map[Endx,Endy,0]==0):
        print('End point is in obstacle')
        error=1

Start=[Startx,Starty]
Goal=[Endx,Endy]

#######------------------------------------------------------------------
cost_of_step=np.mat([1,1,1,1,1,1,1,1,1])
Master_mat=np.mat([0,0,Start[0],Start[1],0,0,0,0,0]).astype(float) # can include initial angle
#####----------------------------------------------------------
if (error==0):
    index_p=0
    flag=True
    while(flag and len(Master_mat)<100000):
        Master_mat,flag=pointexplore(index_p,Master_mat,Map,flag)
        index_p=np.argmin(Master_mat[:,6])
        print(Master_mat[index_p,6])
        print(index_p)
        Map_explore=cv2.rotate(Map.copy(), cv2.ROTATE_90_COUNTERCLOCKWISE)
        scale_p = cv2.resize(Map_explore,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
        cv2.imshow('map_p',scale_p)
        cv2.waitKey(10)
    print(Master_mat[-1,:])
    backtrack_mat=backtrack(Master_mat)
    #print(backtrack_mat,'backtrack')
    Path,Velocity_matrix,Map=backtrackmat(backtrack_mat,Master_mat,Map)
    print(Path)
    print(Velocity_matrix)

    Velocity_matrix_modified=np.array([0,0])
    Path_modified=np.array([0,0])
    for i in range(len(Velocity_matrix)):
        for j in range(15):
            stack_v=np.array([Velocity_matrix[i,0],Velocity_matrix[i,1]])
            Velocity_matrix_modified=np.vstack((Velocity_matrix_modified,stack_v))
            stack_p=np.array([Path[i,0],Path[i,1]])
            Path_modified=np.vstack((Path_modified,stack_p))

    print(np.shape(Velocity_matrix))
    print(np.shape(Velocity_matrix_modified))
    print(np.shape(Path_modified))
    #plt.plot(Path[:,0],Path[:,1],'-r')
    #plt.plot(Path[:,0],Path[:,1],'*g')
    #plt.plot(Master_mat[:,2],Master_mat[:,3],'*b')
    #plt.show()
    Map=Step_plot(Path,Velocity_matrix,Map)
Map=cv2.rotate(Map, cv2.ROTATE_90_COUNTERCLOCKWISE)
res = cv2.resize(Map,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
#print(np.shape(res))

cv2.imshow('map_1',res)
cv2.waitKey(3000)
