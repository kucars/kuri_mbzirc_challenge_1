import rosbag
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as mpatches


header = []
dronePoseXX = [] 
dronePoseYY = []
dronePoseZZ = []
goalPoseXX =[]
goalPoseYY =[]
goalPoseZZ=[]
droneVelXX =[]
droneVelYY =[]
droneVelZZ =[]
positionErrorXX=[]
positionErrorYY=[]
positionErrorZZ=[]
positionErrorWW=[]
PXX=[]
PYY=[]
PZZ=[]
PWW=[]
IXX=[]
IYY=[]
IZZ=[]
IWW= []
DXX=[]
DYY=[]
DZZ=[]
DWW=[]
PDWW=[]
PIDXX=[]
PIDYY=[]
PIDZZ=[]
PIDWW=[]
time=[]
bag = rosbag.Bag('test008.bag')
for topic, msg, t in bag.read_messages(topics=['/pidData']):
    header.append(msg.header.stamp.to_sec()) 
    # time.append(msg.timeR) 
    dronePoseXX.append(msg.dronePoseX) 
    dronePoseYY.append(msg.dronePoseY)
    dronePoseZZ.append(msg.dronePoseZ)
    goalPoseXX.append(msg.goalPoseX)
    goalPoseYY.append(msg.goalPoseY)
    goalPoseZZ.append(msg.goalPoseZ)
    positionErrorXX.append(msg.positionErrorX)
    positionErrorYY.append(msg.positionErrorY)
    positionErrorZZ.append(msg.positionErrorZ)
    positionErrorWW.append(msg.positionErrorW)
    PXX.append(msg.PX)
    PYY.append(msg.PY)
    PZZ.append(msg.PZ)
    PWW.append(msg.PW)
    IXX.append(msg.IX)
    IYY.append(msg.IY)
    IZZ.append(msg.IZ)
    IWW.append(msg.IW)
    DXX.append(msg.DX)
    DYY.append(msg.DY)
    DZZ.append(msg.DZ)
    DWW.append(msg.DW)
    PIDXX.append(msg.PIDX)
    PIDYY.append(msg.PIDY)
    PIDZZ.append(msg.PIDZ)
    PIDWW.append(msg.PIDW)
bag.close()


timeSet = [] 
vx =[] 
vy =[] 
vz = [] 
bag = rosbag.Bag('test006.bag')
for topic, msg, t in bag.read_messages(topics=['/mavros/setpoint_velocity/cmd_vel']):
    timeSet.append(msg.header.seq)
    vx.append(msg.twist.linear.x)
    vy.append(msg.twist.linear.y)
    vz.append(msg.twist.linear.z)
bag.close()

rtimeSet = [] 
rvx =[] 
rvy =[] 
rvz = [] 
bag = rosbag.Bag('test006.bag')
for topic, msg, t in bag.read_messages(topics=['/mavros/local_position/velocity']):
    rtimeSet.append(msg.header.seq)
    rvx.append(msg.twist.linear.x)
    rvy.append(msg.twist.linear.y)
    rvz.append(msg.twist.linear.z)    
bag.close()


#plots 
#velocity
plt.figure(1)
plt.plot(timeSet , vx ,'k')
plt.xlabel('Sequence Number',fontsize=14, color='red')
plt.ylabel('Set x-velocity',fontsize=14, color='red')
plt.grid(True)
plt.savefig("setXvel.png")


plt.figure(2)
plt.plot(rtimeSet , rvx ,'k')
plt.xlabel('Sequence Number',fontsize=14, color='red')
plt.ylabel('Actual x-Velocity',fontsize=14, color='red')
plt.grid(True)
plt.savefig("ActualXvel.png")


plt.figure(3)
plt.plot(timeSet , vy ,'k')
plt.xlabel('Sequence Number',fontsize=14, color='red')
plt.ylabel('Set y-velocity',fontsize=14, color='red')
plt.grid(True)
plt.savefig("setYvel.png")

plt.figure(4)
plt.plot(rtimeSet , rvy ,'k')
plt.xlabel('Sequence Number',fontsize=14, color='red')
plt.ylabel('Actual y-Velocity',fontsize=14, color='red')
plt.grid(True)
plt.savefig("ActualYvel.png")


plt.figure(5)
plt.plot(timeSet , vz ,'k')
plt.xlabel('Sequence Number',fontsize=14, color='red')
plt.ylabel('Set z-velocity',fontsize=14, color='red')
plt.grid(True)
plt.savefig("setZvel.png")

plt.figure(6)
plt.plot(rtimeSet , rvz ,'k')
plt.xlabel('Sequence Number',fontsize=14, color='red')
plt.ylabel('Actual z-Velocity',fontsize=14, color='red')
plt.grid(True)
plt.savefig("ActualZvel.png")


#PID components
plt.figure(7)
plt.plot(header, PXX , 'r' )
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('P controller ',fontsize=14, color='red')
plt.grid(True)
plt.savefig("pxController.png")


plt.figure(8)
plt.plot(header, PYY , 'r' )
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('PY controller ',fontsize=14, color='red')
plt.grid(True)
plt.savefig("pyController.png")

plt.figure(9)
plt.plot(header , PZZ ,'k')
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('PZ controller ',fontsize=14, color='red')
plt.grid(True)
plt.savefig("pzController.png")


plt.figure(10)
plt.plot(header, IXX , 'r' )
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('IX controller ',fontsize=14, color='red')
plt.grid(True)
plt.savefig("ixController.png")


plt.figure(11)
plt.plot(header, IYY , 'r' )
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('IY controller ',fontsize=14, color='red')
plt.grid(True)
plt.savefig("iyController.png")

plt.figure(12)
plt.plot(header , IZZ ,'k')
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('IZ controller ',fontsize=14, color='red')
plt.grid(True)
plt.savefig("izController.png")



plt.figure(13)
plt.plot(header, DXX , 'r' )
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('DX controller ',fontsize=14, color='red')
plt.grid(True)
plt.savefig("dxController.png")


plt.figure(14)
plt.plot(header, DYY , 'r' )
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('DY controller ',fontsize=14, color='red')
plt.grid(True)
plt.savefig("dyController.png")

plt.figure(15)
plt.plot(header , DZZ ,'k')
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('dZ controller ',fontsize=14, color='red')
plt.grid(True)
plt.savefig("dzController.png")
'''
plt.figure()
plt.plot(header, PIDXX , 'r' , header , PIDYY , 'b' , header , PIDZZ ,'k')
plt.xlabel('time (s)')
plt.ylabel('PID ',fontsize=14, color='red')
plt.title('Controller Output PID',fontsize=14, color='red')
plt.grid(True)
plt.savefig("pidController002.png")
'''
#position error 
plt.figure(16)
plt.plot(header, dronePoseXX , 'r' , label='Drone x-position')
plt.plot( header , goalPoseXX ,'k', label='Goal x-position')
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('x-position (m)',fontsize=14, color='red')
plt.grid(True)
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
plt.savefig("x_position.png")


plt.figure(17)
plt.plot(header, dronePoseYY , 'r' , label='Drone y-position')
plt.plot(header , goalPoseYY,'k',label='Goal y-position' )
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('y-position (m)',fontsize=14, color='red')
plt.grid(True)
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
plt.savefig("y_position.png")


plt.figure(18)
plt.plot(header, dronePoseZZ , 'r' , label='Drone z-position')
plt.plot(header , goalPoseZZ ,'k',label='Goal z-position' )
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('z-position (m)',fontsize=14, color='red')
plt.grid(True)
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
plt.savefig("z_position.png")


plt.figure(19)
plt.plot(header, positionErrorXX)
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('Pose Error X',fontsize=14, color='red')
plt.grid(True)
plt.savefig("PoseErrortX.png")

plt.figure(20)
plt.plot( header , positionErrorYY )
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('Pose ErrorY',fontsize=14, color='red')
plt.grid(True)
plt.savefig("PoseErrorY.png")


plt.figure(21)
plt.plot(header , positionErrorZZ ,'k')
plt.xlabel('time (s)',fontsize=14, color='red')
plt.ylabel('Pose Error Z',fontsize=14, color='red')
plt.grid(True)
plt.savefig("PoseErrorZ.png")


plt.show()




















