#!/usr/bin/env python3
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
import math


my_chain = ikpy.chain.Chain.from_urdf_file("me380_robot.urdf", active_links_mask=[False, True, True, True, True, True, True, False])

target_position = [ 0.2, .2, 0.2]
target_orientation = [-1, 0, 0]

ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="X")
print("The angles of each joints are : ", list(map(lambda r:math.degrees(r),ik.tolist())))

print("The angles of each joints are : ", my_chain.inverse_kinematics(target_position))

# ax = plt.figure().add_subplot(111, projection='3d')
fig, ax = plot_utils.init_3d_figure()
fig.set_figheight(9)  
fig.set_figwidth(13)  
my_chain.plot(ik, ax, target=target_position)

plt.xlim(-0.4, 0.4)
plt.ylim(-0.4, 0.4)
ax.set_zlim(0, 0.4)
# Equal axis scale so the robot is not stretched (matplotlib >= 3.3)

plt.show()

def doIK():
    global ik
    old_position= ik.copy()
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Z", initial_position=old_position)

def updatePlot():
    ax.clear()
    my_chain.plot(ik, ax, target=target_position)
    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.6)
    fig.canvas.draw()
    fig.canvas.flush_events()
    
def move(x,y,z):
    global target_position
    target_position = [x,y,z]
    doIK()
    updatePlot()


    # sendCommand(ik[1].item(),ik[2].item(),ik[3].item(),ik[4].item(),ik[5].item(),ik[6].item(),1)