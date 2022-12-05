#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

import matplotlib.pyplot as plt


plot_size = 6000

x = [i for i in range(plot_size)]
y = [0 for i in range(plot_size)]
y_ref_list = [0 for i in range(plot_size)]


i = 0
model = True
t = 0


def plot(x, y, y_ref):
    plt.clf()
    plt.ylim(-0.1, 0.2)
    plt.plot(x, y)
    plt.plot(x, y_ref)
    plt.draw()
    plt.pause(0.001)

def arm0PoseCallBack(data):
    global i, t, model
    arm_pose = data.data

    y.pop(int(0))
    y.append(arm_pose)

    t += 1
    if t > 2000:
        model = ~model
        t = 0
    
    if model == 1:
        y_ref = 0.1
    else:
        y_ref = 0
    y_ref_list.pop(int(0))
    y_ref_list.append(y_ref)

    i += 1
    if(i > 100):
        plot(x, y, y_ref_list)
        i = 0
        # print(data.data)

    pub_0.publish(Float32(y_ref))
    pub_1.publish(Float32(0))



if __name__ == "__main__":
    rospy.init_node("arm_pid_plot", anonymous=True)
    rospy.Subscriber("/hummingbird_arm/aerial_arm/arm_0_joint/pose", Float32, arm0PoseCallBack)
    
    pub_0 = rospy.Publisher("/hummingbird_arm/aerial_arm/arm_0_joint/pos_cmd", Float32, queue_size=10)
    pub_1 = rospy.Publisher("/hummingbird_arm/aerial_arm/arm_1_joint/pos_cmd", Float32, queue_size=10)   

    rospy.spin()
    