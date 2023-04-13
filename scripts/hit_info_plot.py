import os
import matplotlib.pyplot as plt
import time
import numpy as np

sim = 1

if sim:
    origin_path  = '/home/mzy/Code/workSpace/UAV_Hitter_ws/src/aerial_hitter/data/sim/'
else:
    origin_path  = '/home/mzy/Code/workSpace/UAV_Hitter_ws/src/aerial_hitter/data/exp/'


uav_plan_x = []
uav_plan_y = []
uav_plan_z = []
uav_traj_x = []
uav_traj_y = []
uav_traj_z = []

arm_plan_0 = []
arm_plan_1 = []
arm_traj_0 = []
arm_traj_1 = []


if __name__ == "__main__":
    dirs = os.listdir(origin_path)
    for dir in dirs:
        files = os.listdir(origin_path+dir)
        uav_plan_x = []
        uav_plan_y = []
        uav_plan_z = []
        uav_traj_x = []
        uav_traj_y = []
        uav_traj_z = []

        arm_plan_0 = []
        arm_plan_1 = []
        arm_traj_0 = []
        arm_traj_1 = []

        for file in files:
            with open(os.path.join(origin_path+dir, file), 'r') as f:
                data_lines = f.read().split('\n')
                if file == "uav_plan_x.txt":
                    for data_line in data_lines:
                        data = data_line.split("  ")
                        try:
                            uav_plan_x.append(float(data[0]))
                        except:
                            pass
                elif file == "uav_plan_y.txt":
                    for data_line in data_lines:
                        data = data_line.split("  ")
                        try:
                            uav_plan_y.append(float(data[0]))
                        except:
                            pass
                elif file == "uav_plan_z.txt":
                    for data_line in data_lines:
                        data = data_line.split("  ")
                        try:
                            uav_plan_z.append(float(data[0]))
                        except:
                            pass
                elif file == "uav_traj.txt":
                    for data_line in data_lines:
                        data = data_line.split("  ")
                        try:
                            uav_traj_x.append(float(data[0]))
                            uav_traj_y.append(float(data[1]))
                            uav_traj_z.append(float(data[2]))
                        except:
                            pass
                elif file == "arm_plan_0.txt":
                    for data_line in data_lines:
                        data = data_line.split("  ")
                        try:
                            arm_plan_0.append(float(data[0]))
                        except:
                            pass
                elif file == "arm_plan_1.txt":
                    for data_line in data_lines:
                        data = data_line.split("  ")
                        try:
                            arm_plan_1.append(float(data[0]))
                        except:
                            pass
                elif file == "arm_traj.txt":
                    for data_line in data_lines:
                        data = data_line.split("  ")
                        try:
                            arm_traj_0.append(float(data[0]))
                            if sim:
                                arm_traj_1.append(float(data[0])+float(data[1])+2.6374)
                            else:
                                arm_traj_1.append(float(data[1]))
                        except:
                            pass
                else:
                    pass

        
        fig = plt.figure(0, figsize=(20, 10))
        fig.suptitle(dir)
        

        plt.subplot(2, 3, 1)
        plt.plot(range(0, len(uav_plan_x)), uav_plan_x, 'r', label="${uavPlan}_x$")
        plt.plot(range(0, len(uav_traj_x)), uav_traj_x, 'g', label="${uavTraj}_x$")
        plt.legend(ncol=2, loc="upper right")
        plt.grid()
        plt.xlabel("t")
        plt.ylabel("x")
        plt.title("UAM Position_x")

        plt.subplot(2, 3, 2)
        plt.plot(range(0, len(uav_plan_y)), uav_plan_y, 'r', label="${uavPlan}_y$")
        plt.plot(range(0, len(uav_traj_y)), uav_traj_y, 'g', label="${uavTraj}_y$")
        plt.legend(ncol=2, loc="upper right")
        plt.grid()
        plt.xlabel("t")
        plt.ylabel("y")
        plt.title("UAM Position_y")

        plt.subplot(2, 3, 3)
        plt.plot(range(0, len(uav_plan_z)), uav_plan_z, 'r', label="${uavPlan}_z$")
        plt.plot(range(0, len(uav_traj_z)), uav_traj_z, 'g', label="${uavTraj}_z$")
        plt.legend(ncol=2, loc="upper right")
        plt.grid()
        plt.xlabel("t")
        plt.ylabel("z")
        plt.title("UAM Position_z")

        plt.subplot(2, 3, 4)
        plt.plot(range(0, len(arm_plan_0)), arm_plan_0, 'r', label="${armPlan}_0$")
        plt.plot(range(0, len(arm_traj_0)), arm_traj_0, 'g', label="${armTraj}_0$")
        plt.legend(ncol=2, loc="upper right")
        plt.grid()
        plt.xlabel("t")
        plt.ylabel("arm_0")
        plt.title("Arm Position_0")

        plt.subplot(2, 3, 5)
        plt.plot(range(0, len(arm_plan_1)), arm_plan_1, 'r', label="${armPlan}_1$")
        plt.plot(range(0, len(arm_traj_1)), arm_traj_1, 'g', label="${armTraj}_1$")
        plt.legend(ncol=2, loc="upper right")
        plt.grid()
        plt.xlabel("t")
        plt.ylabel("arm_1")
        plt.title("Arm Position_1")

        plt.show()
        time.sleep(0)