from freenect2 import *
import threading
import numpy as np
import cv2
import time
import os
import open3d as o3d
from PIL import Image
import tkinter as tk

# 是否开启数据流
startflag = 0
stopflag = 0
exitflag = 0
firstRGB = 0

# 全局的数据流
frames = {"newframe": 0}


class ReadFrames(threading.Thread):
    def __init__(self, subdevice):
        threading.Thread.__init__(self)
        self.device = subdevice

    def run(self):
        global stopflag, firstRGB, frames

        # 检测是否开始开始数据流
        while True:
            if startflag == 1:
                self.device.start()

                # 循环接受数据流
                while True:
                    frame_type, frame = self.device.get_next_frame()
                    frames[frame_type] = frame

                    if firstRGB == 0:
                        if FrameType.Color in frames:
                            firstRGB = 1

                    if firstRGB == 1 and frame_type == FrameType.Depth:
                        frames["newframe"] += 1
                        # print(frames)
                        # print("New Frames coming")

                    # 接收到数据后关闭数据流
                    if stopflag == 1:
                        self.device.stop()
                        print("device stopped")
                        break


class WindowUI(threading.Thread):
    def __init__(self, ):
        threading.Thread.__init__(self)

    def run(self):
        root = tk.Tk()
        root.title("Kinect studio")

        Start_ReadFrames_UI = tk.Button(root, text="Start Read Frames", bg="yellow", command=Start_ReadFrames)
        Start_ReadFrames_UI.pack()

        Stop_ReadFrames_UI = tk.Button(root, text="Stop", bg="yellow", command=Stop_ReadFrames)
        Stop_ReadFrames_UI.pack()

        Exit_UI = tk.Button(root, text="Exit", bg="yellow", command=Exit)
        Exit_UI.pack()

        root.mainloop()


def Start_ReadFrames():
    global startflag, stopflag
    print("Start read the frames")
    stopflag = 0
    startflag = 1


def Stop_ReadFrames():
    global stopflag, startflag, firstRGB
    print("Stop read the frames")
    startflag = 0
    firstRGB = 0
    stopflag = 1


def Exit():
    global exitflag, stopflag, startflag
    print("Exit system")
    exitflag = 1
    startflag = 0
    stopflag = 1


class ShowFrames(threading.Thread):
    def __init__(self, subdevice):
        threading.Thread.__init__(self)
        self.device = subdevice

    def run(self):
        global frames
        befoe_sq = 0

        # 具有自定义按键Callback功能的可视化工具。
        geometrie_added = False
        vis = o3d.visualization.Visualizer()

        # 创建显示窗口
        vis.create_window("Pointcloud", 424, 512)
        # 定义点云类
        pointcloud = o3d.geometry.PointCloud()

        while True:
            if befoe_sq != frames["newframe"]:
                befoe_sq = frames["newframe"]

                rgb, depth = frames[FrameType.Color], frames[FrameType.Depth]
                undistorted, registered, big_depth = self.device.registration.apply(rgb, depth, enable_filter=True, with_big_depth=True)

                # depth_frame = undistorted.to_array()
                # depth_frame = depth_frame.astype(np.uint8)
                # dep_frame = np.reshape(depth_frame, [424, 512])
                # dep_frame = cv2.cvtColor(dep_frame, cv2.COLOR_GRAY2RGB)
                #
                # rgb_frame = registered.to_array()
                # rgb_frame = np.reshape(rgb_frame, [424, 512, 4])[:, :, 0:3]
                # print("depth", dep_frame.shape, "RGB", rgb_frame.shape)
                #
                # cv2.imshow('dep_frame', dep_frame)
                # cv2.imshow('rgb_frame', rgb_frame)

                pointcloud.clear()

                # depth_image = np.where((depth_image > 2000) | (depth_image < 0), 0, depth_image)

                # with open('output.pcd', 'wb') as FF:
                #     device.registration.write_big_pcd(FF, big_depth, rgb)

                with open('output.pcd', 'wb') as FF:
                    device.registration.write_pcd(FF, undistorted, registered)

                pcd = o3d.io.read_point_cloud("/home/sbdx/PycharmProjects/pythonProject/output.pcd")

                pcd = pcd.voxel_down_sample(voxel_size=0.0065)
                pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=25, std_ratio=1.0)

                pointcloud += pcd  # 添加点云

                if not geometrie_added:
                    # 第一次将几何体添加到场景并创建相应的着色器的功能，之后只需要更行的就好
                    vis.add_geometry(pointcloud)
                    geometrie_added = True

                # 当用于更新几何的功能。更改几何时必须调用此函数。
                vis.update_geometry(pointcloud)
                # 通知渲染需要更新的功能
                vis.update_renderer()
            else:
                cv2.waitKey(1)


Win_thread = WindowUI()
Win_thread.start()

device = Device()
RF_thread = ReadFrames(device)
RF_thread.start()

SF_thread = ShowFrames(device)
SF_thread.start()
