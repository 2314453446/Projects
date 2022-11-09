# coding: utf-8

# An example using startStreams

import numpy as np
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
import argparse
import datetime
import keyboard
import h5py

parser = argparse.ArgumentParser()
parser.add_argument('--out_name', type=str, help="the output names", default=None)
parser.add_argument('--out_dir', type=str, help="the output directory", default='data/')
args = parser.parse_args()

try:
    from pylibfreenect2 import OpenGLPacketPipeline
    pipeline = OpenGLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenCLPacketPipeline
        pipeline = OpenCLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline
        pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)

enable_rgb = True
enable_depth = True

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("No device connected!")
    sys.exit(1)

serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

types = 0
if enable_rgb:
    types |= FrameType.Color
if enable_depth:
    types |= (FrameType.Ir | FrameType.Depth)
listener = SyncMultiFrameListener(types)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

if enable_rgb and enable_depth:
    device.start()
else:
    device.startStreams(rgb=enable_rgb, depth=enable_depth)

# NOTE: must be called after device.start()
if enable_depth:
    registration = Registration(device.getIrCameraParams(),
                                device.getColorCameraParams())

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

time_list = []

# specify output folder
out_dir = args.out_dir
out_name = args.out_name

# height = 540
# width = 960
height = 1080 // 2
width = 1920 // 2
channel = 3
fps = 30
cnt = 0
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# cv2.VideoWriter_fourcc(*'mp4v')
# cv2.VideoWriter_fourcc(*'MP42')

color_wrapper = cv2.VideoWriter(out_dir + out_name + 'weed30_color0.mp4', fourcc, float(fps), (width, height), True)
depth_wrapper = cv2.VideoWriter(out_dir + out_name + 'weed30_depth0.mp4', fourcc, float(fps), (512, 424), False)
ir_wrapper = cv2.VideoWriter(out_dir + out_name + 'weed30_ir0.mp4', fourcc, float(fps), (512, 424), False)
register_wrapper = cv2.VideoWriter(out_dir + out_name + 'weed30_register0.mp4', fourcc, float(fps), (512, 424), True)

start_time = datetime.datetime.now()

while True:
    frames = listener.waitForNewFrame()

    if enable_rgb:
        color = frames["color"]

    if enable_depth:
        ir = frames["ir"]
        depth = frames["depth"]
    if enable_rgb and enable_depth:
        registration.apply(color, depth, undistorted, registered)
    elif enable_depth:
        registration.undistortDepth(depth, undistorted)

    if enable_depth:
        cv2.imshow("ir0", ir.asarray() / 65535.)
        cv2.imshow("depth0", depth.asarray() / 4500.)
        cv2.imshow("undistorted0", undistorted.asarray(np.float32) / 4500.)
    if enable_rgb:
        cv2.imshow("color0", cv2.resize(color.asarray(),
                                       (int(1920 / 3), int(1080 / 3))))
    if enable_rgb and enable_depth:
        cv2.imshow("registered0", registered.asarray(np.uint8))

    color_wrapper.write(cv2.resize(color.asarray(), (width, height))[:, :, :-1])
    color_wrapper.write(color.asarray()[:, :, :-1])
    depth_wrapper.write((depth.asarray() * (255.0 / 4500.0)).clip(0, 255).astype(np.uint8))
    ir_wrapper.write((ir.asarray() * (255.0 / 65535.0)).clip(0, 255).astype(np.uint8))
    register_wrapper.write(registered.asarray(np.uint8)[:, :, :-1])
    time_list.append(datetime.datetime.now())


    listener.release(frames)
    cnt += 1

    # key = cv2.waitKey(delay=1)
    # if key == ord('q'):
    #     break

    # if keyboard.is_pressed("q"):  # if key 'q' is pressed
    #     print('finishing the loop')
    #     break  # finishing the loop
    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break

ir_wrapper.release()
print("save ir video successfully")
depth_wrapper.release()
print("save depth video successfully")
color_wrapper.release()
print("save color video successfully")
register_wrapper.release()
print("save registered video successfully")
np.save(out_dir + out_name + '_time.npy', time_list)
print("save time array successfully")
print("total frame is: ", cnt)