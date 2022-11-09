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

parser = argparse.ArgumentParser()
parser.add_argument('--out_name', type=str, help="the output names", default=None)
parser.add_argument('--out_dir', type=str, help="the output directory", default='data/')
args = parser.parse_args()

try:
    from pylibfreenect2 import OpenGLPacketPipeline

    pipeline0 = OpenGLPacketPipeline()
    pipeline1 = OpenGLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenCLPacketPipeline

        pipeline0 = OpenCLPacketPipeline()
        pipeline1 = OpenCLPacketPipeline()
    except:
        from pylibfreenect2 import CpuPacketPipeline

        pipeline0 = CpuPacketPipeline()
        pipeline1 = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline0).__name__)

enable_rgb = True
enable_depth = True

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("No device connected!")
    sys.exit(1)

serial0 = fn.getDeviceSerialNumber(0)
# serial1 = fn.getDeviceSerialNumber(1)
device0 = fn.openDevice(serial0, pipeline=pipeline0)
# device1 = fn.openDevice(serial1, pipeline=pipeline1)

types = 0
if enable_rgb:
    types |= FrameType.Color
if enable_depth:
    types |= (FrameType.Ir | FrameType.Depth)
listener0 = SyncMultiFrameListener(types)
# listener1 = SyncMultiFrameListener(types)

# Register listeners
device0.setColorFrameListener(listener0)
device0.setIrAndDepthFrameListener(listener0)

# device1.setColorFrameListener(listener1)
# device1.setIrAndDepthFrameListener(listener1)

if enable_rgb and enable_depth:
    device0.start()
    # device1.start()
else:
    device0.startStreams(rgb=enable_rgb, depth=enable_depth)
    # device1.startStreams(rgb=enable_rgb, depth=enable_depth)

# NOTE: must be called after device.start()
if enable_depth:
    registration0 = Registration(device0.getIrCameraParams(),
                                 device0.getColorCameraParams())
    # registration1 = Registration(device1.getIrCameraParams(),
    #                              device1.getColorCameraParams())

undistorted0 = Frame(512, 424, 4)
registered0 = Frame(512, 424, 4)

undistorted1 = Frame(512, 424, 4)
registered1 = Frame(512, 424, 4)

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
color_wrapper0 = cv2.VideoWriter(out_dir + out_name + '_color0.mp4', fourcc, float(fps), (width, height), True)
depth_wrapper0 = cv2.VideoWriter(out_dir + out_name + '_depth0.mp4', fourcc, float(fps), (512, 424), False)
ir_wrapper0 = cv2.VideoWriter(out_dir + out_name + '_ir0.mp4', fourcc, float(fps), (512, 424), False)
register_wrapper0 = cv2.VideoWriter(out_dir + out_name + '_register0.mp4', fourcc, float(fps), (512, 424), True)

# color_wrapper1 = cv2.VideoWriter(out_dir + out_name + '_color1.mp4', fourcc, float(fps), (width, height), True)
# depth_wrapper1 = cv2.VideoWriter(out_dir + out_name + '_depth1.mp4', fourcc, float(fps), (512, 424), False)
# ir_wrapper1 = cv2.VideoWriter(out_dir + out_name + '_ir1.mp4', fourcc, float(fps), (512, 424), False)
# register_wrapper1 = cv2.VideoWriter(out_dir + out_name + '_register1.mp4', fourcc, float(fps), (512, 424), True)

start_time = datetime.datetime.now()
while True:
    frames0 = listener0.waitForNewFrame()
    # frames1 = listener1.waitForNewFrame()

    if enable_rgb:
        color0 = frames0["color"]
        # color1 = frames1["color"]

    if enable_depth:
        ir0 = frames0["ir"]
        depth0 = frames0["depth"]
        # ir1 = frames1["ir"]
        # depth1 = frames1["depth"]
    if enable_rgb and enable_depth:
        registration0.apply(color0, depth0, undistorted0, registered0)
        # registration1.apply(color1, depth1, undistorted1, registered1)
    elif enable_depth:
        registration0.undistortDepth(depth0, undistorted0)
        # registration1.undistortDepth(depth1, undistorted1)

    if enable_depth:
        cv2.imshow("ir0", ir0.asarray() / 65535.)
        cv2.imshow("depth0", depth0.asarray() / 4500.)
        cv2.imshow("undistorted0", undistorted0.asarray(np.float32) / 4500.)
    #     cv2.imshow("ir1", ir1.asarray() / 65535.)
    #     cv2.imshow("depth1", depth1.asarray() / 4500.)
    #     cv2.imshow("undistorted1", undistorted1.asarray(np.float32) / 4500.)
    # if enable_rgb:
    #     cv2.imshow("color0", cv2.resize(color0.asarray(),
    #                                    (int(1920 / 3), int(1080 / 3))))
    #     cv2.imshow("color1", cv2.resize(color1.asarray(),
    #                            (int(1920 / 3), int(1080 / 3))))
    # if enable_rgb and enable_depth:
    #     cv2.imshow("registered0", registered0.asarray(np.uint8))
    #     cv2.imshow("registered1", registered1.asarray(np.uint8))

    color_wrapper0.write(cv2.resize(color0.asarray(), (width, height))[:, :, :-1])
    # color_wrapper1.write(cv2.resize(color0.asarray(), (width, height))[:, :, :-1])
    color_wrapper0.write(color0.asarray()[:, :, :-1])
    # color_wrapper1.write(color1.asarray()[:, :, :-1])
    depth_wrapper0.write()
    # depth_wrapper1.write((depth1.asarray() * (255.0 / 4500.0)).clip(0, 255).astype(np.uint8))
    ir_wrapper0.write((ir0.asarray() * (255.0 / 65535.0)).clip(0, 255).astype(np.uint8))
    # ir_wrapper1.write((ir1.asarray() * (255.0 / 65535.0)).clip(0, 255).astype(np.uint8))
    register_wrapper0.write(registered0.asarray(np.uint8)[:, :, :-1])
    # register_wrapper1.write(registered1.asarray(np.uint8)[:, :, :-1])
    time_list.append(datetime.datetime.now())

    listener0.release(frames0)
    # listener1.release(frames1)
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

ir_wrapper0.release()
# ir_wrapper1.release()
print("save ir video successfully")
depth_wrapper0.release()
# depth_wrapper1.release()
print("save depth video successfully")
color_wrapper0.release()
# color_wrapper1.release()
print("save color video successfully")
register_wrapper0.release()
# register_wrapper1.release()
print("save registered video successfully")
np.save(out_dir + out_name + '_time.npy', time_list)
print("save time array successfully")
print("total frame is: ", cnt)

end_time = datetime.datetime.now()
print("total time is: ", end_time - start_time)

device0.stop()
device0.close()

# device1.stop()
# device1.close()

sys.exit(0)