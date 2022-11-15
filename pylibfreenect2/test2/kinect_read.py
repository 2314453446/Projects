import numpy as np
import cv2
import os
import h5py

# 读取像素值坐标
pixel_x = 512 // 2
pixel_y = 424 // 2

def video2image(video_dir, save_dir):
    cap = cv2.VideoCapture(video_dir)  # 生成读取视频对象
    n = 1  # 计数
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))  # 获取视频的宽度
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))  # 获取视频的高度
    fps = cap.get(cv2.CAP_PROP_FPS)  # 获取视频的帧率
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))  # 视频的编码
    # 定义视频输出
    # writer = cv2.VideoWriter("teswellvideo_02_result.mp4", fourcc, fps, (width, height))
    i = 0
    timeF = int(fps)  # 视频帧计数间隔频率
    while cap.isOpened():
        ret, frame = cap.read()  # 按帧读取视频
        # 到视频结尾时终止
        if ret is False:
            break
        # 每隔timeF帧进行存储操作
        if (n % timeF == 0):
            i += 1
            print('保存第 %s 张图像' % i)
            save_image_dir = os.path.join(save_dir, 'weed180_col%s.jpg' % i)
            print('save_image_dir: ', save_image_dir)
            cv2.imwrite(save_image_dir, frame)  # 保存视频帧图像
        n = n + 1
        cv2.waitKey(1)  # 延时1ms
    cap.release()  # 释放视频对象


# 读取文件夹所有视频，每个视频按帧保存图像
def video2image_multi(video_path, save_path):
    video_list = os.listdir(video_path)

    for i in range(len(video_list)):
        video_dir = os.path.join(video_path, video_list[i])
        cap = cv2.VideoCapture(video_dir)
        fps = cap.get(cv2.CAP_PROP_FPS)  # 视频的帧率
        save_num = 0
        n = 1  # 计数
        timeF = int(fps)  # 视频帧计数间隔频率
        while cap.isOpened():
            ret, frame = cap.read()
            if ret is False:
                break
            # 每隔timeF帧进行存储操作
            if (n % timeF == 0):
                save_num += 1
                save_image_dir = os.path.join(save_path, "%s_%s.jpg"%(i, save_num))
                cv2.imwrite(save_image_dir, frame)
            n = n + 1
            cv2.waitKey(1)
        cap.release()
        print('读取第 %s 个视频完成 ！！！' % i)

'''
h5文件解码存储
'''
def h52img(h5_path , h5save_dir='./'):
    # save_dir :图片需要存入的文件夹
    h5 = h5py.File(h5_path,'r')
    os.makedirs(h5save_dir,exist_ok=True)
    for key in h5.keys():
        img = cv2.imdecode(np.array(h5[key]),-1)
        print(img)
        img_name = os.path.join(h5save_dir,key)  if key.endswith('.png') else os.path.join(h5save_dir, key + '.png')
        # 判断文件是否以png结尾
        cv2.imwrite(img_name,img)

'''
h5解码图片读取显示
'''
def h5PngRead(PngImg):
    img = cv2.imread(PngImg,flags=cv2.IMREAD_UNCHANGED)
    img2 = img.copy()
    cv2.circle(img2,(pixel_x,pixel_y),1,(0,255,255),4)
    cv2.imshow('depthponit',img2)
    print(img)
    cv2.waitKey(10000)
    cv2.destroyAllWindows()
    depth_value = img[pixel_x,pixel_y]
    print("坐标{}".format(depth_value))

def read_pixel(img):
    depth_img = cv2.imread(img,cv2.IMREAD_GRAYSCALE)
    depth_value=depth_img[pixel_y,pixel_x] /255 *4500
    print("坐标{}".format(depth_value))



img = r"/home/tuolong/Software/libfreenect2/pylibfreenect2/save/image_depth/4.jpg"
video_path = r'/home/tuolong/Software/libfreenect2/pylibfreenect2/save/test1026weed180_color0.mp4'
save_path = r'/home/tuolong/Software/libfreenect2/pylibfreenect2/save/image_depth'
# h5待解码文件的读取路径
h5path = r'/home/tuolong/Software/libfreenect2/pylibfreenect2/save/test1026depth_map.hdf5'
# h5文件读取后.png文件的存储路径
h5PngSave = r'/home/tuolong/Software/libfreenect2/pylibfreenect2/save/h5Png'

# .png 文件路径
pngimg = r'/home/tuolong/Software/libfreenect2/pylibfreenect2/save/h5Png/3.png'

if __name__ == '__main__':
    '''
    rgb 和 8位深度图读取存储方式
    '''
    # video2image(video_path,save_path)

    '''
    h5 16位.png格式 深度图解码存储方式
    '''
    # h52img(h5path,h5PngSave)

    # read_pixel(img)
    h5PngRead(pngimg)