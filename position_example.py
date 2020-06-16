from OpenGL.GLUT import *
import positional_tracking.tracking_viewer as tv
import pyzed.sl as sl
import threading
import cv2
import math
import numpy as np
from scipy import optimize


def rotY(x, tran, x0, num):  # 旋转平移矩阵,x=旋转角度，tran=平移量，x0为原矩阵值，num为xORyORz
    q = np.zeros(shape=[4, 1])
    src = np.zeros(shape=[4, 1])
    q[num] = tran
    q[3] = 1
    rot = np.array([[math.cos(x), 0, math.sin(x)],
                    [0, 1, 0],
                    [-1 * math.sin(x), 0, math.cos(x)],
                    [0, 0, 0]])
    p = np.c_[rot, q]
    src[num] = x0
    src[3] = 1
    result = np.dot(p, src)
    print(result[num])
    return result[num][0]


def piecewise_curve(x, x0, y0, k2):  # 曲线拟合
    return np.piecewise(x, [x < x0, x >= x0], [lambda x: y0,
                                               lambda x: -4.8 * (x - x0) * (x - x0) + y0 + k2 * (x - x0)
                                               ])


def piecewise_linear(x, x0, y0, k1):  # 直线拟合
    return np.piecewise(x, [x < x0, x >= x0], [lambda x: y0,
                                               lambda x: k1 * x + y0 - k1 * x0])


def tracking(viewer, xs, ys, zs, time1, y):  # 实时拟合
    text = ""
    print("旋转后的点", xs[len(xs) - 1], ys[len(ys) - 1], zs[len(zs) - 1])
    xd = np.linspace(0, time1[len(time1) - 1] + 1, 100)
    p3, e3 = optimize.curve_fit(piecewise_linear, time1, zs, bounds=([0, -10, -10], [5, 10, 10]))   # 直线拟合z轴
    p2, e2 = optimize.curve_fit(piecewise_linear, time1, xs, bounds=([p3[0] - 0.1, -10, -10], [p3[0] + 0.1, 10, 10]))   # 直线拟合X轴
    p, e = optimize.curve_fit(piecewise_curve, time1, ys,
                              bounds=([p3[0] - 0.1, -1, -10], [p3[0] + 0.1, 1, 10]))
    print("Y,X,Z的K值", p[2], p2[2], p3[2])
    for i in range(len(piecewise_curve(xd, *p))):
        if piecewise_curve(xd, *p)[i] < y:
            prex = piecewise_linear(xd, *p2)[i]
            prey = piecewise_curve(xd, *p)[i]
            prez = piecewise_linear(xd, *p3)[i]
            print("落点 是 ", piecewise_linear(xd, *p2)[i], piecewise_curve(xd, *p)[i], piecewise_linear(xd, *p3)[i])
            text = str((prex, prey, prez))
            break
    viewer.update_football_traking(piecewise_linear(xd, *p2).tolist(), piecewise_curve(xd, *p).tolist(),
                                   piecewise_linear(xd, *p3).tolist())
    viewer.update_true_football_traking(xs, ys, zs)
    return text


def findBiggestContours(image, contours):  # 最大轮廓
    maxarea = 0
    maxx = None
    if contours is None:
        print("sorry")
    for i in contours:
        rect = cv2.minAreaRect(i)
        if rect[0][0] > 450:      # 仅仅匹配素材，后期轮廓这块交给目标检测
            continue
        if cv2.contourArea(i) > maxarea:
            maxarea = cv2.contourArea(i)
            maxx = i

    if maxx is not None:
        rect = cv2.minAreaRect(maxx)
        print("middle is " + str(rect[0][0]), str(rect[0][1]))
        return rect[0]
    else:
        return None


def main():
    init = sl.InitParameters(camera_resolution=sl.RESOLUTION.RESOLUTION_HD720,
                             depth_mode=sl.DEPTH_MODE.DEPTH_MODE_ULTRA,
                             coordinate_units=sl.UNIT.UNIT_METER,
                             camera_fps=100,
                             depth_minimum_distance=0.6,
                             coordinate_system=sl.COORDINATE_SYSTEM.COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP,
                             svo_input_filename="C:\\Users\\Administrator\\Documents\\ZED\\HD720_SN24807_16-33-54.svo",
                             sdk_verbose=True)
    cam = sl.Camera()
    status = cam.open(init)

    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    transform = sl.Transform()
    tracking_params = sl.TrackingParameters(transform)
    cam.enable_tracking(tracking_params)

    # 位姿初始化
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.SENSING_MODE_STANDARD
    camera_pose = sl.Pose()

    # 界面初始化
    viewer = tv.PyTrackingViewer()
    viewer.init()
    # Prepare new image size to retrieve half-resolution images
    image_size = cam.get_resolution()
    new_width = image_size.width / 2
    new_height = image_size.height / 2

    # Declare your sl.Mat matrices
    image_zed = sl.Mat(new_width, new_height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
    point_cloud = sl.Mat()

    # 时间戳初值
    # time = sl.get_current_timestamp()
    ball_list = np.array([0, 0])
    flag = 1  # 第一次标志位
    flag2 = 0  # 落点标志位
    time1 = []  # 时间戳
    xs = []  # x轴
    ys = []  # y轴
    zs = []  # z轴

    py_translation = sl.Translation()
    start_zed(cam, runtime, camera_pose, viewer, py_translation, new_width, new_height, image_zed, point_cloud,
              ball_list, flag, flag2, time1, xs, ys, zs)

    viewer.exit()
    glutMainLoop()  # openGL


def start_zed(cam, runtime, camera_pose, viewer, py_translation, new_width, new_height, image_zed, point_cloud,
              ball_list, flag, flag2, time1, xs, ys, zs):
    zed_callback = threading.Thread(target=run, args=(
        cam, runtime, camera_pose, viewer, py_translation, new_width, new_height, image_zed, point_cloud, ball_list,
        flag, flag2,
        time1, xs, ys, zs))
    zed_callback.start()


def run(cam, runtime, camera_pose, viewer, py_translation, new_width, new_height, image_zed, point_cloud, ball_list,
        flag, flag2, time1, xs, ys, zs):
    text1 = ""
    text2 = ""
    rx = 0
    ry = 0
    rz = 0
    tx = 0
    ty = 0
    tz = 0
    yy = -0.7
    while True:
        if cam.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            tracking_state = cam.get_position(camera_pose)
            text_translation = ""
            text_rotation = ""
            # -----------------------位姿获取------------------------#
            if tracking_state == sl.TRACKING_STATE.TRACKING_STATE_OK:
                rotation = camera_pose.get_rotation_vector()
                rx = round(rotation[0], 3)
                ry = round(rotation[1], 3)
                rz = round(rotation[2], 3)

                translation = camera_pose.get_translation(py_translation)
                tx = round(translation.get()[0], 3)
                ty = round(translation.get()[1], 3)
                tz = round(translation.get()[2], 3)

                text_translation = str((tx, ty, tz))
                text_rotation = str((rx, ry, rz))
                pose_data = camera_pose.pose_data(sl.Transform())
                viewer.update_zed_position(pose_data)
            viewer.update_text(text_translation, text_rotation, tracking_state)
            # 图像获取
            # Retrieve the left image, depth image in the half-resolution
            cam.retrieve_image(image_zed, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU, int(new_width), int(new_height))
            # Retrieve the RGBA point cloud in half resolution
            cam.retrieve_measure(point_cloud, sl.MEASURE.MEASURE_XYZRGBA, sl.MEM.MEM_CPU, int(new_width),
                                 int(new_height))
            # 轮廓获取
            image_ocv = image_zed.get_data()

            hsv_image = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2HSV)
            binary_output = cv2.inRange(hsv_image, (80, 100, 10), (130, 204, 109))

            binary_output = cv2.morphologyEx(binary_output, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # 闭运算

            binary_output, contours, hierarchy = cv2.findContours(binary_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            ball = findBiggestContours(binary_output, contours)
            if ball is not None:
                min = 99999
                # summ = 0
                # num = 0
                min_point = [0, 0]
                ball_list[0] = round(ball[0])
                ball_list[1] = round(ball[1])
                # print(ball)
                cv2.circle(image_ocv, (ball_list[0], ball_list[1]), 3, (0, 0, 255), -1)
                for i in range(3):
                    for j in range(3):
                        x = ball_list[0] - 3 + i
                        y = ball_list[1] - 3 + j
                        point = point_cloud.get_value(x, y)
                        if min > point[1][2]:
                            min = point[1][2]
                            min_point[0] = ball_list[0] - 3 + i
                            min_point[1] = ball_list[1] - 3 + j
                # for i in range(11):
                #     for j in range(11):
                #         x = ball_list[0] - 5 + i
                #         y = ball_list[1] - 5 + j
                #         point = point_cloud.get_value(x, y)
                #         if min < point[1][2] + 0.5:
                #             summ = summ + point[1][2]
                #             num = num + 1
                # min_point
                ball_list = min_point
                # print(ball_list)
                if ball_list[0] == 0:     # 第一次
                    continue
                point_cloud_value = point_cloud.get_value(ball_list[0], ball_list[1])
                x = point_cloud_value[1][0]
                y = point_cloud_value[1][1]
                z = point_cloud_value[1][2]
                # print(x, y, z)
                if y < -0.65 and flag2 == 0:  # 实际落点位置，需标定
                    # if y < -0.65 or x > 3.5:
                    yy = round(y + ty)
                    text2 = str((x + tx, y + ty, z + tz))
                    print(text2)
                    flag2 = 2
                if len(zs) > 1 and zs[len(zs) - 1] - z > 0.5:  # 异常信息
                    print("point out")
                    continue
                if flag == 1:  # 获取初次时间戳
                    flag = 0
                    time = camera_pose.timestamp
                # 时间戳
                timestamp = camera_pose.timestamp
                print()
                # print("time: " + str(timestamp - time))

                timing = (timestamp - time) % 10000000000 / 1000000000
                if tracking_state == sl.TRACKING_STATE.TRACKING_STATE_OK and flag2 == 0:  # 更新坐标
                    time1.append(timing)
                    xs.append(rotY(rx, tx, x, 0))
                    ys.append(rotY(ry, ty, y, 1))
                    zs.append(rotY(rz, tz, z, 2))
                if timing > 0.3 and flag2 == 0 and len(xs) > 3:  # 拟合轨迹
                    # if flag2 == 0 and len(xs) > 3:
                    text1 = tracking(viewer, xs, ys, zs, time1, yy)
                viewer.update_value(text2, text1)
        else:
            sl.c_sleep_ms(1)
        cv2.imshow("Image", image_ocv)
        if (cv2.waitKey(12) >= 0):
            break


if __name__ == "__main__":
    main()
