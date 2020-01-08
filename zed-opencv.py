import sys
import numpy as np
import pyzed.sl as sl
import cv2
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

help_string = "[s] Save side by side image [d] Save Depth, [n] Change Depth format, [p] Save Point Cloud, [m] Change Point Cloud format, [q] Quit"
prefix_point_cloud = "Cloud_"
prefix_depth = "Depth_"
path = "./"

count_save = 0
mode_point_cloud = 0
mode_depth = 0
point_cloud_format = sl.POINT_CLOUD_FORMAT.POINT_CLOUD_FORMAT_XYZ_ASCII
depth_format = sl.DEPTH_FORMAT.DEPTH_FORMAT_PNG


def findBiggestContours(image, contours):
    maxarea = 0
    maxx = None
    if contours is None:
        print("sorry")
    for i in contours:
        rect = cv2.minAreaRect(i)
        if 380 < rect[0][0] < 425 and 200 < rect[0][1] < 223:
            continue
        if rect[0][0] > 480:
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


#
#
# def get_depth_format_name(f):
#     if f == sl.DEPTH_FORMAT.DEPTH_FORMAT_PNG:
#         return "PNG"
#     elif f == sl.DEPTH_FORMAT.DEPTH_FORMAT_PFM:
#         return "PFM"
#     elif f == sl.DEPTH_FORMAT.DEPTH_FORMAT_PGM:
#         return "PGM"
#     else:
#         return ""
#
#
# def get_point_cloud_format_name(f):
#     if f == sl.POINT_CLOUD_FORMAT.POINT_CLOUD_FORMAT_XYZ_ASCII:
#         return "XYZ"
#     elif f == sl.POINT_CLOUD_FORMAT.POINT_CLOUD_FORMAT_PCD_ASCII:
#         return "PCD"
#     elif f == sl.POINT_CLOUD_FORMAT.POINT_CLOUD_FORMAT_PLY_ASCII:
#         return "PLY"
#     elif f == sl.POINT_CLOUD_FORMAT.POINT_CLOUD_FORMAT_VTK_ASCII:
#         return "VTK"
#     else:
#         return ""
#
#
# def save_point_cloud(zed, filename):
#     print("Saving Point Cloud...")
#     saved = sl.save_camera_point_cloud_as(zed, point_cloud_format, filename, True)
#     if saved:
#         print("Done")
#     else:
#         print("Failed... Please check that you have permissions to write on disk")
#
#
# def save_depth(zed, filename):
#     max_value = 65535.
#     scale_factor = max_value / zed.get_depth_max_range_value()
#
#     print("Saving Depth Map...")
#     saved = sl.save_camera_depth_as(zed, depth_format, filename, scale_factor)
#     if saved:
#         print("Done")
#     else:
#         print("Failed... Please check that you have permissions to write on disk")
#
#
# def save_sbs_image(zed, filename):
#     image_sl_left = sl.Mat()
#     zed.retrieve_image(image_sl_left, sl.VIEW.VIEW_LEFT)
#     image_cv_left = image_sl_left.get_data()
#
#     image_sl_right = sl.Mat()
#     zed.retrieve_image(image_sl_right, sl.VIEW.VIEW_RIGHT)
#     image_cv_right = image_sl_right.get_data()
#
#     sbs_image = np.concatenate((image_cv_left, image_cv_right), axis=1)
#
#     cv2.imwrite(filename, sbs_image)
#
#
# def process_key_event(zed, key):
#     global mode_depth
#     global mode_point_cloud
#     global count_save
#     global depth_format
#     global point_cloud_format
#
#     if key == 100 or key == 68:
#         save_depth(zed, path + prefix_depth + str(count_save))
#         count_save += 1
#     elif key == 110 or key == 78:
#         mode_depth += 1
#         depth_format = sl.DEPTH_FORMAT(mode_depth % 3)
#         print("Depth format: ", get_depth_format_name(depth_format))
#     elif key == 112 or key == 80:
#         save_point_cloud(zed, path + prefix_point_cloud + str(count_save))
#         count_save += 1
#     elif key == 109 or key == 77:
#         mode_point_cloud += 1
#         point_cloud_format = sl.POINT_CLOUD_FORMAT(mode_point_cloud % 4)
#         print("Point Cloud format: ", get_point_cloud_format_name(point_cloud_format))
#     elif key == 104 or key == 72:
#         print(help_string)
#     elif key == 115:
#         save_sbs_image(zed, "ZED_image" + str(count_save) + ".png")
#         count_save += 1
#     else:
#         a = 0
#
#
# def print_help():
#     print(" Press 's' to save Side by side images")
#     print(" Press 'p' to save Point Cloud")
#     print(" Press 'd' to save Depth image")
#     print(" Press 'm' to switch Point Cloud format")
#     print(" Press 'n' to switch Depth format")


def main():
    # Create a ZED camera object
    zed = sl.Camera()
    xs = []
    ys = []
    zs = []
    # Set configuration parameters
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720
    init.camera_fps = 60
    init.depth_mode = sl.DEPTH_MODE.DEPTH_MODE_ULTRA
    init.coordinate_units = sl.UNIT.UNIT_METER
    # init.coordinate_system = sl.COORDINATE_SYSTEM.COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP  # Use a right-handed Y-up coordinate system
    init.depth_minimum_distance = 0.6
    zed.set_depth_max_range_value(7)
    if len(sys.argv) >= 2:
        init.svo_input_filename = sys.argv[1]
    # init.svo_input_filename = "C:\\Users\\Administrator\\Documents\\ZED\\HD720_SN24807_16-48-49.svo"
    key = 0
    # Open the camera
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(err))
        zed.close()
        exit(1)

    # Enable positional tracking with default parameters
    transform = sl.Transform()
    tracking_parameters = sl.TrackingParameters(transform)
    err = zed.enable_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(-1)

    zed_pose = sl.Pose()

    # zed.set_confidence_threshold(50)

    # Set runtime parameters after opening the camera
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.SENSING_MODE_STANDARD

    # Prepare new image size to retrieve half-resolution images
    image_size = zed.get_resolution()
    new_width = image_size.width / 2
    new_height = image_size.height / 2

    # Declare your sl.Mat matrices
    image_zed = sl.Mat(new_width, new_height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
    depth_image_zed = sl.Mat(new_width, new_height, sl.MAT_TYPE.MAT_TYPE_8U_C4)
    point_cloud = sl.Mat()

    # 时间戳初值
    # time = sl.get_current_timestamp()
    ball_list = np.array([0, 0])
    flag = 1
    time1 = []
    while True:
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            # Get the pose of the camera relative to the world frame
            state = zed.get_position(zed_pose, sl.REFERENCE_FRAME.REFERENCE_FRAME_WORLD)
            # Display translation and timestamp
            py_translation = sl.Translation()
            tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
            ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
            tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
            print("Translation: tx: {0}, ty:  {1}, tz:  {2}, timestamp: {3}\n".format(tx, ty, tz, zed_pose.timestamp))
            # Display orientation quaternion
            py_orientation = sl.Orientation()
            ox = round(zed_pose.get_orientation(py_orientation).get()[0], 3)
            oy = round(zed_pose.get_orientation(py_orientation).get()[1], 3)
            oz = round(zed_pose.get_orientation(py_orientation).get()[2], 3)
            ow = round(zed_pose.get_orientation(py_orientation).get()[3], 3)
            print("Orientation: ox: {0}, oy:  {1}, oz: {2}, ow: {3}\n".format(ox, oy, oz, ow))

            # Retrieve the left image, depth image in the half-resolution
            zed.retrieve_image(image_zed, sl.VIEW.VIEW_LEFT, sl.MEM.MEM_CPU, int(new_width), int(new_height))
            zed.retrieve_image(depth_image_zed, sl.VIEW.VIEW_DEPTH, sl.MEM.MEM_CPU, int(new_width), int(new_height))
            # Retrieve the RGBA point cloud in half resolution
            zed.retrieve_measure(point_cloud, sl.MEASURE.MEASURE_XYZRGBA, sl.MEM.MEM_CPU, int(new_width),
                                 int(new_height))

            if flag == 1:
                flag = 0
                time = zed_pose.timestamp
            # 时间戳
            # timestamp = sl.get_current_timestamp()
            timestamp = zed_pose.timestamp
            print()
            print("time: " + str(timestamp - time))

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            image_ocv = image_zed.get_data()

            hsv_image = cv2.cvtColor(image_ocv, cv2.COLOR_BGR2HSV)
            binary_output = cv2.inRange(hsv_image, (80, 100, 10), (130, 204, 109))

            binary_output = cv2.morphologyEx(binary_output, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # 闭运算

            binary_output, contours, hierarchy = cv2.findContours(binary_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            ball = findBiggestContours(binary_output, contours)
            # ball = (240,205)
            if ball is not None:
                min = 99999
                min_point = [0, 0]
                ball_list[0] = round(ball[0])
                ball_list[1] = round(ball[1])
                print(ball)
                cv2.circle(image_ocv, (ball_list[0], ball_list[1]), 3, (0, 0, 255), -1)
                cv2.imshow("Image", image_ocv)
                if (cv2.waitKey(12) >= 0):
                    break
                for i in range(11):
                    for j in range(11):
                        x = ball_list[0] - 5 + i
                        y = ball_list[1] - 5 + j
                        point = point_cloud.get_value(x, y)
                        # print(point)
                        if min > point[1][2]:
                            min = point[1][2]
                            min_point[0] = ball_list[0] - 5 + i
                            min_point[1] = ball_list[1] - 5 + j
                ball_list = min_point
                # if ball_list[0] == 0:
                #     continue
                print(ball_list)
                if ball_list[0]==0:
                    continue
                point_cloud_value = point_cloud.get_value(ball_list[0], ball_list[1])
                x = point_cloud_value[1][0]
                y = point_cloud_value[1][1]
                z = point_cloud_value[1][2]
                print(x, y, z)
                if len(zs) > 1 and zs[len(zs) - 1] - z < -0.5:
                    print("outoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutoutout")
                    continue
                time1.append((timestamp - time) % 10000000000 / 1000000000)
                xs.append(x + tx)
                ys.append(y + ty)
                zs.append(z + tz)
                # distance = math.sqrt(x * x + y * y + z * z)
                # print("距离为" + str(distance))
                # print()
            # cv2.imshow("Depth", depth_image_ocv)
            # cv2.imshow("Dept2h", binary_output)
            # process_key_event(zed, key)

    # print(xs, ys, zs)

    # fig = plt.figure()
    # ax = fig[0][0].gca(projection='3d')
    # ax = fig.add_subplot(221, projection='3d')
    # ax.view_init(135, -90)
    # ax.grid(True)
    # ax.plot(xs, zs, ys)
    # plt.ylabel("y")
    # plt.xlabel("x")
    #
    # ax1 = fig.add_subplot(222)
    # ax2 = fig.add_subplot(223)
    # ax3 = fig.add_subplot(224)
    # ax1.plot(time1, xs)
    # ax2.plot(time1, ys)
    # ax3.plot(time1, zs)
    # plt.show()
    from scipy import optimize
    def piecewise_curve(x, x0, y0, k2, k1):
        # x<x0 ⇒ lambda x: k1*x + y0 - k1*x0
        # x>=x0 ⇒ lambda x: k2*x + y0 - k2*x0
        return np.piecewise(x, [x < x0, x >= x0], [lambda x: y0,
                                                   lambda x: 4.8 * (x - x0) * (x - x0) + y0 + k2 * (x - x0)
                                                   # lambda x: ((k2 * x1 - k2 * x0) / (x1 - x0)) * (
                                                   #         x - x0) + y0,
                                                   # # lambda x: k2 * x + y0 - k2 * x1,
                                                   # lambda x: -k2 * x + ((k2 * x1 - k2 * x0) / (
                                                   #         x1 - x0)) * (
                                                   #                   x1 - x0) + y0 + k2 * x1
                                                   ])
        # return np.piecewise(x, [x < x0, (x1 >= x) & (x >= x0), x > x1], [lambda x: y0,
        #                                                                  lambda x: ((k2 * x1 - k2 * x0) / (x1 - x0)) * (
        #                                                                          x - x0) + y0,
        #                                                                  # lambda x: k2 * x + y0 - k2 * x1,
        #                                                                  lambda x: -k2 * x + ((k2 * x1 - k2 * x0) / (
        #                                                                              x1 - x0)) * (
        #                                                                                    x1 - x0) + y0 + k2 * x1])

    def piecewise_linear(x, x0, y0, k1):
        return np.piecewise(x, [x < x0, x >= x0], [lambda x: y0,
                                                   lambda x: k1 * x + y0 - k1 * x0])

    # 用已有的 (x, y) 去拟合 piecewise_linear 分段函数
    fig = plt.figure()
    ax1 = fig.add_subplot(221)
    ax2 = fig.add_subplot(222)
    ax3 = fig.add_subplot(223)
    ax4 = fig.add_subplot(224, projection='3d')
    xd = np.linspace(0, time1[len(time1) - 1] + 2, 1000)
    p2, e2 = optimize.curve_fit(piecewise_linear, time1, xs)
    ax1.plot(time1, xs, "o")
    ax1.plot(xd, piecewise_linear(xd, *p2))
    p, e = optimize.curve_fit(piecewise_curve, time1, ys,
                              bounds=([p2[0] - 0.1, -1, -10, 1.05], [p2[0] + 0.1, 1, 10, 1.15]))
    ax2.plot(time1, ys, "o")
    ax2.plot(xd, piecewise_curve(xd, *p))
    p3, e3 = optimize.curve_fit(piecewise_linear, time1, zs)
    ax3.plot(time1, zs, "o")
    ax3.plot(xd, piecewise_linear(xd, *p3))
    ax4.view_init(105, 90)
    ax4.grid(True)
    ax4.plot(piecewise_linear(xd, *p2), piecewise_curve(xd, *p), piecewise_linear(xd, *p3))
    plt.ylabel("y")
    plt.xlabel("x")
    plt.show()
    for i in range(len(piecewise_curve(xd, *p))):
        if piecewise_curve(xd, *p)[i] > 0.7:
            print("落点 是 ", piecewise_linear(xd, *p2)[i], piecewise_curve(xd, *p)[i], piecewise_linear(xd, *p3)[i])
            break
    cv2.destroyAllWindows()
    zed.close()

    print("\nFINISH")


if __name__ == "__main__":
    main()
