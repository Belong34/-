# 测试 By 狗仔源
### 2020/1/10

添加旋转平移状态下的轨迹预测，但是X轴的图像有时候会偏移，不知道是旋转矩阵的问题还是说测试视频的不完善(在x轴上也旋转了or摄像头放歪了)

下一步方向打算是目标追踪。

### 2020/1/9

老师建议:mark下:
1.最小值滤波改成在取得最小值之后设置以最小值设置阈值均值滤波
2.分级处理，即远近的时候滤波器大小发生改变(降低帧率)
3.初始设置特征扫描,确定巨型ROI后，在roi范围内做椭圆拟合

### 2020/1/8

完成视频流下的移动坐标系的轨迹预测的可视化界面显示

### 2020/1/7

增加了视频流下的移动坐标系的轨迹预测

### 2019/12/16

调试程序完成，存在问题好像是在灯附件的距离测得有问题，明日重新录制视频。

### 2019/12/17

重新录了一次视频，在刚出手的几个点，Z和X都有偏差，相当于这几个点废了。一般是到达最高点开始下降的时候,数据才恢复正常。感觉可能和光线或者运动有关，明天试着到固定位置取读取数值，或者调整摄像机的一些颜色参数试一试。
