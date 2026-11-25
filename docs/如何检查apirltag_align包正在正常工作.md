# 实时看，每1秒收到一条
ros2 topic echo /uav1/apriltag/alignment_status 

# 或者看最新的轨迹状态（VECTOR 有历史）
ros2 topic echo /uav1/apriltag/landing_target 

# 所有状态（对齐）
WAIT_CAMERA_INFO     未拿到相机内参（K矩阵），三角化做不了
IMAGE_INFO_MISMATCH  相机分辨率变了，尺寸对不上
BAD_IMAGE            ROS→CV2转换失败
TAG_NOT_VISIBLE      图像里没有 tag
TAG_NOT_UNIQUE       一个 tag 在识别时出现多次（角度分散）
TAG_TOO_SMALL        tag 像素面积太小，三角化会塌
POSE_INVALID         三角化方程无解，或者进不去
POSE_REJECTED        投影误差太大 / 高度不在合理范围 / 数据NaN
POSE_JUMP            XY 位置跳了超过 35cm
ACCUMULATING         追踪了但还不够5帧，还没稳定下来
TRACKING             最后一条成功追踪的消息