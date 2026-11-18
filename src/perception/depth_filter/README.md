3. depth_filter：清理深度图
相机深度图可能有无效像素、噪声和跳动。这个模块负责：

将深度转换为以米为单位的数据；
去除超出配置范围的数据；
对连续帧做一定平滑；
按配置决定是否填补小孔洞。
其中 depth_filter_node.py 负责 ROS 订阅与发布，depth_filters.py 实现过滤逻辑。

你当前选择硬件深度时，数据路径是：
/camera/camera/depth/image_rect_raw
        ↓
    depth_filter
        ↓
/uav1/d435i/depth/image_filtered

注意：输出话题名字里虽然写着 d435i，这只是项目沿用的话题名称，不代表实际连接的相机一定是 D435i。