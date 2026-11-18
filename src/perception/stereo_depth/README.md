这个模块用左右两张图的差异计算物体距离：

software_stereo_node.py：接收左右图、进行时间同步并发布深度图。
stereo_matcher.py：根据双目标定和图像视差计算深度。
例如，左右镜头看同一个物体的位置有差别，算法结合镜头间距就能估计距离。

你当前不使用它：当前 launch.yaml 中 depth_source: hardware，项目使用 D430i 驱动直接发布的深度图；选择 software 才会启动这个软件双目节点。