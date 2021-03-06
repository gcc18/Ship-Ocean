# 视觉导航
1. 传感器  
    1. 双目摄像头
        1. 色彩图像（r,g,b）
        1. 深度图像（d）
    1. 激光雷达
        1. 位置点（x,y,z）
    1. IMU  
        1. 线加速度
        1. 角速度
        1. 重力加速度
        1. 基于动力学积分估计坐标变化
1. 数据前序处理
    1. 点云位置对齐
        1. 初始位置标定
        1. 基于深度边缘的特征匹配
        1. ICP算法
        1. FPFH算法
    1. DEM生成
        1. 数据存储和格式转换
        1. 可视化表达
        1. 分辨率和误差处理
1. 路径规划
    1. 全局A*算法
    1. 基于障碍物距离梯度的多路径并行优化
    1. 全局最优路径决策判别
    1. 局部区域下最优路径二次处理（曲线平滑度和视角最优）
    1. 局部区域下障碍物检测和规避
1. 降落场目标识别与跟踪定位
    1. 基于深度分割多个连续曲面
    1. 基于图片完成目标曲面判定
    1. 对目标曲面进行模型拟合
    1. 不同坐标系下的目标平面法向量换算
    1. 目标中心点测定
1. 着陆过程
    1. 基于运动学和动力学的目标跟踪
    1. 基于运动学和动力学的飞行轨迹计算
    1. 传感器视距外的盲降策略
    1. 紧急避险
1. 其他
    1. SLAN
        1. 定位和建图
        1. [VINS算法](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
    1. ROS
        1. 开发环境
        1. 统一数据格式模板
    1. AirSim
        1. 仿真环境建模
    1. Rviz
        1. 可视化工具
            1. 对topic进行订阅显示
