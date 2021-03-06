# SHENLAN_sensorfusion_assignment6
### Purpose : 利用中值法的惯性导航解算 && 最终以pose的形式可视化
深蓝学院-多传感器融合-第二期-第六章作业
本次作业的目的是惯性导航结算。
核心代码文件为Activity.cpp的文件。
里面不仅把IMU的config和subscriber都写好了，而且核心代码都已经写好了。
完成作业只需要读懂代码，看明白为什么是这样的以及如何调用就可以了

### 修改的部分
修改了Activity.cpp的UpdatePose()的函数。
要按照顺序依次调用，GetAngularDelta, UpdateOrientation, GetVelocityDelta,UpdatePosition函数。
顺序不能改变，否则旋转矩阵会被在没有更新的情况下被使用。

```
const size_t index_current = 1;
const size_t index_previous = 0;

Eigen::Vector3d angular_delta;
Eigen::Vector3d velocity_delta;
Eigen::Matrix3d R_curr;
Eigen::Matrix3d R_prev;
double delta_t;

GetAngularDelta(index_current, index_previous, angular_delta);

// update orientation:
UpdateOrientation(
    angular_delta,
    R_curr, R_prev);

// get velocity delta:
GetVelocityDelta(
    index_current, index_previous,
    R_curr, R_prev,
    delta_t, velocity_delta);

// update position:
UpdatePosition(delta_t, velocity_delta);
```

### 结果
生成了和Ground truth类似的蓝色的pose点。
![figure1](https://github.com/Fred159/SHENLAN_sensorfusion_assignment6/blob/main/figures/l6_assignment_1.png)

![figure2](https://github.com/Fred159/SHENLAN_sensorfusion_assignment6/blob/main/figures/l6_assignment_2.png)

![figure3](https://github.com/Fred159/SHENLAN_sensorfusion_assignment6/blob/main/figures/l6_assignment_3.png)

![figure4](https://github.com/Fred159/SHENLAN_sensorfusion_assignment6/blob/main/figures/l6_assignment_4.png)

![figure5](https://github.com/Fred159/SHENLAN_sensorfusion_assignment6/blob/main/figures/l6_assignment_5.png)

![figure6](https://github.com/Fred159/SHENLAN_sensorfusion_assignment6/blob/main/figures/l6_assignment_6.png)

