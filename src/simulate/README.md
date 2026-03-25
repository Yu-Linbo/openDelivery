# 仿真目录（`simulate`）

本目录为**容器目录**：其下只放置与仿真相关的 **ROS 2 功能包**，不在此层级直接放 `package.xml` / `CMakeLists.txt`。

当前包含：

| 包名 | 路径 | 说明 |
|------|------|------|
| **simulate** | [`simulate/simulate/`](simulate/simulate/) | Gazebo 无头仿真（差速、激光、IMU、相机等） |

新增仿真包时：在 `src/simulate/<你的包名>/` 下创建标准 ament 包即可，并与现有包并列。

详细话题、`colcon` 与故障排除见包内 **[`simulate/simulate/README.md`](simulate/simulate/README.md)**。
