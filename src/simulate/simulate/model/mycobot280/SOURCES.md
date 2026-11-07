# SmolVLA 移动操作模型来源

- 导入自本工作区 `smolvla-demo`，仓库 `https://github.com/Yu-Linbo/smolvla-demo`。
  精确提交、生成源码 SHA-256、收纳关节位置与资产哈希见 `smolvla-source.json`。
- 按 `block_pick_demo/model.py` 的 `build_robot('vla')` 生成机械臂、安装板、
  定制双指夹爪和腕部相机，使用 `block_pick_demo/simulation.py` 的 `FOLD_Q`。
  将原活动关节变换合入固定关节的 origin，保留几何、惯量和原比例。
- 机械臂网格与纹理原样复制自 `assets/mycobot280/`。上游为
  `elephantrobotics/mycobot_ros`，提交
  `c0d6fbe8f7d9b18b46a9fcb0b041e5bc2d66196b`；原始清单保存在 `source.json`，
  其中还列有本模型未使用的桌夹及原始 URDF。BSD 3-Clause 原文见 `LICENSE`。
- 安装板代替原桌夹。各节惯量为源 demo 的估算值，不是实体硬件辨识结果。
  夹爪和腕部相机外形为 demo 定制几何。
- 底盘继续使用 OpenDelivery 的 `urdf/simple_2d_robot.urdf.xacro`，保留原有
  差速驱动、激光、IMU、前视与下视相机，并添加 Gazebo 腕部相机。

从 OpenDelivery 根目录运行 `python3 scripts/import_smolvla_model.py ../smolvla-demo`
可以重新生成导入文件；安装后的模型不依赖导入源目录。
