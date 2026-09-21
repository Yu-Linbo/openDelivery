(function () {
  "use strict";

  const STORAGE_KEY = "openDelivery_language_v1";
  const DEFAULT_LOCALE = "zh-CN";
  const SUPPORTED_LOCALES = new Set(["zh-CN", "en"]);
  const TRANSLATABLE_ATTRIBUTES = ["title", "placeholder", "aria-label", "alt"];

  // The web console predates its localization layer and creates a large amount of
  // status text at runtime. Phrase-based translation keeps those messages and
  // newly inserted DOM fragments localized without coupling every call site to
  // the language picker. Longer phrases are applied first.
  const ENGLISH_PHRASES = {
    "暂无机器人数据；请确认 ROS 桥与": "No robot data. Check that the ROS bridge and",
    "已发布。仍可直接在下方填写新 ID 做仿真上线。": "is publishing. You can still enter a new ID below to start a simulation.",
    "填写命名空间 ID（与 Gazebo spawn 一致，如": "Enter the namespace ID used by Gazebo spawn (for example,",
    "），无需事先有历史缓存。": "). No cached history is required.",
    "暂无机器人数据；请确认 ROS 桥与 /*/robot_status 已发布。仍可直接在下方填写新 ID 做仿真上线。": "No robot data. Check that the ROS bridge and /*/robot_status are publishing. You can still enter a new ID below to start a simulation.",
    "填写命名空间 ID（与 Gazebo spawn 一致，如 robot3），无需事先有历史缓存。": "Enter the namespace ID used by Gazebo spawn (for example, robot3). No cached history is required.",
    "楼层地图与栅格展示；选择「robot*_mapping」可订阅对应机器人的 /<robot_name>/mapping 建图流": "Floor map and occupancy grid. Select robot*_mapping to subscribe to that robot's /<robot_name>/mapping stream.",
    "楼层地图与栅格展示；选择「robot*_mapping」可订阅对应机器人的 /": "Floor map and occupancy grid. Select robot*_mapping to subscribe to that robot's /",
    "/mapping 建图流": "/mapping stream",
    "「地图选位姿」用于重定位；「地图选目标点」会按当前机器人 ID 下发导航。两者均为单击定位置、再单击定朝向；Shift+ 拖拽仍平移地图。": "Use Pick pose for relocalization and Pick goal to navigate the selected robot. Click once for position and again for heading; Shift+drag still pans the map.",
    "「地图选位姿」用于重定位；「地图选目标点」会按当前机器人 ID 下发导航。两者均为单击定位置、再单击定朝向；": "Use Pick pose for relocalization and Pick goal to navigate the selected robot. Click once for position and again for heading;",
    "+ 拖拽仍平移地图。": "+drag still pans the map.",
    "读取已归档索引及当前录制 bag；已归档 bag 可下载，所有 bag 均可在浏览器中纯离线回放（不启动 ROS）": "Read archived indexes and active bags. Archived bags can be downloaded, and every bag can be replayed entirely in the browser without starting ROS.",
    "轻量页面直接显示 /drawn_model/topdown_camera/image_raw 实时画面（不依赖 GzWeb）。 点击相机画面只做取点，使用相机内参换算到 world 坐标，确认后再调用 /gazebo/set_model_state 移动模型。": "This lightweight page shows /drawn_model/topdown_camera/image_raw live without GzWeb. Click the camera image to select a point, convert it to world coordinates using camera intrinsics, then confirm before moving the model through /gazebo/set_model_state.",
    "轻量页面直接显示": "This lightweight page shows",
    "实时画面（不依赖 GzWeb）。": "live without GzWeb.",
    "点击相机画面只做取点，使用相机内参换算到 world 坐标，确认后再调用": "Click the camera image to select a point, convert it to world coordinates using camera intrinsics, then confirm before calling",
    "实时画面（不依赖 GzWeb）。 点击相机画面只做取点，使用相机内参换算到 world 坐标，确认后再调用": "live without GzWeb. Click the camera image to select a point, convert it to world coordinates using camera intrinsics, then confirm before calling",
    "移动模型。": "to move the model.",
    "拖拽画面平移，滚轮或双指缩放；单击画面仍会把目标位置换算到 world 坐标并填入右侧表单。": "Drag the view to pan, use the wheel or pinch to zoom, and click to convert a target into world coordinates for the form on the right.",
    "点按立即微调，长按渐进加速；也可使用 WASD / 方向键。区域按钮会自动居中并放大。": "Tap for an immediate nudge, hold for progressive acceleration, or use WASD / arrow keys. Zone buttons center and zoom automatically.",
    "拖拽只移动相机；滚轮会以光标位置为中心缩放，触屏支持单指平移和双指缩放。未发生拖动的单击仍只做 world 坐标取点，不直接移动模型。": "Dragging only moves the camera. The wheel zooms around the pointer; touch supports one-finger pan and pinch zoom. A click without dragging still picks world coordinates without moving the model.",
    "方向与缩放按键控制俯瞰相机；拖拽、滚轮和双指手势只调整本地图片视图。": "Direction and zoom buttons control the overhead camera; dragging, the wheel, and pinch gestures only adjust the local image view.",
    "点按立即微调，长按渐进加速；也可使用 WASD / 方向键。这里的按键会实际移动俯瞰相机。": "Tap for an immediate nudge, hold for progressive acceleration, or use WASD / arrow keys. These controls move the actual overhead camera.",
    "图片上的拖拽、滚轮和双指缩放只做局部查看，不会移动俯瞰相机。未发生拖动的单击仍用于 world 坐标取点。": "Dragging, wheel zoom, and pinch zoom on the image are local-only and do not move the overhead camera. A click without dragging still picks world coordinates.",
    "图片上的拖拽、滚轮和双指缩放只做局部查看，不会移动俯瞰相机。": "Dragging, wheel zoom, and pinch zoom on the image are local-only and do not move the overhead camera.",
    "未发生拖动的单击仍用于 world 坐标取点。": "A click without dragging still picks world coordinates.",
    "已移除图片上的拖拽和滚轮操控。当前通过按钮控制俯视相机，点击画面只做 world 坐标取点，不直接移动模型。": "Dragging and wheel controls on the image are disabled. Use the buttons to control the overhead camera; clicking only picks world coordinates and does not move the model directly.",
    "已移除图片上的拖拽和滚轮操控。当前通过按钮控制俯视相机，点击画面只做": "Dragging and wheel controls on the image are disabled. Use the buttons to control the overhead camera; clicking only picks",
    "world 坐标取点，不直接移动模型。": "world coordinates and does not move the model directly.",
    "点击左侧画面可快速填入目标 X/Y；Yaw 可手填微调，然后直接移动模型。": "Click the left view to fill target X/Y. Fine-tune Yaw manually, then move the model.",
    "调试视图：显示当前全部 ROS2 节点，支持可用的 lifecycle set 与 kill 操作。": "Debug view showing all current ROS 2 nodes with available lifecycle set and kill actions.",
    "取点时先点击位置，再点击方向；按住已有点位可直接拖动。": "Click a position, then a heading. Hold an existing point to drag it.",
    "按住按钮或 W/A/S/D；松开立即停车。": "Hold a button or W/A/S/D; release to stop immediately.",
    "通过 OpenDelivery API 操作当前页面": "Operate this page through the OpenDelivery API",
    "实时位姿 + 最近心跳缓存": "Live pose + recent heartbeat cache",
    "新机器人仿真上线": "Start a new simulated robot",
    "查看各机器人在线 / 离线状态": "View online/offline status for every robot",
    "机器人在线状态": "Robot connectivity status",
    "在线机器人快捷入口": "Online robot shortcuts",
    "机器人详情分类": "Robot detail categories",
    "关闭机器人详情": "Close robot details",
    "暂无在线机器人": "No online robots",
    "暂无在线机器人，无法移动模型": "No online robot is available to move",
    "正在读取机器人详情…": "Loading robot details…",
    "任务工作队列": "Task work queue",
    "关联进程 CPU / 内存": "Related process CPU / memory",
    "该机器人 ROS 节点": "Robot ROS nodes",
    "暂无该机器人 ROS 节点": "No ROS nodes for this robot",
    "暂无匹配的机器人进程": "No matching robot processes",
    "该机器人暂无日志索引。": "No log index for this robot.",
    "暂无工作项。": "No work items.",
    "切图 / 重定位": "Map switch / relocalization",
    "地图选目标点": "Pick goal on map",
    "地图选位姿": "Pick pose on map",
    "切图并重定位": "Switch map and relocalize",
    "记录重定位点": "Record relocalization point",
    "机器人遥控方向键": "Robot drive direction controls",
    "机器人遥控": "Robot teleoperation",
    "正在读取地图列表…": "Loading map list…",
    "正在读取地图…": "Loading maps…",
    "加载地图后即可编辑。": "Load a map to begin editing.",
    "滚轮缩放 · Shift/中键拖拽平移": "Wheel to zoom · Shift/middle-drag to pan",
    "请选择语义标签": "Select a semantic label",
    "取消图层修改": "Discard layer changes",
    "取消点位修改": "Discard point changes",
    "撤销上一步图层修改": "Undid the last layer change",
    "撤销上一步点位修改": "Undid the last point change",
    "确定放弃全部未保存的图层修改吗？": "Discard all unsaved layer changes?",
    "确定放弃全部未保存的点位修改吗？": "Discard all unsaved point changes?",
    "当前地图有未保存修改，确定切换地图吗？": "The current map has unsaved changes. Switch maps anyway?",
    "存在未保存修改，确定关闭编辑器吗？": "There are unsaved changes. Close the editor anyway?",
    "建图中的临时地图不可编辑，请输入已保存地图名称": "Temporary mapping maps cannot be edited. Select a saved map.",
    "地图已加载；默认使用栅格障碍画笔": "Map loaded. The occupancy obstacle brush is selected by default.",
    "没有可编辑的已保存地图": "No editable saved maps",
    "地图列表加载失败：": "Failed to load map list: ",
    "语义地图图片加载失败": "Failed to load semantic map image",
    "位置已确定，请再次点击地图设置方向": "Position set. Click the map again to set heading.",
    "点位位置和方向已更新，保存后生效": "Point position and heading updated; save to apply.",
    "正在保存图层…": "Saving layers…",
    "正在保存点位…": "Saving points…",
    "图层修改已保存": "Layer changes saved",
    "点位修改已保存": "Point changes saved",
    "图层保存失败：": "Failed to save layers: ",
    "点位保存失败：": "Failed to save points: ",
    "本地图暂无点位": "No points on this map",
    "新增点位：请先点击地图位置": "New point: click a map position first",
    "重选点位：请先点击新位置": "Reposition point: click a new position first",
    "已取消点位取点": "Point picking cancelled",
    "已启用点位设置；图层绘制已暂停": "Point editing enabled; layer drawing paused",
    "点位操作已暂停": "point editing paused",
    "图层绘制已暂停": "layer drawing paused",
    "请先填写点位名称": "Enter a point name first",
    "未加载地图": "No map loaded",
    "暂无已保存地图": "No saved maps",
    "请选择地图": "Select a map",
    "独立地图编辑画布": "Standalone map editor canvas",
    "关闭地图编辑": "Close map editor",
    "地图编辑画布": "Map editor canvas",
    "适应窗口": "Fit to window",
    "显示语义": "Show semantics",
    "显示底图": "Show base map",
    "显示点位": "Show points",
    "显示规划路径": "Show planned path",
    "显示 scan_2d": "Show scan_2d",
    "叠加语义地图": "Overlay semantic map",
    "勾选后显示当前地图的全部点位": "Show all points on the current map when enabled",
    "仅建图模式：保存到 map/<名称>/；切图时通过 /<robot>/robot_status 发布 current_map 字段也用此名": "Mapping mode only: save to map/<name>/. The current_map field published through /<robot>/robot_status uses the same name.",
    "地图目录名，如 floor1": "Map directory name, e.g. floor1",
    "保存到仓库 map/ 目录": "Save to the repository map/ directory",
    "重置缩放与平移": "Reset zoom and pan",
    "等待加载地图...": "Waiting for map…",
    "机器人: 未连接": "Robot: disconnected",
    "机器人：未连接": "Robot: disconnected",
    "需 ROS2 TF 桥；切图发布 RobotStatus（current_map），重定位发布 initial": "Requires the ROS 2 TF bridge. Map switching publishes RobotStatus (current_map); relocalization publishes initial.",
    "跳过朝向": "Skip heading",
    "清除选点": "Clear selection",
    "填入位姿": "Use current pose",
    "仅重定位": "Relocalize only",
    "仅切图": "Switch map only",
    "线速度": "Linear speed",
    "角速度": "Angular speed",
    "简易遥控": "Quick teleoperation",
    "基础控制参数配置": "Basic control parameters",
    "按机器人绑定、持久化并下发 Nav2 运行参数": "Bind, persist, and apply Nav2 runtime settings per robot",
    "目标机器人": "Target robot",
    "请选择机器人。配置不会再保存在当前浏览器中。": "Select a robot. Settings are no longer stored only in this browser.",
    "最大线速度": "Maximum linear speed",
    "最大角速度": "Maximum angular speed",
    "障碍膨胀半径": "Obstacle inflation radius",
    "应用到 DWB 的 max_vel_x 与 max_speed_xy。": "Applied to DWB max_vel_x and max_speed_xy.",
    "应用到路径跟踪与恢复旋转速度上限。": "Applied to path tracking and recovery rotation limits.",
    "同时应用到该机器人局部与全局代价地图。": "Applied to both local and global costmaps for this robot.",
    "保存并应用": "Save and apply",
    "离线机器人会在下次由本平台启动导航栈时自动加载。": "Offline robots load these settings the next time this platform starts their navigation stack.",
    "最大速度": "Maximum speed",
    "避障距离": "Obstacle clearance",
    "刷新间隔": "Refresh interval",
    "保存参数": "Save settings",
    "等待加载日志...": "Waiting for logs…",
    "刷新日志索引": "Refresh log index",
    "选择全部可删除的已归档 bag": "Select all deletable archived bags",
    "请先在左侧选择一个 bag。": "Select a bag on the left first.",
    "请先在左侧勾选一个或多个 bag。": "Select one or more bags on the left first.",
    "播放所选 bag": "Play selected bags",
    "删除所选 bag": "Delete selected bags",
    "未选择文件": "No files selected",
    "相关文件": "Related files",
    "日志操作": "Log actions",
    "bag 列表": "Bag list",
    "俯视相机方向控制": "Overhead camera direction controls",
    "俯视相机控制": "Overhead camera controls",
    "俯视相机说明": "About the overhead camera",
    "基础移动速度": "Base movement speed",
    "左上移动": "Move up-left",
    "右上移动": "Move up-right",
    "左下移动": "Move down-left",
    "右下移动": "Move down-right",
    "向上移动": "Move up",
    "向下移动": "Move down",
    "向左移动": "Move left",
    "向右移动": "Move right",
    "移动到该坐标": "Move to coordinates",
    "快速移动机器人": "Quick robot positioning",
    "Gazebo 模型名": "Gazebo model name",
    "移动速度": "Movement speed",
    "快速定位": "Quick locations",
    "状态: 未知": "Status: unknown",
    "机器人管理平台": "Robot Management Console",
    "机器人管理": "Robot Management",
    "机器人监控": "Robot Monitoring",
    "机器人状态": "Robot Status",
    "机器人详情": "Robot Details",
    "机器人": "Robot",
    "机器人 ID": "Robot ID",
    "新机器人 ID": "New robot ID",
    "连接状态": "Connection status",
    "参数设置": "Settings",
    "ROS 节点控制": "ROS Node Control",
    "ROS 节点": "ROS Nodes",
    "地图编辑": "Map Editor",
    "地图信息": "Map Information",
    "地图名": "Map name",
    "地图": "Map",
    "选择地图": "Select map",
    "楼层": "Floor",
    "保存地图": "Save map",
    "保存图层": "Save layers",
    "保存点位": "Save points",
    "重置视图": "Reset view",
    "显示网格": "Show grid",
    "图层工具": "Layer tools",
    "编辑层": "Edit layer",
    "栅格地图": "Occupancy map",
    "语义地图": "Semantic map",
    "画笔大小": "Brush size",
    "栅格颜色": "Grid color",
    "语义标签": "Semantic label",
    "撤销图层": "Undo layer",
    "点位设置": "Point settings",
    "点击取点": "Pick point",
    "取消取点": "Cancel picking",
    "撤销点位": "Undo point",
    "电梯等待点": "Elevator waiting point",
    "电梯内点": "Inside-elevator point",
    "重定位点": "Relocalization point",
    "自定义点位": "Custom point",
    "待机点": "Standby point",
    "障碍 · 黑色": "Obstacle · black",
    "空闲 · 白色": "Free · white",
    "未知 · 灰色": "Unknown · gray",
    "正在读取…": "Loading…",
    "等待加载…": "Waiting to load…",
    "离线 bag 回放": "Offline bag replay",
    "纯离线 · 不连接 ROS": "Fully offline · no ROS connection",
    "跟随 current_map": "Follow current_map",
    "等待地图…": "Waiting for map…",
    "正在只读解析 bag…": "Parsing bag read-only…",
    "录制相机画面": "Recorded camera views",
    "前视相机离线回放画面": "Offline front camera replay",
    "下视相机离线回放画面": "Offline downward camera replay",
    "bag 未记录前视图像": "Bag has no front camera images",
    "bag 未记录下视图像": "Bag has no downward camera images",
    "录制机器人状态": "Recorded robot state",
    "定位方式": "Localization method",
    "语义位置": "Semantic location",
    "当前 bag": "Current bag",
    "bag 信息": "Bag information",
    "回放进度": "Replay progress",
    "关闭回放": "Close replay",
    "等待图像…": "Waiting for image…",
    "前视相机": "Front camera",
    "下视相机": "Downward camera",
    "倍速": "Speed",
    "打开 OpenClaw 助手": "Open the OpenClaw assistant",
    "OpenClaw 页面助手": "OpenClaw page assistant",
    "OpenClaw 助手": "OpenClaw Assistant",
    "管理页 ↗": "Admin ↗",
    "关闭对话框": "Close dialog",
    "例如：robot1 当前状态怎么样？": "For example: What is robot1's current status?",
    "发送给 OpenClaw 的消息": "Message to OpenClaw",
    "正在执行中": "Working",
    "正在处理": "Processing",
    "正在加载": "Loading",
    "正在读取": "Loading",
    "正在保存": "Saving",
    "正在删除": "Deleting",
    "正在打包下载": "Preparing download",
    "正在记录": "Recording",
    "发送中…": "Sending…",
    "保存中…": "Saving…",
    "加载失败：": "Load failed: ",
    "保存失败：": "Save failed: ",
    "读取失败：": "Read failed: ",
    "删除失败：": "Delete failed: ",
    "记录失败：": "Record failed: ",
    "操作已完成": "Operation completed",
    "执行成功": "completed successfully",
    "无法连接 Web 后端，请检查 Web 栈": "Cannot connect to the web backend. Check the web stack.",
    "后端初始化失败: ": "Backend initialization failed: ",
    "状态: 无法连接后端": "Status: backend unavailable",
    "请求失败：": "Request failed: ",
    "请求失败: ": "Request failed: ",
    "请求失败 (": "Request failed (",
    "导航请求超时，请检查导航 action server": "Navigation request timed out. Check the navigation action server.",
    "没有可撤销的修改": "Nothing to undo",
    "已撤销上一步修改": "Undid the last change",
    "确定放弃当前地图的全部未保存修改吗？": "Discard all unsaved changes to the current map?",
    "已取消全部未保存修改": "Discarded all unsaved changes",
    "地图编辑仅支持已保存地图，请先选择地图": "The map editor only supports saved maps. Select a map first.",
    "未命名语义": "Unnamed semantic region",
    "背景/未标注": "Background / unlabelled",
    "障碍/墙体": "Obstacle / wall",
    "电梯区域": "Elevator area",
    "走廊": "Corridor",
    "暂停编辑": "Pause editing",
    "继续编辑": "Continue editing",
    "仅允许字母、数字、下划线与连字符": "Only letters, numbers, underscores, and hyphens are allowed",
    "该 ID 已在上方列表中，请用对应行的按钮": "This ID is already listed above. Use the button in its row.",
    "另一台机器人正在仿真上线中": "Another simulated robot is currently starting",
    "已下发启动脚本，等待 /…/robot_status": "Startup command sent; waiting for /…/robot_status",
    "已检测到 /…/robot_status，避免重复仿真上线": "Detected /…/robot_status; skipped duplicate simulation startup",
    "对应节点未发现": "Related node not found",
    "点位位置已填入，填写名称后保存": "Point position filled; enter a name and save",
    "目标点已设置，正在下发导航…": "Goal set; sending navigation request…",
    "已设置朝向，可下发重定位": "Heading set; relocalization can now be sent",
    "无可用位姿，请手填 x/y/yaw": "No pose available; enter x/y/yaw manually",
    "无效的建图楼层名": "Invalid mapping floor name",
    "后端返回的地图数据格式不正确": "The backend returned invalid map data",
    "请先在设置点位中填写名称和语义类型，再点击地图": "Enter a name and semantic type in Point settings before clicking the map",
    "点位已添加，点击保存地图修改后落盘": "Point added; save map changes to persist it",
    "达到分包阈值": "Bag size threshold reached",
    "录制停止": "Recording stopped",
    "录制器退出": "Recorder exited",
    "录制异常：缺少 robot_status": "Recording error: robot_status missing",
    "启动恢复": "Startup recovery",
    "恢复未完成": "Recovery incomplete",
    "旧版任务开始切包": "Legacy task started a new bag",
    "旧版任务结束切包": "Legacy task ended and rotated the bag",
    "旧版任务切换切包": "Legacy task transition rotated the bag",
    "无时间": "No timestamp",
    "所选 bag 没有关联文件。": "The selected bags have no related files.",
    "读取日志索引失败": "Failed to read log index",
    "下载失败": "Download failed",
    "下载已开始": "Download started",
    "删除失败": "Delete failed",
    "未记录 current_map": "current_map was not recorded",
    "当前 bag 段无图像": "No images in the current bag segment",
    "bag 未记录位姿": "Bag has no recorded pose",
    "bag 未记录速度": "Bag has no recorded velocity",
    "没有可用的已保存地图；可在上方切换地图": "No saved map is available; select a map above",
    "bag 未关联已保存地图": "Bag is not linked to a saved map",
    "含语义层": "Semantic layer available",
    "无语义层": "No semantic layer",
    "等待 bag 元数据…": "Waiting for bag metadata…",
    "bag 解析失败": "Failed to parse bag",
    "回放加载失败：": "Failed to load replay: ",
    "回放不可用": "Replay unavailable",
    "地图视图已重置": "Map view reset",
    "地图编辑器加载失败，请刷新页面后重试": "Map editor failed to load. Refresh the page and try again.",
    "修改仍保留，重新打开后可继续或保存": "Changes are retained; reopen to continue or save",
    "请在地图上依次设置位置和朝向": "Set the position and heading on the map",
    "请填写名称并在地图选点或填写 X/Y": "Enter a name and pick a map point or enter X/Y",
    "点位已加入，点击保存地图修改后落盘": "Point added; save map changes to persist it",
    "点位已移除，点击保存地图修改后落盘": "Point removed; save map changes to persist it",
    "地图修改已保存": "Map changes saved",
    "已清除地图选点": "Cleared the selected map point",
    "已下发到 ROS": "Sent to ROS",
    "正在记录当前位姿和激光帧…": "Recording the current pose and laser frame…",
    "保存失败: 请先填写地图名": "Save failed: enter a map name first",
    "正在调用 map_saver_cli 保存…": "Saving with map_saver_cli…",
    "未找到 topdown_camera 模型，请确认 world 中模型名": "topdown_camera model not found. Check its model name in the world.",
    "未连接 · 无 topdown 图像": "Disconnected · no topdown image",
    "桥未缓存帧（检查 Gazebo、ros_tf_bridge 与图像 topic）": "The bridge has no cached frame. Check Gazebo, ros_tf_bridge, and the image topic.",
    "无图像（桥未收到 topdown 相机 topic）": "No image (the bridge has not received the topdown camera topic)",
    "当前相机参数不足，无法从画面换算 world 坐标": "Camera parameters are insufficient to convert the image to world coordinates",
    "请填写 Gazebo 模型名（与 spawn -entity 一致，如 robot2）": "Enter the Gazebo model name used by spawn -entity, such as robot2",
    "请填写 x、y 坐标": "Enter x and y coordinates",
    "Gazebo 瞬移中…": "Moving Gazebo model…",
    "已切换到全图视角": "Switched to the full view",
    "俯视相机已复原": "Overhead camera restored",
    "接收导航目标": "Receive navigation goal",
    "行为树调度": "Behavior-tree scheduling",
    "全局路径规划": "Global path planning",
    "局部轨迹控制": "Local trajectory control",
    "恢复行为": "Recovery behavior",
    "到达目标 / 任务反馈": "Goal arrival / task feedback",
    "执行模块": "Execution module",
    "模块状态": "Module status",
    "个关联文件": "related files",
    "OpenClaw 未返回内容。": "OpenClaw returned no content.",
    "请确认 OpenClaw Gateway 已启动并可用。": "Make sure the OpenClaw Gateway is running and available.",
    "…结果已截断": "…result truncated",
    "请输入地图名称": "Enter a map name",
    "地图数据格式不正确": "Invalid map data format",
    "像素": "pixel",
    "暂无数据 · 点击展开": "No data · click to expand",
    "暂无数据": "No data",
    "无数据": "No data",
    "暂无机器人": "No robots",
    "暂无节点": "No nodes",
    "无机器人，暂无日志索引": "No robots or log indexes",
    "没有在线机器人可遥控": "No online robot available for teleoperation",
    "请先选择在线机器人": "Select an online robot first",
    "请先选择机器人": "Select a robot first",
    "请先填写机器人 ID": "Enter a robot ID first",
    "请先选择已保存地图": "Select a saved map first",
    "请先选择并加载地图": "Select and load a map first",
    "请先选择 bag": "Select a bag first",
    "请选择一个或多个可用的 bag": "Select one or more available bags",
    "请明确说明要执行的操作": "Clearly state the operation to perform",
    "仿真上线": "Start simulation",
    "仿真离线": "Stop simulation",
    "在线": "Online",
    "离线": "Offline",
    "已在线": "Online",
    "未连接": "Disconnected",
    "未运行": "Not running",
    "运行中": "Running",
    "正常": "Normal",
    "未知大小": "Unknown size",
    "未知机型": "Unknown model",
    "无地图": "No map",
    "未上报": "Not reported",
    "未选择": "Not selected",
    "未保存": "Unsaved",
    "录制中": "Recording",
    "当前任务": "Current task",
    "等待执行": "Waiting",
    "行为树": "Behavior tree",
    "待执行任务": "Pending tasks",
    "概览": "Overview",
    "日志": "Logs",
    "参数": "Parameters",
    "菜单": "Menu",
    "展开侧边栏": "Expand sidebar",
    "收起侧边栏": "Collapse sidebar",
    "监控": "Monitor",
    "名称": "Name",
    "类型": "Type",
    "尺寸": "Dimensions",
    "分辨率": "Resolution",
    "原点": "Origin",
    "点位": "Points",
    "X — · Y — · 像素 —": "X — · Y — · pixel —",
    "路径": "Path",
    "位姿": "Pose",
    "速度": "Velocity",
    "时间": "Time",
    "操作": "Actions",
    ".bag 与 tag": ".bag and tag",
    "bag 离线地图回放": "Offline bag map replay",
    "0.5 米网格": "0.5 m grid",
    "如 A座电梯": "e.g. Building A elevator",
    "全选": "Select all",
    "下载": "Download",
    "删除": "Delete",
    "重选": "Re-pick",
    "加载": "Load",
    "刷新": "Refresh",
    "发送": "Send",
    "保存为": "Save as",
    "工具": "Tool",
    "画笔": "Brush",
    "擦除": "Erase",
    "新增点位": "Add point",
    "移动点位": "Move point",
    "删除点位": "Delete point",
    "缩小": "Zoom out",
    "放大": "Zoom in",
    "复原": "Restore",
    "全图": "Full view",
    "区域": "Zone",
    "俯视相机": "Overhead camera",
    "上": "Up",
    "下": "Down",
    "左": "Left",
    "右": "Right",
    "播放": "Play",
    "暂停": "Pause",
    "建图中": "Mapping",
    "未选择已保存地图": "No saved map selected",
    "丢失": "Lost",
    "仿真": "Simulation",
    "已收到": "Received",
    "120s 内未收到 robot_status，已恢复按钮；请查看": "No robot_status was received within 120 seconds. The button was restored; check",
    "与 sim_bringup.sh 日志": "and the sim_bringup.sh logs",
    "机器人组": "Robot groups",
    "节点总数": "Total nodes",
    "（不可用）": "(unavailable)",
    "（该模块暂不开放）": "(not available for this module)",
    "lifecycle 不可用": "Lifecycle unavailable",
    "成功": "Succeeded",
    "ROS 调试操作失败": "ROS debug operation failed",
    "请填写机器人 ID，未下发导航目标": "Enter a robot ID; the navigation goal was not sent",
    "正在向": "Sending to",
    "下发导航目标": "Send navigation goal",
    "已提交给": "submitted to",
    "已定导航目标位置，再点击地图设置朝向（或「跳过朝向」）": "Navigation goal position set. Click the map to set its heading, or choose Skip heading.",
    "已定点位位置，再点击地图设置朝向（或「跳过朝向」）": "Point position set. Click the map to set its heading, or choose Skip heading.",
    "已定点，再点击地图设置朝向（或「跳过朝向」）": "Position set. Click the map to set its heading, or choose Skip heading.",
    "实时": "Live",
    "持久化": "Persisted",
    "robot_status 与 task_status": "robot_status and task_status",
    "持久化记录曾标记为仿真": "The persisted record was previously marked as a simulation",
    "上次仿真": "Previous simulation",
    "正在上线": "Starting simulation",
    "新机器人仿真上线失败": "Failed to start the new robot simulation",
    "仿真离线失败": "Failed to stop simulation",
    "仿真上线失败": "Failed to start simulation",
    "定位: 未选择地图": "Localization: no map selected",
    "无该机器人位姿": "No pose for this robot",
    "本图定位": "Localized on this map",
    "无实时数据": "No live data",
    "当前无机器人定位": "No robot is currently localized",
    "已填入": "Filled from",
    "建图视图": "Mapping view",
    "加载完成": "loaded",
    "加载失败:": "Load failed:",
    "等待": "Waiting for",
    "需向该 topic 发布 OccupancyGrid，且 ROS 桥运行中": "Publish OccupancyGrid to this topic and keep the ROS bridge running",
    "已绑定持久配置": "Persistent configuration bound",
    "当前使用系统默认值": "Using system defaults",
    "保存后重启仍然有效。": "The settings remain effective after restart.",
    "正在读取机器人配置…": "Loading robot settings…",
    "已从服务器读取该机器人配置。": "Loaded this robot's settings from the server.",
    "该机器人尚无单独配置，显示系统默认值。": "This robot has no dedicated settings; system defaults are shown.",
    "正在保存并应用到": "Saving and applying to",
    "参数保存结果": "Settings save result",
    "录制中 · 仅回放": "Recording · replay only",
    "无 log": "No logs",
    "为保证页面流畅，仅展示最新": "To keep the page responsive, only the latest",
    "全选仍会选择全部可删除 bag": "Select all still selects every deletable bag",
    "删除所选已归档 bag": "Delete selected archived bags",
    "录制中的 bag 不能删除": "An active recording cannot be deleted",
    "一次最多回放": "Replay at most",
    "按录制时间连续回放": "Replay continuously in recording-time order",
    "不启动 ROS": "without starting ROS",
    "未勾选下载文件": "No download files selected",
    "个 bag": "bags",
    "个文件": "files",
    "录制中不可下载": "active recording cannot be downloaded",
    "所选 bag 共关联": "The selected bags contain",
    "当前仅展示前": "Only the first",
    "正在读取机器人列表...": "Loading robot list…",
    "的归档和活动 bag...": "archived and active bags…",
    "已加载": "Loaded",
    "另有": "plus",
    "确定删除所选": "Delete the selected",
    "无其他 bag 引用的关联 txt 也会删除。": "Related txt files not referenced by another bag will also be deleted.",
    "正在删除所选 bag...": "Deleting selected bags…",
    "已删除": "Deleted",
    "条解析提示": "parse notices",
    "条消息": "messages",
    "个 DB": "databases",
    "激光": "laser",
    "图像": "images",
    "状态": "status",
    "记录地图": "Recorded map",
    "未保存，无法绘制": "is not saved and cannot be rendered",
    "正在从磁盘加载地图": "Loading map from disk",
    "个点位": "points",
    "按录制时间连续播放": "continuous playback in recording-time order",
    "正在只读解析": "Parsing read-only",
    "不会启动 ROS": "ROS will not be started",
    "单击地图设置点位位置，再单击设置朝向": "Click the map to set the point position, then click again to set its heading",
    "编辑已开启；Shift+拖拽仍可平移。": "Editing enabled; Shift-drag still pans.",
    "编辑已暂停，有未保存修改": "Editing paused with unsaved changes",
    "编辑已暂停": "Editing paused",
    "单击地图定位置，再单击定朝向": "Click the map to set position, then click again to set heading",
    "为 ": "For ",
    "选导航目标": "select a navigation goal",
    "建图模式下请填写「保存为」地图名（切图经 task_manager 准备后经 robot_status 下发）": "In mapping mode, enter a Save as map name. Map switching is prepared by task_manager and sent through robot_status.",
    "请填写「保存为」地图名或选择已保存楼层": "Enter a Save as map name or select a saved floor",
    "请填写 x、y（米），可用「填入位姿」": "Enter x and y in meters, or use Fill pose",
    "下发 localize_nav_command": "Send localize_nav_command",
    "已记录重定位点": "Recorded relocalization point",
    "当前地图": "current map",
    "已保存": "Saved",
    "地图已保存": "Map saved",
    "保存地图失败": "Failed to save map",
    "切换楼层到": "Switched floor to",
    "遥控失败": "Teleoperation failed",
    "已停车": "Stopped",
    "已填入：": "Filled: ",
    "相机 X": "Camera X",
    "高度": "height",
    "图片": "image",
    "已聚焦区域": "Focused zone",
    "读取 topdown_camera 位姿中…": "Loading topdown_camera pose…",
    "已读取 topdown_camera": "Loaded topdown_camera",
    "距上次收到帧": "Time since last frame",
    "历史画面 / 可能已离线": "historical frame / possibly offline",
    "帧偏旧": "stale frame",
    "新鲜": "fresh",
    "ROS 时间戳": "ROS timestamp",
    "超过 30s 未更新：仍显示最后一帧，取点可能不准；恢复 topic 后将自动回到高频刷新。": "No update for over 30s. The last frame remains visible and picking may be inaccurate; normal refresh resumes when the topic recovers.",
    "超过 10s 未更新：请确认仿真与相机插件仍在发布。": "No update for over 10s. Check that the simulation and camera plugin are still publishing.",
    "已从画面取点": "Picked from image",
    "可调整 yaw 后移动模型": "adjust yaw before moving the model",
    "Gazebo 已瞬移": "Gazebo model moved",
    "Gazebo 瞬移失败": "Failed to move Gazebo model",
    "切换全图失败": "Failed to switch to full view",
    "切换区域": "Switch zone",
    "失败": "Failed",
    "复原俯视相机中…": "Restoring overhead camera…",
    "复原俯视相机失败": "Failed to restore overhead camera",
    "是": "Yes",
    "否": "No",
    " 点": " points",
    "打开完整日志页面": "Open full logs page",
    "服务器持久配置": "Server-persisted settings",
    "系统默认值": "System defaults",
    "保存后，在线机器人立即应用；离线机器人由本平台下次启动导航栈时加载。": "Online robots apply settings immediately after saving; offline robots load them when this platform next starts their navigation stack.",
    "正在保存并下发…": "Saving and applying…",
    "你好，我可以查询机器人状态、地图、ROS 节点和日志，也能在确认后执行操作。": "Hello. I can query robot status, maps, ROS nodes, and logs, and perform confirmed operations.",
    "OpenClaw 正在处理，涉及操作时可能需要一些时间…": "OpenClaw is processing. Operations may take a little time…",
    "收到，正在执行中。": "Received. Execution is in progress.",
    "上线中": "starting",
    "执行失败": "Execution failed",
    "未知错误": "Unknown error",
    "已启用": "Enabled",
    "栅格": "occupancy",
    "语义": "semantic",
    "图层工具；点位操作已暂停": "layer tools; point editing paused",
    "语义区域": "Semantic region",
    "标签": "Label",
    "新点位": "New point",
    "已选中点位": "Selected point",
    "按住拖动可移动": "hold and drag to move it",
    "可在地图上按住拖动": "hold and drag it on the map",
    "已撤销上一步点位修改": "Undid the last point change",
    "已撤销上一步图层修改": "Undid the last layer change",
    "已取消图层修改": "Discarded layer changes",
    "已取消点位修改": "Discarded point changes",
    "已启用栅格图层工具；点位操作已暂停": "Occupancy-layer tools enabled; point editing paused",
    "已启用语义图层工具；点位操作已暂停": "Semantic-layer tools enabled; point editing paused",
    " 失败:": " failed:",
    "任务": "Task",
    "请填写机器人 ID": "Enter a robot ID",
    "完成": "completed",
    "建图": "Mapping",
    "正在连接": "Connecting to",
    "滚轮缩放 · 拖拽平移": "wheel to zoom · drag to pan",
    "保存失败 (": "Save failed (",
    "保存失败:": "Save failed:",
    "共 ": "Total ",
    "仍会选择全部可删除 bag。": "still selects every deletable bag.",
    " 个": " items",
    "共享 Gazebo/Xvfb 世界（不属于任何单台机器人）": "Shared Gazebo/Xvfb world (not owned by any single robot)",
    "假数据发布": "Mock data publisher",
    "检测到仿真机器人在线，可直接执行仿真离线": "A simulated robot is online and can be stopped directly",
    "已在线(非仿真)": "Online (physical robot)",
    "非仿真机器人已在线；此处仅用于仿真栈上下线": "A physical robot is online; these controls only start or stop the simulation stack",
    "配置已绑定并保存到该机器人；机器人离线，将在下次由本平台启动导航栈时加载": "Settings are bound and saved for this robot. It is offline, so they will load the next time this platform starts its navigation stack.",
    "部分 ROS 参数已应用；失败项将在下次由本平台启动导航栈时从持久配置加载": "Some ROS parameters were applied. Failed items will load from persistent settings the next time this platform starts the navigation stack.",
    "配置已保存；当前 ROS 导航节点未接受参数，将在下次由本平台启动导航栈时加载": "Settings were saved. The current ROS navigation nodes did not accept them, so they will load the next time this platform starts the navigation stack.",
    "配置已保存并应用到当前机器人导航栈": "Settings were saved and applied to the current robot navigation stack.",
    "仅支持包含 .db3 的 rosbag2 目录": "Only rosbag2 directories containing .db3 files are supported",
    "无法只读打开 rosbag2 数据库": "Could not open the rosbag2 database read-only",
    "文件不是有效的 rosbag2 SQLite 数据库": "The file is not a valid rosbag2 SQLite database",
    "读取 rosbag2 SQLite 失败": "Failed to read rosbag2 SQLite",
    "无法只读打开 rosbag2 图像数据库": "Could not open the rosbag2 image database read-only",
    "读取 rosbag2 图像失败": "Failed to read rosbag2 images",
    "bag 中没有可回放消息": "The bag contains no replayable messages",
    "至少需要一个可回放 bag": "At least one replayable bag is required",
    "点位类型": "Point type",
    "不唯一": "is ambiguous",
    "模型选择的点位不存在": "The point selected by the model does not exist",
    "未获取到": "Could not obtain",
    "的当前位置": "current position",
    "当前没有运行中的任务": "has no running task",
    "任务已停止": "Task stopped",
    "取货导航未返回任务编号": "Pickup navigation returned no task ID",
    "等待导航完成超时": "Timed out waiting for navigation to finish",
    "最后状态": "last status",
    "上线超时": "startup timed out",
    "仿真已上线": "simulation is online",
    "机器人仿真已下线": "Robot simulation is offline",
    "导航任务已下发": "Navigation task sent",
    "已下发前往": "Sent a task to go to",
    "并返回的任务": "and return",
    "查询成功": "Query succeeded",
    "已到达目标点": "Arrived at the target point",
    " 台": " robots",
    "个地图": "maps",
    "个地图点位": "map points",
    "上次 current_map": "Previous current_map",
    "机器人信息": "Robot information",
    "机型": "Model",
    "心跳": "Heartbeat",
    "生命周期": "Lifecycle",
    "导航": "Navigation",
    "运行状态": "Operating status",
    "定位": "Localization",
    "任务进度": "Task progress",
    "位置": "Location",
    "可视化数据": "Visualization data",
    "scan 坐标": "scan coordinates",
    "规划轨迹": "Planned path",
    "个该机器人节点": "nodes for this robot",
    "资源": "Resources",
    "个关联进程": "related processes",
    "当前没有任务状态。": "No task status is currently available.",
    "暂无待执行任务。": "No pending tasks.",
    "汇总状态": "Overall status",
    "进度": "Progress",
    "当前项": "Current item",
    "命令": "Command",
    "节点": "Node",
    "保存并应用到 ": "Save and apply to ",
    "保存并应用到": "Save and apply",
    "，同时删除": ", also deleted",
    "个孤立 txt": "orphan txt files",
    "语言": "Language",
    "中文": "中文"
  };

  const replacements = Object.entries(ENGLISH_PHRASES).sort((a, b) => b[0].length - a[0].length);
  const textSources = new WeakMap();
  const textRendered = new WeakMap();
  const attributeSources = new WeakMap();
  const titleSource = document.title;
  let locale = readLocale();
  let observer = null;

  function readLocale() {
    try {
      const saved = localStorage.getItem(STORAGE_KEY);
      return SUPPORTED_LOCALES.has(saved) ? saved : DEFAULT_LOCALE;
    } catch {
      return DEFAULT_LOCALE;
    }
  }

  function translate(source) {
    const value = String(source ?? "");
    if (locale !== "en" || !/[\u3400-\u9fff]/.test(value)) return value;
    const translated = replacements.reduce((result, [zh, en]) => {
      // Single-character labels such as 上/下 are valid button text, but a
      // substring replacement would corrupt longer Chinese sentences.
      if (zh.length === 1) {
        const match = result.match(/^(\s*)([^\s])(\s*)$/s);
        return match && match[2] === zh ? match[1] + en + match[3] : result;
      }
      return result.split(zh).join(en);
    }, value);
    return translated
      .replace(/；\s*/g, "; ")
      .replace(/：\s*/g, ": ")
      .replace(/，\s*/g, ", ")
      .replace(/。/g, ".")
      .replace(/？/g, "?")
      .replace(/！/g, "!")
      .replace(/（/g, "(")
      .replace(/）/g, ")")
      .replace(/「/g, '"')
      .replace(/」/g, '"')
      .replace(/“/g, '"')
      .replace(/”(?=\S)/g, '" ')
      .replace(/”/g, '"')
      .replace(/……/g, "…")
      .replace(/([.!?])"(?=\S)/g, '$1 "');
  }

  function shouldIgnore(node) {
    const parent = node.nodeType === Node.ELEMENT_NODE ? node : node.parentElement;
    return !parent || Boolean(parent.closest("script, style, [data-i18n-ignore]"));
  }

  function translateTextNode(node) {
    if (shouldIgnore(node)) return;
    const current = node.nodeValue || "";
    const lastRendered = textRendered.get(node);
    if (!textSources.has(node) || (lastRendered !== undefined && current !== lastRendered)) {
      textSources.set(node, current);
    }
    const source = textSources.get(node);
    const next = locale === DEFAULT_LOCALE ? source : translate(source);
    textRendered.set(node, next);
    if (current !== next) node.nodeValue = next;
  }

  function translateElementAttributes(element) {
    if (shouldIgnore(element)) return;
    let sources = attributeSources.get(element);
    if (!sources) {
      sources = new Map();
      attributeSources.set(element, sources);
    }
    TRANSLATABLE_ATTRIBUTES.forEach((name) => {
      if (!element.hasAttribute(name)) return;
      const current = element.getAttribute(name) || "";
      const previous = sources.get(name);
      if (!previous || current !== previous.rendered) sources.set(name, { source: current, rendered: current });
      const item = sources.get(name);
      const next = locale === DEFAULT_LOCALE ? item.source : translate(item.source);
      item.rendered = next;
      if (current !== next) element.setAttribute(name, next);
    });
  }

  function translateTree(root) {
    if (!root) return;
    if (root.nodeType === Node.TEXT_NODE) {
      translateTextNode(root);
      return;
    }
    if (root.nodeType !== Node.ELEMENT_NODE && root.nodeType !== Node.DOCUMENT_NODE) return;
    if (root.nodeType === Node.ELEMENT_NODE) translateElementAttributes(root);
    const walker = document.createTreeWalker(root, NodeFilter.SHOW_ELEMENT | NodeFilter.SHOW_TEXT);
    let node = walker.nextNode();
    while (node) {
      if (node.nodeType === Node.TEXT_NODE) translateTextNode(node);
      else translateElementAttributes(node);
      node = walker.nextNode();
    }
  }

  function apply() {
    document.documentElement.lang = locale;
    document.title = locale === DEFAULT_LOCALE ? titleSource : translate(titleSource);
    translateTree(document.body);
    const selector = document.getElementById("language-select");
    if (selector) selector.value = locale;
  }

  function setLocale(nextLocale) {
    if (!SUPPORTED_LOCALES.has(nextLocale) || nextLocale === locale) return;
    locale = nextLocale;
    try {
      localStorage.setItem(STORAGE_KEY, locale);
    } catch {
      // The selection still applies for the current page when storage is unavailable.
    }
    apply();
    document.dispatchEvent(new CustomEvent("openDelivery:languagechange", { detail: { locale } }));
    window.dispatchEvent(new Event("resize"));
  }

  function observe() {
    observer = new MutationObserver((mutations) => {
      mutations.forEach((mutation) => {
        if (mutation.type === "characterData") translateTextNode(mutation.target);
        else if (mutation.type === "attributes") translateElementAttributes(mutation.target);
        else mutation.addedNodes.forEach(translateTree);
      });
    });
    observer.observe(document.body, {
      subtree: true,
      childList: true,
      characterData: true,
      attributes: true,
      attributeFilter: TRANSLATABLE_ATTRIBUTES
    });
  }

  function wrapCanvasText(methodName) {
    const prototype = window.CanvasRenderingContext2D && window.CanvasRenderingContext2D.prototype;
    if (!prototype || prototype[methodName].__openDeliveryI18n) return;
    const original = prototype[methodName];
    const localized = function (text, ...args) {
      return original.call(this, translate(text), ...args);
    };
    localized.__openDeliveryI18n = true;
    prototype[methodName] = localized;
  }

  const nativeAlert = window.alert.bind(window);
  const nativeConfirm = window.confirm.bind(window);
  window.alert = (message) => nativeAlert(translate(message));
  window.confirm = (message) => nativeConfirm(translate(message));
  wrapCanvasText("fillText");
  wrapCanvasText("strokeText");

  window.OpenDeliveryI18n = {
    get locale() { return locale; },
    setLocale,
    t: (source) => translate(source),
    apply
  };

  function init() {
    const selector = document.getElementById("language-select");
    if (selector) selector.addEventListener("change", (event) => setLocale(event.target.value));
    apply();
    observe();
  }

  if (document.readyState === "loading") document.addEventListener("DOMContentLoaded", init, { once: true });
  else init();
})();
