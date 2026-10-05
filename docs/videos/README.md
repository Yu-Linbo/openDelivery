# OpenDelivery demo recording

**English** | 简体中文 below

[Watch the demo](opendelivery-autonomous-delivery.mp4).

[Preview the scenes](opendelivery-autonomous-delivery.contact-sheet.png).

The English recording starts with a short conversational request:

> Bring robot2 online and get my delivery from floor 1 to floor 4.

Robot2 actually starts in simulation on floor 1, visits the reception pickup point, takes the simulated elevator and arrives at the floor-4 elevator waiting point. The OpenClaw job completed successfully: startup and both navigation steps finished. Pickup and delivery are simulated navigation stops.

The console tour includes the status popup, monitoring map, robot details/tasks/tree/resources/logs/parameters, Gazebo, ROS nodes, Settings and the map editor. It returns to the live OpenClaw replies twice during the tour. Afterward, the entire conversation is scrolled through and the complete final feedback stays visible before bag playback begins. All 24 stored conversation messages were checked against the recorded panel.

The ending briefly shows the real TXT log for five seconds, then collapses it to enlarge the map. The complete mission bag plays at 2× from zero to 140.521147 seconds without seeking or removing any timeline section. The bag contains both successful navigation task IDs and the floor-1 to floor-4 transition. The UI and replay footage come from the same successful mission; chapter editing removes only an automation pause before replay.

The replay ending uses the corrected trajectory renderer: floor changes, localization resets and recording gaps start a new trail, and the view fits the current floor map. Six real-bag browser checkpoints verify that the former 28.34 m connector is absent.

[Recording verification](delivery-replay-proof.json) includes the real request, replies, final feedback, successful task IDs, TXT visibility and complete playback observations.

## 简体中文

[播放演示视频](opendelivery-autonomous-delivery.mp4)。默认英文界面，输入简短口语：“Bring robot2 online and get my delivery from floor 1 to floor 4.”

本轮真实完成了一楼上线、前台取货、乘坐仿真电梯及四楼送达。页面展示期间两次回看 OpenClaw 回复；展示结束后滚动查看完整对话，停留显示最终成功反馈，再开始 bag 回放。录制面板与保存的 24 条会话消息全部核对一致。

TXT 日志只展开约 5 秒后收起，留出更大地图空间。最后以 2 倍速从零连续播放本轮任务的整个 bag，完整时间线为 140.521147 秒，包含两段成功导航和一楼到四楼的切换，不跳段。页面巡览与回放均来自同一次成功任务；剪辑只去掉回放前的一段录制脚本等待。取送货以导航停靠模拟，电梯也是仿真。

回放结尾已使用修复后的轨迹绘制重新录制：切楼层、定位坐标突变及录制间隔会断开轨迹，视野按当前地图适配。真实 bag 的六个浏览器检查点确认原来的 28.34 米异常长线已消失。

[场景预览](opendelivery-autonomous-delivery.contact-sheet.png) · [录制验证](delivery-replay-proof.json)，包含真实请求、回复、最终反馈、成功任务 ID、TXT 展示和完整回放记录。
