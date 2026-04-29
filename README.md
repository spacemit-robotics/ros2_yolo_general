# yolo_general

## 项目简介

ROS2 通用 YOLOv8 检测节点，支持 COCO 80 类目标检测。基于 `vision_service.h`，使用 `model_zoo/vision` 的 `YOLOv8Detector` 实现通用目标检测功能。

## 功能特性

- 支持 COCO 80 类目标检测
- 基于 YOLOv8 模型
- 输出标准 Detection2DArray 格式
- 可视化调试图像
- 不支持：实例分割、关键点检测

## 快速开始

### 环境准备

- ROS2 Humble 或更高版本
- 已编译的 `components/model_zoo/vision` 组件
- YOLOv8 模型文件

### 构建编译

```bash
colcon build --packages-select yolo_general
source install/setup.bash
```

### 运行示例

```bash
ros2 launch yolo_general yolo_general.launch.py
```

## 详细使用


### 依赖

- `components/model_zoo/vision`：提供 `libvision.so` 与 `vision_service.h`
- YOLOv8 模型与对应配置文件

### 话题

| 类型 | 话题（默认） | 说明 |
|------|--------------|------|
| 订阅 | `/camera/image_raw` | 输入图像 |
| 发布 | `/perception/detections` | vision_msgs/Detection2DArray（class_id 为 label 数字字符串） |
| 发布 | `/yolo_general/boxes` | Float32MultiArray，每目标 7 个数：x1,y1,x2,y2,score,label,track_id |
| 发布 | `/yolo_general/debug_image` | 可视化图 |

### 配置

- 默认配置为 `config/yolov8.yaml`
- 需先构建 `model_zoo/vision` 并准备对应 `yolov8n` 模型


## 常见问题

- 若无检测结果，先确认模型路径与输入图像话题是否正常
- 若需要标准检测消息，确认当前环境已提供 `vision_msgs`

## 版本与发布

当前版本：1.0.0

变更记录：
- 初始版本发布

## 贡献方式

欢迎提交 Issue 和 Pull Request。

贡献者与维护者名单见：`CONTRIBUTORS.md`（如有）

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
