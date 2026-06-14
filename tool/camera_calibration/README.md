# Camera Calibration Tool

这个目录提供两部分：

- `calibrate_camera.py`
  - 真正执行 OpenCV 标定
  - 输入棋盘格照片
  - 输出 JSON、代码片段、角点图、去畸变对比图
- `index.html`
  - 离线前端辅助页
  - 生成命令
  - 读取 `calibration_result.json`
  - 判断误差大致是否合理
  - 生成抄回项目的代码片段

## 目录建议

可以按下面放：

```text
tool/camera_calibration/
  calibrate_camera.py
  index.html
  README.md
  photos/
    001.jpg
    002.jpg
    ...
  output/
```

## 依赖

本机 Python 环境需要：

```powershell
pip install opencv-python numpy
```

## 拍照要求

- 用外部板子的摄像头拍，不用 VEX
- 分辨率、裁剪、去畸变开关、对焦模式要和实战一致
- 推荐 20 到 40 张
- 棋盘格要覆盖中间和四角
- 要有近有远，有平视也有倾斜
- 不要模糊，不要强反光
- 不建议投影棋盘格做正式标定

## 棋盘格说明

脚本参数里的：

- `pattern-cols`
  - 每行内角点数量
- `pattern-rows`
  - 每列内角点数量

注意是“内角点”，不是黑白格数量。

例子：

- 一个 `10 x 7` 的方格棋盘
- 它的内角点就是 `9 x 6`

## 运行示例

在这个目录下运行：

```powershell
python calibrate_camera.py ^
  --images "photos" ^
  --pattern-cols 9 ^
  --pattern-rows 6 ^
  --square-size-mm 20 ^
  --output-dir "output" ^
  --preview-limit 12
```

`--images` 现在支持三种写法：

- 目录：`photos`
- 通配符：`photos/*.bmp`
- 单文件：`photos/001.bmp`

支持的图片格式：

- `bmp`
- `jpg`
- `jpeg`
- `png`
- `tif`
- `tiff`

## 输出内容

运行后会生成：

- `output/calibration_result.json`
  - 完整结果
- `output/repo_camera_model.json`
  - 只保留项目里要填的字段
- `output/repo_snippet.txt`
  - 直接抄回 C++ 的代码片段
- `output/corners/*.jpg`
  - 角点检测效果图
- `output/undistort_previews/*.jpg`
  - 原图和去畸变对比图

## 怎么判断结果大致可用

- `RMS < 0.5 px`
  - 通常很好
- `0.5 ~ 1.0 px`
  - 常常能用，但要看预览图
- `> 1.0 px`
  - 一般建议重拍

还要看：

- 边缘直线是否更直
- 四角是否改善明显
- 已知距离下的测距是否更准

## 抄回项目

当前项目默认填在：

- `src/hardware/football_robot/football_robot.cpp`
- `default_vision_config_for_sensor()`

可以直接使用 `output/repo_snippet.txt` 里的内容。

## 重要注意

如果外部板子在检测前已经做了去畸变：

- 板子发回来的框坐标就是“去畸变后图像坐标”
- 那 VEX 侧不要再按畸变参数去畸变一次

如果外部板子做了：

- crop
- letterbox
- 非等比例缩放

那就要确认发回来的框坐标属于哪个图像坐标系，再决定用哪一套内参，不能直接生搬。
