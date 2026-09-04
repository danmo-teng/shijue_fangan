# 将训练模型部署到RDK X5

`/home/sunrise/RDK_X5/yolo.zip`中的`best.pt`是四类别YOLOv8s模型，类别顺序为：

```text
conmon
kernel
risk
wound
```

其中`conmon`是训练标签中已经存在的拼写，板端脚本会将其映射为`green_supply`。

压缩包自带的`convert_rknn.py`、`model.rknn`路径和`run_steelball.py`面向Orange Pi 5B/RK3588，
不能用于地平线RDK X5。X5需要Bayes-e架构的BPU `.bin`模型。

## 在训练电脑上转换

转换必须在x86 Ubuntu 22.04、Python 3.10环境中进行，不要在RDK X5板端运行编译工具链。
以下流程以D-Robotics官方`rdk_model_zoo`的`rdk_x5`分支为准：

```bash
unzip yolo.zip
git clone -b rdk_x5 https://github.com/D-Robotics/rdk_model_zoo.git
cd rdk_model_zoo/samples/vision/ultralytics_yolo/conversion

conda create -n rdkx5-yolo python=3.10 -y
conda activate rdkx5-yolo
pip install -r requirements.txt
pip install rdkx5-yolo-mapper

python3 export_monkey_patch.py --pt /你的路径/rdkx5/best.pt
python3 mapper.py \
  --onnx /你的路径/rdkx5/best.onnx \
  --cal-images /你的路径/rdkx5/calibration_images \
  --output-dir /你的路径/rdkx5/x5_output
```

### 320x320高速模型

如果不要求远距离小目标，并希望提高实时帧率，把本项目的`export_yolo_x5_320.py`复制到
D-Robotics官方`conversion/`目录（与`export_monkey_patch.py`放在一起），然后执行：

```bash
python3 export_yolo_x5_320.py --pt /你的路径/rdkx5/best.pt --imgsz 320
python3 mapper.py \
  --onnx /你的路径/rdkx5/best.onnx \
  --cal-images /你的路径/rdkx5/calibration_images \
  --output-dir /你的路径/rdkx5/x5_output_320
```

用`hrt_model_exec model_info`确认生成模型输入为`1x3x320x320`、输出为三组4通道分类和
三组64通道边框张量。板端`run_yolo_x5.py`会自动读取320输入，不需要修改后处理代码。

若`export_monkey_patch.py`输出的实际文件名不同，以终端输出和生成的`.onnx`文件为准。
不要优先使用压缩包中针对RKNN修改过的`model.onnx`；从`best.pt`重新执行官方X5导出可确保输出协议与
X5运行时一致。

生成后先检查模型：

```bash
hb_model_info /你的路径/rdkx5/x5_output/*.bin
hrt_model_exec perf --model_file /你的路径/rdkx5/x5_output/*.bin --thread_num 1
```

## 复制到RDK X5

在RDK上准备目录：

```bash
mkdir -p /home/sunrise/RDK_X5/shijue_fangan/vision/models
```

在电脑上复制并统一命名：

```bash
scp /你的路径/rdkx5/x5_output/*.bin \
  sunrise@RDK_IP:/home/sunrise/RDK_X5/shijue_fangan/vision/models/best_bayese_320x320_nv12.bin
```

## 板端运行

先关闭占用`/dev/video0`的Web编辑器或其他视觉程序，然后执行：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan/vision
export DISPLAY=:0
python3 run_yolo_x5.py --device /dev/video0
```

无窗口性能测试：

```bash
python3 run_yolo_x5.py --device /dev/video0 --no-display --duration 10
```

结果写入`runtime_result.json`，检测框和中心点均已映射为原始1280x1024相机坐标。

官方转换说明：

- <https://github.com/D-Robotics/rdk_model_zoo/blob/rdk_x5/samples/vision/ultralytics_yolo/conversion/README_cn.md>
