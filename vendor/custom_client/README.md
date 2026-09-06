# 自定义客户端（车端编码 + 操作间接收）

从 `192.168.8.1` 的 `jlauto-aim` 拷入，**不改业务逻辑**。

| 目录 | 作用 |
| --- | --- |
| `linuxsender1/` | 车端：海康 → H264 chunk → 串口 `0x0310` |
| `linuxideo/` | 操作间：MQTT 收包、Qt 查看器、FSRCNN 超分菜单 |

**不包含** `linuxideo/sr/`（超分 Python venv，约 7.2G）。超分训练仍在 8.1；查看器缺 `models/FSRCNN_*.pb` 时跑 `linuxideo/models/download_models.py`。

## 车端

```bash
cd vendor/custom_client/linuxsender1
# 按该包 README / docs 编译与 launch（依赖车上海康与串口）
```

## 操作间

```bash
cd vendor/custom_client/linuxideo
python3 mqtt_video_receiver_v4.py
# 或
python3 -m doorlock_viewer
```

MQTT 默认见源码配置（常为 `192.168.12.1:3333`）。
