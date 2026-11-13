# Android 视频运行器

[English](README.md) | 简体中文

使用 Still Flow AAR 在 Android 设备上处理视频，并与参考视频比较结果。

## 运行条件

需要 Linux 或 WSL、JDK、Android SDK/NDK、CMake、`adb`、FFmpeg 和 `uv`。将 `ANDROID_HOME` 指向 SDK 目录，连接已授权且支持 `GL_EXT_YUV_target` 的 Android 10+ arm64 设备。

输入须为逐行扫描、偶数尺寸、方形像素的 8 位 H.264 视频，并带有 SDR BT.601/BT.709 色彩元数据。处理前将旋转应用到像素。输出仅含视频，保留帧时间戳。

## 使用

在仓库根目录运行：

```sh
uv run --locked --group video python scripts/android.py input.mp4
uv run --locked --group video python scripts/android.py input.mp4 --realtime --mobile
uv run --locked --group video python scripts/android.py input.mp4 --serial DEVICE --reference host.mp4 --max-displacement 0.5
uv run --locked --group video python scripts/android.py --help
```

默认离线处理。`--realtime` 启用固定前视流式处理，`--mobile` 使用较轻量的分析配置。默认编码码率为 12 Mbps。连接多台设备时使用 `--serial`，使用已有 AAR 时指定 `--aar`。

每次运行将视频和报告保存在 `.local/android-runs/` 下，使用 `--output` 指定新目录。运行器会构建并安装应用；处理完成后清理设备临时文件，保留已安装的应用。

## 结果对比

`--reference` 比较帧时间和图像几何，对比预览左侧为参考视频，右侧为设备输出。`--max-displacement` 要求每帧具有足够的跟踪特征，且位移的第 95 百分位不超过指定像素阈值；失败时返回非零退出状态。省略阈值时仅报告测量结果。
