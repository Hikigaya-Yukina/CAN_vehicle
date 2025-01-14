## 设置波特率

```bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
```

设置虚拟can
```bash
sudo ip link add dev can0 type vcan
sudo ip link set up can0
ip addr show can0

```

记录can数据
```bash
candump -l can0 | tee can_data.log
```

播放can数据
```bash
canplayer -I <log文件名>
canplayer -I my_can_data.log
#canplayer 有几个有用的选项，可以帮助你更好地控制回放过程：
# -l 或 --loop：使日志文件循环播放。
# -g 或 --gap：设置两个消息之间的时间间隔（单位为秒），如 -g 0.1 会在每个消息间增加0.1秒的延迟。
# -v 或 --verbose：提供更详细的输出信息。
canplayer -I my_can_data.log -l -g 0.1
```
滤波部分CAN信息
```bash
candump can0,123:7FF,456:7FF
#在这个例子中，掩码 7FF 指的是在比较时会考虑所有位（对于标准帧）。
#因为标准 CAN ID 的长度是 11 位，所以 7FF 就是 11 位都为 1 的二进制数，确保只有完全匹配 123 和 456 的 ID 才会被接收。
```