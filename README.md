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