import os
import libusb_package

# 预加载 libusb
lib_path = str(libusb_package.get_library_path())
os.environ["PATH"] = os.path.dirname(lib_path) + os.path.pathsep + os.environ.get("PATH", "")
os.environ["PYUSB_LIBUSB_1_0"] = lib_path

import can

print("正在监听 CAN 总线数据 (500k)... 按 Ctrl+C 退出")
try:
    # 建立 CAN 连接
    bus = can.ThreadSafeBus(interface="gs_usb", channel=0, bitrate=500000)
    while True:
        # 等待接收数据，超时 1 秒
        msg = bus.recv(timeout=1.0)
        if msg is not None:
            print(f"✅ 收到底盘数据! ID: {hex(msg.arbitration_id)} Data: {msg.data.hex()}")
        else:
            print("⏳ 正在等待数据...（未收到任何底盘报文）")
except Exception as e:
    print(f"❌ 监听出错: {e}")
