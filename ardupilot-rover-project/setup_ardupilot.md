# 🚗 Hướng dẫn Setup ArduPilot Rover từ đầu

## 📋 Yêu cầu hệ thống
- Ubuntu 20.04/22.04 (hoặc WSL2 trên Windows)
- Python 3.8+
- Git

## 🔧 1. Cài đặt ArduPilot SITL

### Bước 1.1: Cài đặt dependencies
```bash
sudo apt-get update
sudo apt-get install -y git python3-pip python3-dev python3-opencv python3-wxgtk4.0 python3-matplotlib python3-lxml python3-pygame
```

### Bước 1.2: Clone ArduPilot
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

### Bước 1.3: Cài đặt build tools
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### Bước 1.4: Reload environment
```bash
. ~/.profile
# Hoặc khởi động lại terminal
```

### Bước 1.5: Build ArduRover
```bash
cd ~/ardupilot
./waf configure --board sitl
./waf rover
```

## 🐍 2. Cài đặt Python Virtual Environment

### Bước 2.1: Tạo virtual environment
```bash
# Tạo thư mục cho dự án
mkdir -p ~/ardupilot-rover-project
cd ~/ardupilot-rover-project

# Tạo virtual environment
python3 -m venv venv

# Kích hoạt virtual environment
source venv/bin/activate

# Nâng cấp pip
pip install --upgrade pip
```

### Bước 2.2: Cài đặt các packages cần thiết
```bash
# Đảm bảo venv đã được activate (xem $ (venv) ... )
pip install pymavlink websockets asyncio

# (Optional) Xác minh cài đặt
pip list
```

**Output mong đợi:**
```
Package         Version
----------      -------
pymavlink       2.4.x
websockets      10.x+
asyncio         ...
```

### Bước 2.3: Deactivate virtual environment
```bash
# Để thoát khỏi venv (khi không dùng)
deactivate
```

**Lưu ý quan trọng:**
- ✅ **Luôn activate venv** trước khi chạy code: `source venv/bin/activate`
- ✅ **Dòng `(venv) # 🚗 Hướng dẫn Setup ArduPilot Rover từ đầu

## 📋 Yêu cầu hệ thống
- Ubuntu 20.04/22.04 (hoặc WSL2 trên Windows)
- Python 3.8+
- Git

## 🔧 1. Cài đặt ArduPilot SITL

### Bước 1.1: Cài đặt dependencies
```bash
sudo apt-get update
sudo apt-get install -y git python3-pip python3-dev python3-opencv python3-wxgtk4.0 python3-matplotlib python3-lxml python3-pygame
```

### Bước 1.2: Clone ArduPilot
```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

### Bước 1.3: Cài đặt build tools
```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

### Bước 1.4: Reload environment
```bash
. ~/.profile
# Hoặc khởi động lại terminal
```

### Bước 1.5: Build ArduRover
```bash
cd ~/ardupilot
./waf configure --board sitl
./waf rover
```

 sẽ xuất hiện** khi venv đang active
- ✅ **Nếu quên activate** sẽ bị lỗi "No module named 'pymavlink'"

## 🚀 3. Cấu trúc thư mục dự án

```
~/ardupilot-rover-project/
├── venv/                    # Virtual environment
│   ├── bin/
│   ├── lib/
│   └── ...
├── websocket_bridge.py      # Bridge MAVLink ↔ WebSocket
├── mission_planner.html     # Web interface
└── README.md
```

## ▶️ 4. Chạy hệ thống

### ⚠️ Chuẩn bị trước khi chạy

```bash
# Tại thư mục ~/ardupilot-rover-project
cd ~/ardupilot-rover-project

# Kích hoạt virtual environment
source venv/bin/activate

# Xác nhận venv đã activate (phải thấy (venv) ở đầu dòng)
# (venv) user@machine:~/ardupilot-rover-project$
```

### Terminal 1️⃣: Khởi động SITL ArduRover
```bash
cd ~/ardupilot/Tools/autotest
python3 sim_vehicle.py -v Rover --console --map
```

**Các tham số hữu ích:**
- `--console`: Mở console MAVProxy
- `--map`: Mở bản đồ
- `-L <location>`: Chọn vị trí khởi động (ví dụ: `-L CMAC` cho Canberra)
- `--speedup <N>`: Tăng tốc simulation (ví dụ: `--speedup 2`)

**Lưu ý:** MAVProxy sẽ listen trên port TCP 5763 để kết nối từ các client

### Terminal 2️⃣: Chạy WebSocket Bridge

```bash
# ⚠️ ĐẦU TIÊN: Kích hoạt virtual environment
cd ~/ardupilot-rover-project
source venv/bin/activate

# Sau đó chạy bridge
python websocket_bridge.py --mavlink tcp:127.0.0.1:5763 --port 5760
```

**Giải thích:**
- `--mavlink tcp:127.0.0.1:5763`: Kết nối đến MAVProxy (không phải SITL trực tiếp)
- `--port 5760`: WebSocket server chạy trên port 5760

**Output mong đợi:**
```
============================================================
MAVLink WebSocket Bridge - FIXED VERSION
============================================================
MAVLink: tcp:127.0.0.1:5763
WebSocket Port: 5760
============================================================

🔧 FIX Applied:
  ✓ Waypoint 0 được set là current (seq=0)
  ✓ Không chuyển sang GUIDED trước khi AUTO
  ✓ Auto thêm vị trí hiện tại làm waypoint đầu tiên
  ✓ Verify mission chạy từ waypoint 0

============================================================

✓ MAVLink kết nối thành công!
  System ID: 1
  Component ID: 1
✓ WebSocket server đang chạy tại ws://localhost:5760
  Kết nối từ trình duyệt: ws://localhost:5760

============================================================

Đang chờ client kết nối...
```

### Terminal 3️⃣: Mở Web Interface

1. **Lưu file `mission_planner.html`** vào máy tính
2. **Mở browser** (Chrome, Firefox, Edge, etc.)
3. **Mở file** `mission_planner.html` (Ctrl+O hoặc File → Open)
4. **Nhập WebSocket URL:** `ws://localhost:5760`
5. **Nhấn "Kết nối"**

## 📊 5. Sử dụng Web Mission Planner

### 🎯 Quy trình sử dụng:

```
1. Kết nối WebSocket
   ↓
2. Xem Telemetry (vị trí, tốc độ, battery, GPS)
   ↓
3. Click trên bản đồ để tạo waypoints
   ↓
4. ARM xe
   ↓
5. Bắt đầu Mission
   ↓
6. Giám sát mission trên bản đồ
```

### 🖱️ Các thao tác trên bản đồ:

- **Click để thêm waypoint**: Click bất kỳ vị trí nào trên bản đồ
- **Kéo waypoint**: Di chuyển marker để thay đổi vị trí
- **Xóa waypoint**: Nhấn nút "Xóa" bên cạnh waypoint

### 🎮 Các lệnh điều khiển:

| Nút | Chức năng |
|-----|----------|
| **Kết nối** | Kết nối đến WebSocket bridge |
| **ARM** | Arm vehicle (chuẩn bị di chuyển) |
| **DISARM** | Disarm vehicle (tắt động cơ) |
| **GUIDED** | Chuyển sang GUIDED mode |
| **AUTO** | Chuyển sang AUTO mode (chạy mission) |
| **HOLD** | Dừng lại |
| **RTL** | Quay về Home |
| **Bắt đầu Mission** | Upload waypoints và bắt đầu |
| **Xóa tất cả** | Xóa tất cả waypoints |
| **Tải lên Mission** | Upload waypoints mà không bắt đầu |

### 📈 Telemetry Display:

- **Flight Mode**: Chế độ bay/di chuyển hiện tại
- **Latitude/Longitude**: Tọa độ GPS
- **Altitude**: Độ cao (m)
- **Speed**: Tốc độ (m/s)
- **Heading**: Hướng đi (°)
- **Battery**: Mức pin (%)

## 🐛 6. Troubleshooting

### ❌ Lỗi: "Connection refused" hoặc "WebSocket không kết nối"

**Kiểm tra:**
1. Terminal 1 (SITL) có chạy không?
2. Terminal 2 (Bridge) có hiển thị "Đang chờ client kết nối..."?
3. Port 5760 có bị chiếm không?

**Giải pháp:**
```bash
# Kiểm tra port
lsof -i :5760

# Nếu bị chiếm, kill process
kill -9 <PID>

# Hoặc dùng port khác
python3 websocket_bridge.py --mavlink tcp:127.0.0.1:5763 --port 5761
```

### ❌ Lỗi: "MAVLink kết nối thất bại"

**Kiểm tra:**
1. SITL có chạy không? (xem Terminal 1)
2. Kết nối đúng là `tcp:127.0.0.1:5763` chưa?

**Giải pháp:**
```bash
# Trong MAVProxy console (Terminal 1):
help output

# Kiểm tra output connections
```

### ❌ Xe không di chuyển sau khi bắt đầu Mission

**Kiểm tra:**
1. Xe đã ARM chưa?
2. GPS có fix không? (xem Telemetry)
3. Có 2 waypoints trên bản đồ không? (Bridge auto-insert waypoint 0)

**Giải pháp:**
1. Xóa tất cả waypoints
2. Click 1 điểm trên bản đồ
3. Nhấn "Bắt đầu Mission"
4. Kiểm tra Log có lỗi gì không

### ❌ Lỗi: "Đổi mode thất bại! Result code 4"

**Nguyên nhân:** Mission quá ít hoặc không hợp lệ

**Giải pháp:**
1. Bridge hiện tại **tự động thêm waypoint 0** (vị trí xe hiện tại)
2. Click ít nhất 1 điểm khác trên bản đồ
3. Bắt đầu Mission lại

### ❌ Xem Log để debug

Trong Terminal 2 (Bridge), sẽ hiển thị chi tiết:
```
==================================================
Nhận lệnh từ client: MISSION_START
==================================================
→ Mission count: 1 waypoints
→ Lấy vị trí hiện tại của xe...
  ✓ Vị trí xe: 16.054400, 108.202200
  → Auto thêm waypoint 0 (vị trí hiện tại)
→ Đang upload 2 waypoints...
...
```

## 📚 7. Tài liệu tham khảo

- [ArduPilot Documentation](https://ardupilot.org/)
- [MAVLink Protocol](https://mavlink.io/)
- [pymavlink Guide](https://github.com/ArduPilot/pymavlink)
- [SITL Guide](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

## 🗺️ 8. Các vị trí SITL có sẵn

```bash
# Xem danh sách vị trí
cat ~/ardupilot/Tools/autotest/locations.txt

# Ví dụ:
-L CMAC      # Canberra, Australia
-L KSFO      # San Francisco
-L AVC       # Arizona
-L DUBAI     # Dubai
```

**Chạy tại vị trí khác:**
```bash
cd ~/ardupilot/Tools/autotest
python3 sim_vehicle.py -v Rover --console --map -L KSFO
```

## ⚡ 9. Tips & Tricks

### Tăng tốc simulation
```bash
python3 sim_vehicle.py -v Rover --console --map --speedup 5
```

### Chạy SITL headless (không GUI)
```bash
python3 sim_vehicle.py -v Rover --no-console --no-map
```

### Kiểm tra MAVProxy connections
```bash
# Trong MAVProxy console:
help output
output list
```

### Reset SITL về vị trí ban đầu
```bash
# Ctrl+C để tắt SITL, sau đó chạy lại
python3 sim_vehicle.py -v Rover --console --map
```

### Sử dụng MAVProxy console song song

Bạn có thể điều khiển xe từ cả web interface và MAVProxy console cùng lúc:

**Từ MAVProxy:**
```bash
mode GUIDED
arm throttle
guided 16.055 108.202
```

**Hoặc từ Web Interface:**
- Click trên bản đồ để tạo waypoints
- Nhấn "Bắt đầu Mission"

## 🎓 10. Bài tập thực hành

### Level 1: Cơ bản
1. Khởi động SITL
2. Kết nối Web Interface
3. Xem Telemetry (vị trí, GPS, Battery)
4. ARM và DISARM xe

### Level 2: Mission đơn giản
1. Click 3 điểm trên bản đồ (hình tam giác)
2. Nhấn "Bắt đầu Mission"
3. Quan sát xe di chuyển qua các waypoints
4. Nhấn "HOLD" để dừng lại

### Level 3: Mission phức tạp
1. Tạo mission hình chữ nhật (4-6 waypoints)
2. Kiểm tra "Mission Distance" (tính toán khoảng cách)
3. Sử dụng MAVProxy để monitor
4. Test tốc độ khác nhau (modify CRUISE_SPEED)

### Level 4: Nâng cao
1. Tạo 2 SITL instances cùng lúc (`-I 0` và `-I 1`)
2. Chạy Bridge cho cả 2 (port 5760 và 5761)
3. Điều khiển 2 xe cùng lúc từ 2 web interface

## 🔗 11. Cấu hình nâng cao

### Thay đổi CRUISE_SPEED

**Từ MAVProxy console:**
```bash
param show CRUISE_SPEED
param set CRUISE_SPEED 5
```

**Lưu persistent:**
```bash
param save
```

### Thay đổi vị trí khởi động

```bash
python3 sim_vehicle.py -v Rover -L "16.0544, 108.2022"
```

### Enable logging

```bash
python3 sim_vehicle.py -v Rover --console --map --log
# Logs sẽ lưu trong ~/ardupilot/logs
```

## 📋 12. Checklist cơ bản

- [ ] Đã tạo thư mục `~/ardupilot-rover-project`
- [ ] Đã tạo virtual environment `venv`
- [ ] **Đã activate venv** (`source venv/bin/activate`)
- [ ] Đã cài đặt pymavlink, websockets
- [ ] SITL đang chạy (Terminal 1)
- [ ] Bridge đang chạy (Terminal 2) với venv
- [ ] Web Interface đã kết nối WebSocket
- [ ] Telemetry hiển thị vị trí GPS
- [ ] Có ít nhất 1 waypoint trên bản đồ
- [ ] Xe đã ARM
- [ ] Mission bắt đầu thành công

## 📖 13. Quick Start Script (Optional)

Để không phải activate venv mỗi lần, bạn có thể tạo script:

**Tạo file `start_bridge.sh`:**
```bash
#!/bin/bash
cd ~/ardupilot-rover-project
source venv/bin/activate
python websocket_bridge.py --mavlink tcp:127.0.0.1:5763 --port 5760
```

**Phân quyền và chạy:**
```bash
chmod +x start_bridge.sh
./start_bridge.sh
```

---

**Happy Coding! 🚀**

**Phiên bản:** 1.1 (Updated - Fixed Auto-insert Waypoint 0)
