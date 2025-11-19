#!/usr/bin/env python3
"""
ArduPilot Rover Controller - Python MAVLink
Điều khiển xe ArduRover qua MAVLink
"""

from pymavlink import mavutil
import time
import math

class RoverController:
    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        """
        Khởi tạo kết nối với ArduPilot
        connection_string: Địa chỉ kết nối (udp, tcp, serial)
        """
        print(f"Đang kết nối tới {connection_string}...")
        self.vehicle = mavutil.mavlink_connection(connection_string)
        
        # Đợi heartbeat đầu tiên
        print("Đợi heartbeat từ xe...")
        self.vehicle.wait_heartbeat()
        print(f"Heartbeat nhận được từ hệ thống {self.vehicle.target_system}, component {self.vehicle.target_component}")
        
        # Request data stream
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 1)
        
    def request_message_interval(self, message_id, frequency_hz):
        """Yêu cầu message với tần suất cụ thể"""
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            1e6 / frequency_hz,
            0, 0, 0, 0, 0
        )

    def arm(self):
        """ARM xe (bật động cơ)"""
        print("Đang ARM xe...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        
        # Đợi xác nhận
        ack = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack and ack.result == 0:
            print("✓ ARM thành công!")
            return True
        else:
            print("✗ ARM thất bại!")
            return False

    def disarm(self):
        """DISARM xe (tắt động cơ)"""
        print("Đang DISARM xe...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("✓ DISARM hoàn tất!")

    def set_mode(self, mode):
        """
        Đổi chế độ bay
        Modes: MANUAL, GUIDED, AUTO, RTL, HOLD
        """
        mode_mapping = {
            'MANUAL': 0,
            'ACRO': 1,
            'STEERING': 3,
            'HOLD': 4,
            'AUTO': 10,
            'RTL': 11,
            'GUIDED': 15
        }
        
        if mode not in mode_mapping:
            print(f"Chế độ không hợp lệ: {mode}")
            return False
            
        mode_id = mode_mapping[mode]
        
        print(f"Đổi sang chế độ {mode}...")
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        # Chờ xác nhận
        time.sleep(0.5)
        print(f"✓ Đã chuyển sang chế độ {mode}")
        return True

    def goto_position(self, lat, lon, altitude=0):
        """
        Di chuyển xe đến vị trí GPS
        lat: Vĩ độ (độ)
        lon: Kinh độ (độ)
        altitude: Độ cao (m) - không dùng cho rover
        """
        print(f"Đang di chuyển đến: {lat}, {lon}")
        
        self.vehicle.mav.mission_item_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            0,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,  # current = 2 để set guided mode waypoint
            1,  # autocontinue
            0, 0, 0, 0,  # params 1-4
            lat, lon, altitude
        )

    def set_waypoint_mission(self, waypoints):
        """
        Upload mission với nhiều waypoint
        waypoints: list của (lat, lon, altitude)
        """
        print(f"Upload mission với {len(waypoints)} waypoints...")
        
        # Xóa mission cũ
        self.vehicle.mav.mission_clear_all_send(
            self.vehicle.target_system,
            self.vehicle.target_component
        )
        time.sleep(0.5)
        
        # Gửi số lượng waypoint
        self.vehicle.mav.mission_count_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            len(waypoints)
        )
        
        # Upload từng waypoint
        for i, (lat, lon, alt) in enumerate(waypoints):
            # Đợi request cho waypoint này
            msg = self.vehicle.recv_match(type='MISSION_REQUEST', blocking=True, timeout=5)
            if msg and msg.seq == i:
                self.vehicle.mav.mission_item_send(
                    self.vehicle.target_system,
                    self.vehicle.target_component,
                    i,  # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0,  # current (0 = not current, 1 = current)
                    1,  # autocontinue
                    0, 0, 0, 0,  # params
                    lat, lon, alt
                )
                print(f"  Waypoint {i+1}/{len(waypoints)} uploaded: {lat}, {lon}")
        
        # Đợi ACK
        ack = self.vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if ack:
            print("✓ Mission upload thành công!")
            return True
        return False

    def start_mission(self):
        """Bắt đầu mission AUTO"""
        print("Khởi động mission...")
        self.set_mode('AUTO')
        
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("✓ Mission đã bắt đầu!")

    def get_position(self):
        """Lấy vị trí hiện tại"""
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg:
            return {
                'lat': msg.lat / 1e7,
                'lon': msg.lon / 1e7,
                'alt': msg.alt / 1000.0,
                'heading': msg.hdg / 100.0
            }
        return None

    def get_telemetry(self):
        """Lấy thông tin telemetry"""
        msg = self.vehicle.recv_match(type='VFR_HUD', blocking=True, timeout=3)
        if msg:
            return {
                'groundspeed': msg.groundspeed,
                'airspeed': msg.airspeed,
                'altitude': msg.alt,
                'climb': msg.climb,
                'throttle': msg.throttle
            }
        return None

    def get_distance_to_waypoint(self, lat, lon):
        """Tính khoảng cách đến waypoint (m)"""
        current_pos = self.get_position()
        if not current_pos:
            return None
        
        # Haversine formula
        R = 6371000  # bán kính trái đất (m)
        lat1 = math.radians(current_pos['lat'])
        lat2 = math.radians(lat)
        dlat = math.radians(lat - current_pos['lat'])
        dlon = math.radians(lon - current_pos['lon'])
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        return R * c

    def monitor_mission(self):
        """Theo dõi mission realtime"""
        print("\n=== Theo dõi Mission ===")
        print("Nhấn Ctrl+C để dừng\n")
        
        try:
            while True:
                pos = self.get_position()
                telem = self.get_telemetry()
                
                if pos and telem:
                    print(f"\rVị trí: ({pos['lat']:.6f}, {pos['lon']:.6f}) | "
                          f"Hướng: {pos['heading']:.0f}° | "
                          f"Tốc độ: {telem['groundspeed']:.1f} m/s | "
                          f"Throttle: {telem['throttle']}%", end='')
                
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n\nDừng theo dõi.")


# ============================================
# DEMO: Sử dụng controller
# ============================================

def demo_basic():
    """Demo cơ bản: ARM, GUIDED mode, di chuyển"""
    rover = RoverController('udp:127.0.0.1:14550')
    
    # ARM xe
    rover.arm()
    time.sleep(2)
    
    # Chuyển sang GUIDED mode
    rover.set_mode('GUIDED')
    time.sleep(2)
    
    # Lấy vị trí hiện tại
    pos = rover.get_position()
    print(f"Vị trí hiện tại: {pos}")
    
    # Di chuyển 100m về phía Bắc
    if pos:
        new_lat = pos['lat'] + 0.0009  # ~100m
        rover.goto_position(new_lat, pos['lon'])
        
        # Theo dõi cho đến khi gần đến
        print("Đang di chuyển...")
        while True:
            dist = rover.get_distance_to_waypoint(new_lat, pos['lon'])
            if dist and dist < 5:  # Dừng khi còn 5m
                print(f"\n✓ Đã đến gần mục tiêu! (còn {dist:.1f}m)")
                break
            print(f"\rKhoảng cách: {dist:.1f}m", end='')
            time.sleep(0.5)
    
    # DISARM
    rover.disarm()


def demo_waypoint_mission():
    """Demo mission với nhiều waypoint"""
    rover = RoverController('udp:127.0.0.1:14550')
    
    # Tạo mission hình vuông
    start_pos = rover.get_position()
    if not start_pos:
        print("Không lấy được vị trí!")
        return
    
    lat = start_pos['lat']
    lon = start_pos['lon']
    
    # Waypoints tạo hình vuông 100m x 100m
    waypoints = [
        (lat + 0.0009, lon, 0),           # Bắc
        (lat + 0.0009, lon + 0.0009, 0),  # Đông Bắc
        (lat, lon + 0.0009, 0),           # Đông
        (lat, lon, 0),                    # Về điểm bắt đầu
    ]
    
    print("\n=== Bắt đầu Mission Hình Vuông ===")
    
    # Upload waypoints
    rover.set_waypoint_mission(waypoints)
    time.sleep(2)
    
    # ARM
    rover.arm()
    time.sleep(2)
    
    # Bắt đầu mission
    rover.start_mission()
    
    # Theo dõi mission
    rover.monitor_mission()
    
    # DISARM khi hoàn tất
    rover.disarm()


def demo_manual_control():
    """Demo điều khiển thủ công"""
    rover = RoverController('udp:127.0.0.1:14550')
    
    print("\n=== Điều khiển Manual ===")
    print("Các lệnh:")
    print("  a - ARM")
    print("  d - DISARM")
    print("  g - GUIDED mode")
    print("  m - MANUAL mode")
    print("  h - HOLD mode")
    print("  p - Hiển thị vị trí")
    print("  t - Hiển thị telemetry")
    print("  q - Thoát")
    
    while True:
        cmd = input("\nNhập lệnh: ").strip().lower()
        
        if cmd == 'a':
            rover.arm()
        elif cmd == 'd':
            rover.disarm()
        elif cmd == 'g':
            rover.set_mode('GUIDED')
        elif cmd == 'm':
            rover.set_mode('MANUAL')
        elif cmd == 'h':
            rover.set_mode('HOLD')
        elif cmd == 'p':
            pos = rover.get_position()
            if pos:
                print(f"Vị trí: {pos}")
        elif cmd == 't':
            telem = rover.get_telemetry()
            if telem:
                print(f"Telemetry: {telem}")
        elif cmd == 'q':
            print("Thoát...")
            break
        else:
            print("Lệnh không hợp lệ!")


if __name__ == '__main__':
    import sys
    
    print("=" * 50)
    print("ArduPilot Rover Controller - Python")
    print("=" * 50)
    print("\nChọn demo:")
    print("1. Demo cơ bản (di chuyển 1 điểm)")
    print("2. Demo mission waypoint (hình vuông)")
    print("3. Demo điều khiển manual")
    
    choice = input("\nNhập lựa chọn (1-3): ").strip()
    
    try:
        if choice == '1':
            demo_basic()
        elif choice == '2':
            demo_waypoint_mission()
        elif choice == '3':
            demo_manual_control()
        else:
            print("Lựa chọn không hợp lệ!")
    except KeyboardInterrupt:
        print("\n\nĐã dừng chương trình.")
    except Exception as e:
        print(f"\n\nLỗi: {e}")
