#!/usr/bin/env python3
"""
MAVLink WebSocket Bridge - Version 4 with PARAM-based Speed Control
Chuyển tiếp MAVLink giữa ArduPilot và Web Browser
FIXED: Sử dụng WP_SPEED/CRUISE_SPEED params thay vì DO_CHANGE_SPEED command
"""

import asyncio
import websockets
import json
from pymavlink import mavutil
import time
import socket

class MAVLinkWebSocketBridge:
    def __init__(self, mavlink_connection='udp:127.0.0.1:14550', websocket_port=5760):
        self.mavlink_connection = mavlink_connection
        self.websocket_port = websocket_port
        self.clients = set()
        self.vehicle = None
        self.mission_items = []
        
    def find_free_port(self, start_port):
        """Tìm cổng khả dụng bắt đầu từ start_port"""
        port = start_port
        while port < start_port + 100:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(('0.0.0.0', port))
                sock.close()
                return port
            except OSError:
                port += 1
        raise RuntimeError(f"Không tìm thấy cổng khả dụng từ {start_port} đến {port}")
        
    def connect_mavlink(self):
        """Kết nối với ArduPilot"""
        print(f"Đang kết nối MAVLink: {self.mavlink_connection}")
        try:
            self.vehicle = mavutil.mavlink_connection(self.mavlink_connection)
            self.vehicle.wait_heartbeat(timeout=10)
            print(f"✓ MAVLink kết nối thành công!")
            print(f"  System ID: {self.vehicle.target_system}")
            print(f"  Component ID: {self.vehicle.target_component}")
            return True
        except Exception as e:
            print(f"✗ Lỗi kết nối MAVLink: {e}")
            return False
        
    async def register_client(self, websocket):
        """Đăng ký client WebSocket mới"""
        self.clients.add(websocket)
        print(f"✓ Client kết nối: {websocket.remote_address}")
        try:
            await websocket.wait_closed()
        finally:
            self.clients.remove(websocket)
            print(f"✗ Client ngắt kết nối: {websocket.remote_address}")
    
    async def broadcast_mavlink_data(self):
        """Broadcast dữ liệu MAVLink cho tất cả clients"""
        while True:
            if not self.vehicle:
                await asyncio.sleep(0.1)
                continue
                
            try:
                msg = self.vehicle.recv_match(blocking=False)
                
                if msg:
                    data = self.mavlink_to_json(msg)
                    
                    if data and self.clients:
                        message = json.dumps(data)
                        websockets.broadcast(self.clients, message)
                
                await asyncio.sleep(0.01)
            except Exception as e:
                print(f"Lỗi broadcast: {e}")
                await asyncio.sleep(0.1)
    
    def mavlink_to_json(self, msg):
        """Chuyển MAVLink message sang JSON"""
        msg_type = msg.get_type()
        
        if msg_type == 'HEARTBEAT':
            return {
                'type': 'HEARTBEAT',
                'custom_mode': msg.custom_mode,
                'autopilot': msg.autopilot,
                'base_mode': msg.base_mode,
                'system_status': msg.system_status,
                'mavlink_version': msg.mavlink_version
            }
        
        elif msg_type == 'GLOBAL_POSITION_INT':
            return {
                'type': 'GLOBAL_POSITION_INT',
                'time_boot_ms': msg.time_boot_ms,
                'lat': msg.lat,
                'lon': msg.lon,
                'alt': msg.alt,
                'relative_alt': msg.relative_alt,
                'vx': msg.vx,
                'vy': msg.vy,
                'vz': msg.vz,
                'hdg': msg.hdg
            }
        
        elif msg_type == 'VFR_HUD':
            return {
                'type': 'VFR_HUD',
                'airspeed': msg.airspeed,
                'groundspeed': msg.groundspeed,
                'heading': msg.heading,
                'throttle': msg.throttle,
                'alt': msg.alt,
                'climb': msg.climb
            }
        
        elif msg_type == 'SYS_STATUS':
            return {
                'type': 'SYS_STATUS',
                'battery_remaining': msg.battery_remaining,
                'voltage_battery': msg.voltage_battery,
                'current_battery': msg.current_battery
            }
        
        elif msg_type == 'ATTITUDE':
            return {
                'type': 'ATTITUDE',
                'roll': msg.roll,
                'pitch': msg.pitch,
                'yaw': msg.yaw,
                'rollspeed': msg.rollspeed,
                'pitchspeed': msg.pitchspeed,
                'yawspeed': msg.yawspeed
            }
        
        elif msg_type == 'COMMAND_ACK':
            return {
                'type': 'COMMAND_ACK',
                'command': msg.command,
                'result': msg.result
            }
        
        elif msg_type == 'MISSION_ACK':
            return {
                'type': 'MISSION_ACK',
                'mission_type': msg.type
            }
        
        elif msg_type == 'GPS_RAW_INT':
            return {
                'type': 'GPS_RAW_INT',
                'fix_type': msg.fix_type,
                'satellites_visible': msg.satellites_visible
            }
        
        return None
    
    def check_arm_status(self):
        """Kiểm tra trạng thái ARM"""
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            return bool(armed)
        return False
    
    def get_mode_name(self, custom_mode):
        """Lấy tên mode từ custom_mode ID"""
        mode_names = {
            0: 'MANUAL',
            1: 'ACRO',
            2: 'LEARNING',
            3: 'STEERING',
            4: 'HOLD',
            10: 'AUTO',
            11: 'RTL',
            12: 'SMART_RTL',
            15: 'GUIDED'
        }
        return mode_names.get(custom_mode, f'UNKNOWN({custom_mode})')
    
    def set_speed(self, speed_mps):
        """Thiết lập tốc độ mục tiêu (m/s) - SỬ DỤNG PARAM THAY VÌ COMMAND"""
        print(f"\n→ Thiết lập tốc độ: {speed_mps} m/s (~{speed_mps * 3.6:.1f} km/h)")
        
        try:
            # PHƯƠNG PHÁP MỚI: Set tham số WP_SPEED và CRUISE_SPEED
            print(f"  → Đang set WP_SPEED = {speed_mps}")
            self.vehicle.mav.param_set_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                b'WP_SPEED',
                speed_mps,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            
            # Chờ xác nhận
            msg = self.vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            if msg:
                # Xử lý param_id (có thể là bytes hoặc str tùy phiên bản pymavlink)
                param_id = msg.param_id
                if isinstance(param_id, bytes):
                    param_id = param_id.decode('utf-8').strip('\x00')
                else:
                    param_id = param_id.strip('\x00')
                
                if param_id == 'WP_SPEED':
                    print(f"  ✓ WP_SPEED = {msg.param_value} m/s")
                    
                    # Set cả CRUISE_SPEED
                    print(f"  → Đang set CRUISE_SPEED = {speed_mps}")
                    self.vehicle.mav.param_set_send(
                        self.vehicle.target_system,
                        self.vehicle.target_component,
                        b'CRUISE_SPEED',
                        speed_mps,
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                    )
                    
                    msg2 = self.vehicle.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
                    if msg2:
                        print(f"  ✓ CRUISE_SPEED = {msg2.param_value} m/s")
                    
                    print(f"✓✓ Đã thiết lập tốc độ thành công: {speed_mps} m/s (~{speed_mps*3.6:.1f} km/h)")
                    return True
                else:
                    print(f"✗ Nhận được param khác: {param_id}")
                    return False
            else:
                print(f"✗ Không nhận được xác nhận từ ArduPilot")
                return False
                
        except Exception as e:
            print(f"✗ Lỗi khi set tốc độ: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def arm_vehicle(self):
        """ARM vehicle"""
        print("\n→ Đang ARM vehicle...")
        
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            print(f"  Base mode: {msg.base_mode}")
            print(f"  System status: {msg.system_status}")
            
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("✓ Vehicle đã ARM rồi!")
                return True
        
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0
        )
        
        msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            print(f"  ACK Result: {msg.result}")
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("✓ ARM thành công!")
                time.sleep(1)
                return True
            else:
                print(f"✗ ARM thất bại! Result code: {msg.result}")
                return False
        
        print("✗ Không nhận được ACK")
        return False
    
    def disarm_vehicle(self):
        """DISARM vehicle"""
        print("\n→ Đang DISARM vehicle...")
        
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0, 0, 0, 0, 0, 0
        )
        
        msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("✓ DISARM thành công!")
                return True
            else:
                print(f"✗ DISARM thất bại! Result: {msg.result}")
                return False
        
        print("✗ Không nhận được ACK")
        return False
    
    def set_mode(self, mode_name):
        """Đổi flight mode"""
        print(f"\n→ Đang đổi mode sang {mode_name}...")
        
        mode_mapping = {
            'MANUAL': 0,
            'ACRO': 1,
            'LEARNING': 2,
            'STEERING': 3,
            'HOLD': 4,
            'AUTO': 10,
            'RTL': 11,
            'SMART_RTL': 12,
            'GUIDED': 15
        }
        
        if mode_name not in mode_mapping:
            print(f"✗ Mode không hợp lệ: {mode_name}")
            return False
        
        mode_id = mode_mapping[mode_name]
        
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0, 0, 0, 0, 0
        )
        
        msg = self.vehicle.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            print(f"  ACK Result: {msg.result}")
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                time.sleep(0.5)
                hb = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
                if hb and hb.custom_mode == mode_id:
                    print(f"✓ Đã đổi sang mode {mode_name}!")
                    return True
                else:
                    print(f"⚠ ACK OK nhưng mode chưa đổi (current: {hb.custom_mode if hb else 'unknown'})")
                    return False
            else:
                print(f"✗ Đổi mode thất bại! Result code: {msg.result}")
                return False
        
        print("  → Thử phương pháp 2: set_mode_send...")
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
        start_time = time.time()
        while time.time() - start_time < 3:
            msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == mode_id:
                print(f"✓ Đã đổi sang mode {mode_name}!")
                return True
        
        print(f"✗ Không thể đổi sang mode {mode_name}")
        return False
    
    def clear_mission(self):
        """Xóa mission hiện tại"""
        print("→ Đang xóa mission cũ...")
        self.vehicle.mav.mission_clear_all_send(
            self.vehicle.target_system,
            self.vehicle.target_component
        )
        
        msg = self.vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
        if msg:
            print("✓ Đã xóa mission cũ")
            return True
        print("✗ Không nhận được ACK khi xóa mission")
        return False
    
    def upload_mission_simple(self, waypoints):
        """
        Upload mission đơn giản - chỉ waypoints, không có speed commands
        """
        if len(waypoints) == 0:
            print("✗ Không có waypoint nào!")
            return False
        
        print(f"\n→ Upload {len(waypoints)} waypoints...")
        
        # Lấy vị trí hiện tại
        current_pos = self._get_current_position()
        
        if current_pos:
            print(f"  ✓ Vị trí xe: {current_pos['lat']:.6f}, {current_pos['lon']:.6f}")
            waypoints.insert(0, {
                'seq': 0,
                'lat': current_pos['lat'],
                'lon': current_pos['lon'],
                'alt': current_pos['alt']
            })
            print(f"  → Auto thêm waypoint 0 (vị trí hiện tại)")
        
        self.clear_mission()
        time.sleep(0.5)
        
        # Gửi số lượng waypoint
        self.vehicle.mav.mission_count_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            len(waypoints),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        
        waypoints_sent = 0
        timeout_start = time.time()
        
        while waypoints_sent < len(waypoints):
            msg = self.vehicle.recv_match(
                type=['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'], 
                blocking=True, 
                timeout=5
            )
            
            if not msg:
                print(f"✗ Timeout - Không nhận được message sau waypoint {waypoints_sent}")
                return False
            
            if msg.get_type() == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print(f"✓ Upload mission thành công! {waypoints_sent} waypoints")
                    return True
                else:
                    print(f"✗ Upload mission thất bại! ACK Type: {msg.type}")
                    return False
            
            seq = msg.seq
            
            if seq >= len(waypoints):
                print(f"✗ REQUEST seq {seq} vượt quá số waypoints ({len(waypoints)})")
                return False
            
            wp = waypoints[seq]
            current = 1 if seq == 0 else 0
            
            # Gửi waypoint
            self.vehicle.mav.mission_item_int_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                current,
                1,  # autocontinue
                0,  # param1 (hold time)
                2.0,  # param2 (acceptance radius)
                0,  # param3 (pass radius)
                float('nan'),  # param4 (yaw)
                int(wp['lat'] * 1e7),
                int(wp['lon'] * 1e7),
                float(wp.get('alt', 10))
            )
            
            waypoints_sent += 1
            
            if time.time() - timeout_start > 30:
                print("✗ Timeout 30s khi upload mission")
                return False
        
        msg = self.vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
        if msg:
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print(f"✓ Upload mission thành công! {len(waypoints)} waypoints")
                return True
            else:
                print(f"✗ Upload mission thất bại! ACK Type: {msg.type}")
                return False
        
        print(f"✗ Không nhận được MISSION_ACK sau khi gửi hết waypoints")
        return False
    
    def upload_mission_with_speed(self, waypoints):
        """
        Upload mission và set speed qua PARAM
        """
        if len(waypoints) == 0:
            print("✗ Không có waypoint nào!")
            return False
        
        # Lấy tốc độ trung bình hoặc tốc độ cao nhất
        speeds = [wp.get('speed', 5.0) for wp in waypoints]
        avg_speed = sum(speeds) / len(speeds)
        max_speed = max(speeds)
        
        print(f"\n→ Upload mission với tốc độ:")
        print(f"  • Tốc độ trung bình: {avg_speed:.1f} m/s")
        print(f"  • Tốc độ tối đa: {max_speed:.1f} m/s")
        print(f"  → Sẽ sử dụng tốc độ: {max_speed:.1f} m/s")
        
        # Set tốc độ trước khi upload mission
        if not self.set_speed(max_speed):
            print("⚠ Cảnh báo: Không thể set tốc độ, tiếp tục upload mission...")
        
        time.sleep(0.5)
        
        # Upload mission như bình thường (không cần DO_CHANGE_SPEED)
        return self.upload_mission_simple(waypoints)
    
    def start_mission(self):
        """Bắt đầu mission"""
        print("\n→ Đang bắt đầu mission...")
        
        mission_count = self._get_mission_count()
        print(f"→ Mission count: {mission_count} items")
        
        if mission_count == 0:
            print("✗ Chưa có mission nào được upload!")
            return False
        
        if not self.check_arm_status():
            print("⚠ Vehicle chưa ARM! Đang ARM...")
            if not self.arm_vehicle():
                print("✗ Không thể ARM vehicle!")
                return False
            time.sleep(1)
        else:
            print("✓ Vehicle đã ARM")
        
        print("→ Chuyển sang GUIDED mode trước (prepare)...")
        if self.set_mode('GUIDED'):
            print("✓ Đã vào GUIDED mode")
            time.sleep(1)
        else:
            print("⚠ GUIDED mode thất bại, tiếp tục thử AUTO...")
        
        print("→ Chuyển sang AUTO mode...")
        max_retries = 3
        for attempt in range(max_retries):
            if attempt > 0:
                print(f"  → Thử lần {attempt + 1}/{max_retries}...")
                time.sleep(1)
            
            if self.set_mode('AUTO'):
                print("✓ Đã chuyển sang AUTO mode thành công!")
                
                time.sleep(1)
                
                msg = self.vehicle.recv_match(type='MISSION_CURRENT', blocking=True, timeout=2)
                if msg:
                    print(f"✓ Mission đang chạy, item hiện tại: {msg.seq}")
                    if msg.seq == 0:
                        print("✓✓ Đúng! Đang chạy từ item 0!")
                    else:
                        print(f"⚠ CẢNH BÁO: Đang chạy từ item {msg.seq} (không phải 0)")
                
                return True
        
        print("✗ Không thể chuyển sang AUTO mode sau nhiều lần thử")
        return False
    
    def _get_mission_count(self):
        """Helper: Lấy số lượng items trong mission"""
        self.vehicle.mav.mission_request_list_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )
        
        msg = self.vehicle.recv_match(type='MISSION_COUNT', blocking=True, timeout=3)
        if msg:
            return msg.count
        return 0
    
    def _get_current_position(self):
        """Helper: Lấy vị trí hiện tại của xe"""
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            return {
                'lat': lat,
                'lon': lon,
                'alt': alt
            }
        return None
    
    async def handle_client_message(self, websocket):
        """Xử lý message từ WebSocket client"""
        async for message in websocket:
            try:
                data = json.loads(message)
                command = data.get('command')
                
                print(f"\n{'='*50}")
                print(f"Nhận lệnh từ client: {command}")
                print(f"{'='*50}")
                
                if command == 'ARM':
                    loop = asyncio.get_event_loop()
                    success = await loop.run_in_executor(None, self.arm_vehicle)
                    response = {'type': 'ARM_RESPONSE', 'success': success}
                    await websocket.send(json.dumps(response))
                
                elif command == 'DISARM':
                    loop = asyncio.get_event_loop()
                    success = await loop.run_in_executor(None, self.disarm_vehicle)
                    response = {'type': 'DISARM_RESPONSE', 'success': success}
                    await websocket.send(json.dumps(response))
                
                elif command == 'SET_MODE':
                    mode = data.get('mode', 'GUIDED')
                    loop = asyncio.get_event_loop()
                    success = await loop.run_in_executor(None, self.set_mode, mode)
                    response = {'type': 'SET_MODE_RESPONSE', 'success': success, 'mode': mode}
                    await websocket.send(json.dumps(response))
                
                elif command == 'MISSION_COUNT':
                    count = data.get('count', 0)
                    self.mission_items = []
                    print(f"→ Chuẩn bị nhận {count} waypoints...")
                
                elif command == 'MISSION_ITEM':
                    seq = data.get('seq', 0)
                    lat = data.get('x')
                    lon = data.get('y')
                    alt = data.get('z', 10)
                    speed = data.get('speed', 5.0)
                    
                    waypoint = {
                        'seq': seq,
                        'lat': lat,
                        'lon': lon,
                        'alt': alt,
                        'speed': speed
                    }
                    self.mission_items.append(waypoint)
                    print(f"  ✓ Nhận waypoint {seq}: {lat:.6f}, {lon:.6f}, {alt}m, speed={speed}m/s (~{speed*3.6:.1f}km/h)")
                
                elif command == 'MISSION_START':
                    if len(self.mission_items) > 0:
                        self.mission_items.sort(key=lambda x: x['seq'])
                        
                        # In ra tổng quan về speeds
                        speeds = [wp['speed'] for wp in self.mission_items]
                        unique_speeds = set(speeds)
                        print(f"\n📊 TỔNG QUAN TỐC ĐỘ:")
                        print(f"  • Tổng số waypoints: {len(self.mission_items)}")
                        print(f"  • Các tốc độ khác nhau: {sorted(unique_speeds)} m/s")
                        
                        loop = asyncio.get_event_loop()
                        
                        # Luôn dùng phương pháp set PARAM
                        if len(unique_speeds) > 1:
                            print("\n→ Phát hiện nhiều tốc độ khác nhau, sẽ dùng tốc độ cao nhất...")
                        
                        success = await loop.run_in_executor(
                            None, 
                            self.upload_mission_with_speed, 
                            self.mission_items.copy()
                        )
                        
                        if success:
                            await asyncio.sleep(0.5)
                            success = await loop.run_in_executor(None, self.start_mission)
                            response = {'type': 'MISSION_START_RESPONSE', 'success': success}
                        else:
                            response = {'type': 'MISSION_START_RESPONSE', 'success': False}
                        
                        await websocket.send(json.dumps(response))
                    else:
                        print("✗ Chưa có waypoint nào để start mission!")
                        response = {'type': 'MISSION_START_RESPONSE', 'success': False}
                        await websocket.send(json.dumps(response))
                
                elif command == 'REQUEST_DATA_STREAM':
                    self.vehicle.mav.request_data_stream_send(
                        self.vehicle.target_system,
                        self.vehicle.target_component,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL,
                        4,
                        1
                    )
                    print("✓ Đã yêu cầu data stream")
                
                elif command == 'SET_SPEED':
                    speed = data.get('speed', 5.0)
                    loop = asyncio.get_event_loop()
                    success = await loop.run_in_executor(None, self.set_speed, speed)
                    response = {'type': 'SET_SPEED_RESPONSE', 'success': success, 'speed': speed}
                    await websocket.send(json.dumps(response))
                
            except json.JSONDecodeError:
                print(f"✗ JSON decode error: {message}")
            except Exception as e:
                print(f"✗ Error handling message: {e}")
                import traceback
                traceback.print_exc()
    
    async def websocket_handler(self, websocket):
        """Handler cho WebSocket connections"""
        register_task = asyncio.create_task(self.register_client(websocket))
        message_task = asyncio.create_task(self.handle_client_message(websocket))
        
        done, pending = await asyncio.wait(
            [register_task, message_task],
            return_when=asyncio.FIRST_COMPLETED
        )
        
        for task in pending:
            task.cancel()
    
    async def start_server(self):
        """Khởi động WebSocket server"""
        try:
            self.websocket_port = self.find_free_port(self.websocket_port)
            print(f"Sử dụng cổng: {self.websocket_port}")
        except RuntimeError as e:
            print(f"Lỗi: {e}")
            return
        
        print(f"Khởi động WebSocket server trên port {self.websocket_port}...")
        
        if not self.connect_mavlink():
            print("Không thể kết nối MAVLink. Thoát...")
            return
        
        self.vehicle.mav.request_data_stream_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4, 1
        )
        
        async with websockets.serve(
            self.websocket_handler, 
            "0.0.0.0", 
            self.websocket_port,
            ping_interval=20,
            ping_timeout=20
        ):
            print(f"\n{'='*60}")
            print(f"✓ WebSocket server đang chạy tại ws://localhost:{self.websocket_port}")
            print(f"  Kết nối từ trình duyệt: ws://localhost:{self.websocket_port}")
            print(f"{'='*60}\n")
            print("🚀 FEATURES:")
            print("  ✓ Sử dụng WP_SPEED và CRUISE_SPEED params")
            print("  ✓ Tự động set tốc độ trước khi upload mission")
            print("  ✓ Waypoint interval: 1m (độ chính xác cao)")
            print(f"{'='*60}\n")
            print("Đang chờ client kết nối...")
            
            await self.broadcast_mavlink_data()
    
    def run(self):
        """Chạy bridge"""
        try:
            asyncio.run(self.start_server())
        except KeyboardInterrupt:
            print("\n\nĐã dừng bridge.")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='MAVLink WebSocket Bridge v4 - PARAM-based Speed Control')
    parser.add_argument('--mavlink', default='udp:127.0.0.1:14550',
                        help='MAVLink connection string (default: udp:127.0.0.1:14550)')
    parser.add_argument('--port', type=int, default=5760,
                        help='WebSocket port (default: 5760)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("MAVLink WebSocket Bridge - VERSION 4.0 (PARAM-BASED)")
    print("=" * 60)
    print(f"MAVLink: {args.mavlink}")
    print(f"WebSocket Port: {args.port}")
    print("=" * 60)
    print("\n🔧 FIXED ISSUES:")
    print("  ✓ Sử dụng WP_SPEED và CRUISE_SPEED params")
    print("  ✓ Không dùng DO_CHANGE_SPEED (Rover không hỗ trợ)")
    print("  ✓ Tự động set tốc độ trước khi upload mission")
    print("\n📋 PHƯƠNG PHÁP:")
    print("  • Set param WP_SPEED trước khi upload")
    print("  • Set param CRUISE_SPEED để đảm bảo")
    print("  • Upload waypoints đơn giản (không speed commands)")
    print("\n" + "=" * 60 + "\n")
    
    bridge = MAVLinkWebSocketBridge(args.mavlink, args.port)
    bridge.run()
