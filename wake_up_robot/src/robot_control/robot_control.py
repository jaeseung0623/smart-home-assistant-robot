import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init
from queue import Queue
from od_msg.srv import SrvDepthPosition
from od_msg.srv import ExampleInterface
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG
from .weather_forecast import WeatherForecast
from .pick_bedding import Pickbedding
import threading
import pygame
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from .sound_player import SoundPlayer
from geometry_msgs.msg import Point
from std_msgs.msg import String # 발표 전 수정


package_path = get_package_share_directory("pick_and_place_voice")
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0
init_pos = [-0.36, 20.12, 36.90, 1.08, 122.98, 0.36] # 시작점
init_pos_l =[450.00, 10.0, 370.0, 90.01, 179.00, 90.01] # 시작점L
switch_pos_up = [34.10, 12.19, 80.16, 1.13, 87.46, 33.98] # 스위치 위에
switch_left = [362.06, 265.96, 90.59, 153.69, 178.69, 153.43] # 스위치 왼쪽
switch_right = [392.97, 265.94, 90.58, 153.75, 178.92, 153.49] # 스위치 오른쪽
hammer_up = [28.57, 26.96, 51.28, -0.11, 101.76, -60.34] # 망치 위
hammer_up_up = [28.60, 30.79, 35.13, -0.13, 114.08, -60.35] # 망치 위의 위
hammer_hand = [476.17, 267.14, 164.69, 29.99, -180.0, -59.02] # 망치 손잡이
# glasses_depart_up = [-2.73, 25.14, 62.74, -0.10, 92.00, -6.62] # 안경 도착지 위
# glasses_depart = [541.92, -18.80, 77.92, 177.45, -180.00, 173.44] # 안경 도착지
glasses_depart_up = [-52.02, 31.90, 45.95, -0.12, 102.15, 38.07] # 안경 도착지 위
glasses_depart = [360.0, -450.01, 70.00, 130.66, -180.0, -139.34] # 안경 도착지
pillow_arrange_up = [18.88, 12.64, 94.55, -0.06, 72.95, 109.41] # 베개 정리 위치
pillow_arrange = [413.45, 148.99, 74.95, 18.32, -179.85, 108.70] # 베개 정리 위치
new_sect1=[445, -11, 300, 0, -180, 0]
# sect2 yaw ->-180
new_sect3=[420, -304, 286, 32, 180, -145]
swing_set = [0, 0, 0, 0, 10, -90]
swing_1 = [0, 0, 0, 0, -15, 0]
swing_2 = [0, 0, 0, 0, 15, 0]
bedding_check = [-31.21, 12.26, 91.59, -72.60, 14.71, -13.28] # 이불 잡았는 지 확인 위치

rclpy.init()
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
DR_init.__dsr__node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)

try:
    from DSR_ROBOT2 import (
        movej, movel, get_current_posx, trans, task_compliance_ctrl,
          set_desired_force, release_force, release_compliance_ctrl, DR_FC_MOD_REL, 
          check_force_condition, move_periodic, DR_AXIS_Z, DR_TOOL)
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.light_is_on = True
        self.init_robot()
        self.callback_group = ReentrantCallbackGroup()
        self.create_timer(1.0, self.robot_control, callback_group=self.callback_group)
        self.hammer_stop_event = threading.Event()

        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position", callback_group=self.callback_group
        )

        self.get_keyword_client = self.create_client(
            Trigger, "/get_keyword", callback_group=self.callback_group
        )

        self.motion_lock = threading.Lock()
        self.motion_queue = Queue()
        self.create_timer(0.1, self.process_motion_queue)
        self.alarm_player = SoundPlayer(self.get_logger(), label="Alarm")
        self.asmr_player = SoundPlayer(self.get_logger(), label="ASMR")
        self.announcement_player = SoundPlayer(self.get_logger(), label="Announcement")  # 추가
        self.registered_alarms = []
        threading.Thread(target=self.alarm_checker_loop, daemon=True).start()
        self.alarm_thread = None
        self.alarm_stop_event = threading.Event()
        self.alarm_active = False # 알람 미리 종료가 안되서 추가
        self.alarm_playing = False
        self.already_triggered = False  # 중복 실행 방지용 플래그
        self.alarm_gui = self.create_client(ExampleInterface, 'set_alarm_time', callback_group=self.callback_group)
        self.alarm_gui_request = ExampleInterface.Request()
        self.find_glasses = False
        self.log_pub = self.create_publisher(String, '/robot_log', 10) # 발표 전 수정

        try:
            pygame.mixer.init()
        except Exception as e:
            self.get_logger().error(f"pygame mixer 초기화 실패: {e}")

        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

        self.alarm_gui = self.create_client(ExampleInterface, '/set_alarm_time', callback_group=self.callback_group)
        self.alarm_gui_request = ExampleInterface.Request()
        self.find_glasses = False
    
    def log_to_topic(self, msg): # 발표 전 수정
        self.log_pub.publish(String(data=msg))
        self.get_logger().info(msg)  # 기존 로거 유지하고 싶다면

    def send_alarm_request(self, hour, minute, flag):
        self.alarm_gui_request.hour = hour
        self.alarm_gui_request.minute = minute
        self.alarm_gui_request.flag = flag
        future = self.alarm_gui.call_async(self.alarm_gui_request)

        def callback(fut):
            try:
                result = fut.result()
                if result:
                    self.get_logger().info(f"응답 수신: success={result.success}, message='{result.message}'")
                else:
                    self.get_logger().error("서비스 응답이 None입니다.")
            except Exception as e:
                self.get_logger().error(f"요청 중 예외 발생: {e}")

        future.add_done_callback(callback)

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]
    def register_alarm(self, hour, minute, second=0):
        alarm = (hour, minute, second)
        if alarm not in self.registered_alarms:
            self.registered_alarms.append(alarm)
            self.get_logger().info(f"알람 등록됨: {hour:02d}:{minute:02d}:{second:02d}")
            self.log_to_topic(f"알람 등록됨: {hour:02d}:{minute:02d}:{second:02d}") # 발표 전 수정
        else:
            self.get_logger().info(f"이미 등록된 알람입니다: {hour:02d}:{minute:02d}:{second:02d}")
            self.log_to_topic(f"이미 등록된 알람입니다: {hour:02d}:{minute:02d}:{second:02d}") # 발표 전 수정

    def play_announcement(self):
        if not self.alarm_player.is_playing() and not self.asmr_player.is_playing() and not self.announcement_player.is_playing():
            self.announcement_player.play(
                '/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/sound/alarm_registration.mp3',
                loop=False
            )
        else:
            self.get_logger().info("재생 중인 사운드가 있어 안내 음성 생략")

    def alarm_checker_loop(self):
        triggered = set()
        while True:
            try:
                now = time.localtime()
                current = (now.tm_hour, now.tm_min, now.tm_sec)

                for alarm_time in self.registered_alarms:
                    # 현재 시간과 알람 시간 차이 계산 (초 단위)
                    alarm_sec = alarm_time[0]*3600 + alarm_time[1]*60 + alarm_time[2]
                    current_sec = current[0]*3600 + current[1]*60 + current[2]
                    if abs(alarm_sec - current_sec) <= 1 and alarm_time not in triggered:
                        self.get_logger().info(f"알람 트리거됨: {alarm_time[0]:02d}:{alarm_time[1]:02d}:{alarm_time[2]:02d}")

                        if self.asmr_player.is_playing():
                            self.get_logger().info("알람으로 인해 ASMR 중단")
                            self.log_to_topic("알람으로 인해 ASMR 중단") # 발표 전 수정

                            self.asmr_player.stop()

                        # 알람 동작은 별도 스레드로 실행
                        threading.Thread(target=self.run_alarm_sequence, daemon=True).start()

                        triggered.add(alarm_time)
            except Exception as e:
                self.get_logger().error(f"[alarm_checker_loop] 내부 예외 발생: {e}")
            time.sleep(1)

    def run_alarm_sequence(self):
        with open("/tmp/alarm_run_log.txt", "a") as f:
            f.write(">>> run_alarm_sequence 진입\n")

        self.get_logger().info("[알람] run_alarm_sequence() 진입")
        try:
            self.get_logger().info("[알람] run_alarm_sequence() 진입")
            time.sleep(0.1)

            self.get_logger().info("조명 켜기 요청 시작")
            self.log_to_topic("조명 켜기 요청 시작") # 발표 전 수정
            self.motion_queue.put("light_on")

            self.get_logger().info("망치 동작 요청 시작")
            self.log_to_topic("알람 시작") # 발표 전 수정
            self.motion_queue.put("hammer_action")

            self.get_logger().info("알람 사운드 재생 시도 중...")
            self.alarm_player.play(
                "/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/sound/alarm.mp3",  # 경로 수정해 주세요
                loop=True
            )
            self.get_logger().info("알람 사운드 재생 호출 완료")

        except Exception as e:
            self.get_logger().error(f"[run_alarm_sequence] 예외 발생: {e}")

    def cancel_alarm(self, hour=None, minute=None):
        if hour is not None and minute is not None:
            removed = False
            for alarm in self.registered_alarms:
                if alarm[0] == hour and alarm[1] == minute:
                    self.registered_alarms.remove(alarm)
                    self.get_logger().info(f"알람 {hour:02d}:{minute:02d} 취소됨")
                    self.log_to_topic(f"알람 {hour:02d}:{minute:02d} 취소됨") # 발표 전 수정

                    removed = True
                    break
            if not removed:
                self.get_logger().info(f"해당 알람은 등록되어 있지 않음")
        else:
            self.get_logger().info("전체 알람 취소")
            self.log_to_topic("전체 알람 취소") # 발표 전 수정

            self.registered_alarms.clear()

        self.alarm_player.stop()

    def play_asmr(self, sound_index): # 발표 전 수정
        if self.alarm_player.is_playing():
            self.get_logger().warn("현재 알람이 재생 중입니다. ASMR 요청을 무시합니다.")
            self.log_to_topic("현재 알람이 재생 중입니다. ASMR 요청을 무시합니다.") # 발표 전 수정
            return
        
        sounds = {
            "0": '/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/sound/fire.mp3',
            "1": '/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/sound/Dream.mp3',
            "2": '/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/sound/water.mp3',
        } # 경로는 절대 경로로 하는게 안전할 듯

        if sound_index not in sounds:
            self.get_logger().warn(f"지원하지 않는 ASMR 번호: {sound_index}")
            return
        
        path = sounds.get(sound_index)
        if not path:
            self.get_logger().warn(f"지원하지 않는 ASMR 번호: {sound_index}")
            return
        self.asmr_player.play(path, loop=True, duration=600)  # 10분 자동 종료

    def stop_asmr(self):
        self.asmr_player.stop()

    def light_on(self):
        try:
            self.light_is_on = True
            movej(init_pos, vel=VELOCITY, acc=ACC) # 최초위치로 가고
            gripper.close_gripper() # 그리퍼 닫기
            time.sleep(0.5)
            movej(switch_pos_up, vel=VELOCITY, acc=ACC) # 스위치 위까지 이동
            movel(switch_left, vel=VELOCITY, acc=ACC) # 스위치 오른쪽 위까지 이동
            time.sleep(0.1)
            task_compliance_ctrl(stx=[1000.0,1000.0, 1000.0, 1000.0,1000.0, 1000.0]) # 순응
            time.sleep(0.1)
            set_desired_force([0,0,-20,0,0,0],[0,0,1,0,0,0],mod = DR_FC_MOD_REL) # z축 방향으로 외력
            time.sleep(0.1)
            while check_force_condition(DR_AXIS_Z, max=14)==0:
                pass
            release_force()
            time.sleep(0.1)
            release_compliance_ctrl()
            time.sleep(0.1)
            movej(switch_pos_up, vel=VELOCITY, acc=ACC)
            movej(init_pos, vel=VELOCITY, acc=ACC) # 최초위치로 가고
        finally:
            self.motion_lock.release()

    def light_off(self):
        try:
            self.light_is_on = False
            movej(init_pos, vel=VELOCITY, acc=ACC) # 최초위치로 가고
            gripper.close_gripper() # 그리퍼 닫기
            time.sleep(0.5)
            movej(switch_pos_up, vel=VELOCITY, acc=ACC) # 스위치 위까지 이동
            movel(switch_right, vel=VELOCITY, acc=ACC) # 스위치 왼쪽 위까지 이동
            time.sleep(0.1)
            task_compliance_ctrl(stx=[1000.0,1000.0, 1000.0, 1000.0,1000.0, 1000.0]) # 순응
            time.sleep(0.1)
            set_desired_force([0,0,-20,0,0,0],[0,0,1,0,0,0],mod = DR_FC_MOD_REL) # z축 방향으로 외력
            time.sleep(0.1)
            while check_force_condition(DR_AXIS_Z, max=12)==0:
                pass
            release_force()
            time.sleep(0.1)
            release_compliance_ctrl()
            time.sleep(0.1)
            movej(switch_pos_up, vel=VELOCITY, acc=ACC)
            movej(init_pos, vel=VELOCITY, acc=ACC) # 최초 위치
        finally:
            self.motion_lock.release()
            
    def pick_bedding(self):
        self.get_logger().info("이불 좌표 탐색 시도 중...")
        self.log_to_topic("이불 정리 시작") # 발표 전 수정
        self.get_position_request.target = "bedding"
        future = self.get_position_client.call_async(self.get_position_request)
        rclpy.spin_until_future_complete(self, future)        
        result = future.result()
        if result:
            bbox = result.bbox  # 예: [x_min, y_min, x_max, y_max]
            depth_img = self.latest_depth_image  # 저장해둔 depth image
            camera_model = self.camera_model     # deprojection을 제공하는 객체            
            far_point = self.get_farthest_corner(bbox, depth_img, camera_model)
            if far_point is None:
                self.get_logger().warn("유효한 꼭짓점이 없습니다.")
                return None            # 변환: 카메라 → 베이스 좌표계
            td_coord = self.transform_to_base(
                list(far_point),
                os.path.join(package_path, "resource", "T_gripper2camera.npy"),
                get_current_posx()[0]
            )            
            if len(td_coord) >= 3 and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)
                target_pos = list(td_coord[:3]) + get_current_posx()[0][3:]
                return target_pos
        else:
            self.get_logger().warn("이불 위치 인식 실패")
            self.log_to_topic("이불 위치 인식 실패") # 발표 전 수정
            return None
    def glasses_process(self):
        def on_glasses_pos_ready(target_pos):
            if target_pos:
                self.get_logger().warn("안경 찾기 시작!!.")
                self.log_to_topic("안경 찾기 시작") # 발표 전 수정
                self.pick_and_place_glasses(target_pos)
            else:
                self.get_logger().warn("안경 위치를 찾을 수 없습니다.")
                self.log_to_topic("안경 위치를 찾을 수 없습니다.") # 발표 전 수정

        movel(new_sect1, vel=VELOCITY, acc=ACC)
        self.get_logger().warn(f"{self.find_glasses}")
        self.get_glasses_pos_async("glasses", on_glasses_pos_ready)
        time.sleep(7)
        if self.find_glasses==True:
            self.get_logger().warn(f"{self.find_glasses}, 리턴")
            return
        
        self.get_logger().warn(f"{self.find_glasses}")
        movej([0,0,0,0,0,-180], vel=VELOCITY, acc=ACC, mod=1)
        self.get_glasses_pos_async("glasses", on_glasses_pos_ready)
        time.sleep(7)
        if self.find_glasses==True:
            self.get_logger().warn(f"{self.find_glasses}, 리턴")
            return
        
    def process_motion_queue(self):
        if self.motion_lock.locked():
            return

        if not self.motion_queue.empty():
            action = self.motion_queue.get()
            self.get_logger().info(f"큐 실행: {action}, 남은 큐: {self.motion_queue.qsize()}")

            def run_action():
                self.motion_lock.acquire()
                try:
                    if action == "light_on" and not self.light_is_on:
                        self.light_on()
                    elif action == "light_off" and self.light_is_on:
                        self.light_off()
                    elif action == "bedding_action":
                        self.on_bedding_pos_ready()  # release는 콜백 내부에서
                    elif action == "pillow_action":
                        self.on_pillow_pos_ready()
                    elif action == "hammer_action":
                        self.hammer_action()
                    elif action == "glasses":
                        self.glasses_process()
                        self.motion_lock.release()
                    else:
                        self.get_logger().warn(f"알 수 없는 동작: {action}")
                        self.motion_lock.release()
                except Exception as e:
                    self.get_logger().error(f"실행 중 예외: {e}")
                    self.motion_lock.release()

            threading.Thread(target=run_action, daemon=True).start()

    def handle_command(self, target_list):
        self.get_logger().info("기능시작")
        self.get_logger().info(f"타겟 리스트:{target_list}")

        try:
            if not target_list:
                return
            
            cmd = target_list[0].strip()

            if cmd == '0':  # 지정 시간 알람
                hour = int(target_list[1] + target_list[2])
                minute = int(target_list[3] + target_list[4])
                second = 0  # 초는 기본적으로 0초에 울리도록 설정
                self.send_alarm_request(hour, minute, 0)
                self.register_alarm(hour, minute, second)
                self.motion_queue.put("light_off")
                self.get_logger().info(f'알람등록 :{hour}시 {minute}분 {second}초')
                self.play_announcement()

            elif cmd == '1':
                now = time.time()
                delta_min = int(target_list[3] + target_list[4])
                target_time = time.localtime(now + delta_min * 60)

                hour = target_time.tm_hour
                minute = target_time.tm_min
                second = target_time.tm_sec
                self.send_alarm_request(hour, minute, 1)
                self.register_alarm(hour, minute, second)
                self.motion_queue.put("light_off")
                self.get_logger().info(f'알람등록 :{hour:02d}시 {minute:02d}분 {second:02d}초')
                self.play_announcement()

            elif cmd == '2': # asmr 재생 # 발표 전 수정
                if self.alarm_player.is_playing():
                    self.get_logger().warn("현재 알람이 재생 중이므로 ASMR 요청을 무시합니다.")
                    return  # ASMR 요청 무시
                self.play_asmr(target_list[1])

            elif cmd == '3': # asmr 종료
                self.stop_asmr()

            elif cmd == '4': # 알람 취소 및 기상
                self.cancel_alarm()
                self.motion_queue.put("light_on")

            elif cmd == '5': # 알람으로 일어났을 시 # 발표 전 수정
                self.get_logger().info('알람 멈춤')
                self.cancel_alarm()
                self.log_to_topic("알람 멈춤") # 발표 전 수정
                self.get_logger().info('망치 멈춤')
                self.hammer_stop_event.set()
                self.log_to_topic("물리 알람 멈춤") # 발표 전 수정

            elif cmd == '6': # 안경
                self.find_glasses= False
                self.motion_queue.put("glasses")

            elif cmd == '7': # 날씨
                API_KEY = "V4nblHN9fhu9M6RsJ39UWoUeKYFxqJiCtncteqq12VQm206vlC9z9nGn9WmPAimaan/2YbuluBdNVV5WkrFmXA=="
                wf = WeatherForecast(API_KEY)
                summary = wf.get_weather_summary(37.495, 126.856)
                self.get_logger().info(f'{summary}')

            elif cmd == '8':  # 특정 알람 취소 # 프롬프트 수정해야 함
                hour = int(target_list[1] + target_list[2])
                minute = int(target_list[3] + target_list[4])
                self.cancel_alarm(hour, minute)            
                
            elif cmd == '9':  # 전체 알람 취소 # 프롬프트 수정해야 함
                self.cancel_alarm()

            elif cmd == 'A' or cmd == 'a': #이불

                self.motion_queue.put("pillow_action")
                self.get_logger().info('베개 추가 완료')
                time.sleep(1)
                self.motion_queue.put("bedding_action")
                self.get_logger().info('이불 추가 완료')

            else: # 발표 전 수정
                self.get_logger().info('실패') # 여기에 무슨말인지 모르겠다는 음성 파일 넣음면 됨
                if not self.asmr_player.is_playing() and not self.alarm_player.is_playing():
                    pygame.mixer.music.load('/home/rokey/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/sound/reanswer.mp3') # 재물음 음성파일 추가
                    pygame.mixer.music.play(loops=1)
        finally:
        # 어떤 작업이든 끝났으면 다음 명령 받을 수 있도록 초기화
            self.already_triggered = False

    def on_bedding_pos_ready(self):
        def on_pos_ready(target_pos):
            if target_pos is None:
                self.get_logger().info("이불 객체 탐색 실패")
                
                if self.motion_lock.locked():
                    self.motion_lock.release()                
                return
            if target_pos[0] ==0.0:
                self.get_logger().info(f'{target_pos}')
                self.get_logger().info("이불 객체 탐색 실패.")
                if self.motion_lock.locked():
                    self.motion_lock.release()
                return

            try:
                self.get_logger().info("이불 위치 탐색 성공. 이불 정리 동작 시작!")
                self.log_to_topic("이불 정리 동작 시작") # 발표 전 수정
                gripper.open_gripper()
                time.sleep(1)
                current_ori = get_current_posx()[0][3:]

                approach_pose = target_pos.copy()
                approach_pose[2] += 20
                approach_pose[3:] = current_ori
                movel(approach_pose, vel=VELOCITY, acc=ACC)
                approach_pose[2] -= 20
                time.sleep(1.5)

                # 2. 이불 잡기
                grasp_pose = target_pos.copy()
                grasp_pose[3:] = current_ori
                if grasp_pose[2] <=61:
                    self.get_logger().info("z값이 너무 낮아 쫌 높여요")
                    grasp_pose[2] = 61.0
                movel(grasp_pose, vel=VELOCITY, acc=ACC)
                gripper.close_gripper()
                while gripper.get_status()[0]:
                    time.sleep(0.5)

                time.sleep(5)

                # 3. 들어올리기
                lift_pose = target_pos.copy()

                lift_pose[2] += 250
                lift_pose[3:] = current_ori

                movel(lift_pose, vel=VELOCITY, acc=ACC)
                time.sleep(1)               
                
                 # 4. 흔들기
                movej(swing_set, vel=VELOCITY, acc=ACC, mod=1)
                for i in range(3):
                    movej(swing_1, vel=VELOCITY, acc=ACC, mod=1)
                    movej(swing_2, vel=VELOCITY, acc=ACC, mod=1)
                movej(init_pos, vel=VELOCITY, acc=ACC)
                # time.sleep(1)                
                
                # 5. 놓기
                init_pos_l[1] += 100
                movel(init_pos_l, vel=VELOCITY, acc=ACC)
                gripper.close_gripper()  

                init_pos_l[2] -= 180
                movel(init_pos_l, vel=VELOCITY, acc=ACC)
                init_pos_l[1] -= 200
                init_pos_l[2] -= 50
                movel(init_pos_l, vel=VELOCITY, acc=ACC)
                gripper.open_gripper()
                time.sleep(1)

                movej(init_pos, vel=VELOCITY, acc=ACC)

            except Exception as e:
                self.get_logger().error(f"이불 동작 중 예외 발생: {e}")
            finally:
                self.motion_lock.release()

        self.get_target_pos_async("bedding", on_pos_ready)

    def on_pillow_pos_ready(self):
        def on_pos_ready(target_pos, retry = False):
            if target_pos is None:
                self.get_logger().info("베개 객체 탐색 실패")
                
                if self.motion_lock.locked():
                    self.motion_lock.release()                
                return
            if target_pos[0] ==0.0:
                self.get_logger().info(f'{target_pos}')
                self.get_logger().info("베개 객체 탐색 실패.")
                if self.motion_lock.locked():
                    self.motion_lock.release()
                return

            try:
                self.log_to_topic("베개 정리 동작 시작") # 발표 전 수정
                gripper.open_gripper()
                time.sleep(1)
                current_ori = get_current_posx()[0][3:]

                # 1. 접근 위치
                approach_pose = target_pos.copy()
                approach_pose[2] += 20
                # approach_pose[3:] = current_ori
                movel(approach_pose, vel=VELOCITY, acc=ACC)
                approach_pose[2] -= 20
                time.sleep(1.5)

                # 2. 베개 잡기
                grasp_pose = target_pos.copy()
                # grasp_pose[3:] = current_ori
                grasp_pose[2] -= 40
                movel(grasp_pose, vel=VELOCITY, acc=ACC)
                gripper.close_gripper()
                time.sleep(3)

                if gripper.get_status()[1] == 0:
                    self.get_logger().warn("베개를 잡지 못했습니다.")
                    movej(init_pos, vel=VELOCITY, acc=ACC)

                    if not retry:
                        self.get_logger().warn("베개 잡기 재시도 중...")
                        on_pos_ready(target_pos, True)
                    else:
                        self.get_logger().error("재시도에도 베개를 잡지 못했습니다. 종료합니다.")
                        
                    return


                # 3. 들어올리기
                lift_pose = target_pos.copy()
                lift_pose[2] += 200
                lift_pose[3:] = current_ori
                movel(lift_pose, vel=VELOCITY, acc=ACC)
                time.sleep(1)
                


                # 4. 베개 놓을 위치로 이동
                movej(pillow_arrange_up, vel=VELOCITY, acc=ACC)
                movel(pillow_arrange, vel=VELOCITY, acc=ACC)
                # 5. 그리퍼 열기
                gripper.open_gripper()
                time.sleep(1)


                # 6. 초기 위치로 복귀
                movej(init_pos, vel=VELOCITY, acc=ACC)

            except Exception as e:
                self.get_logger().error(f"베개 동작 중 예외 발생: {e}")
            finally:
                if self.motion_lock.locked():
                    self.motion_lock.release()

        self.get_target_pos_async("pillow", on_pos_ready)

    def robot_control(self):
        if self.already_triggered:
            return    
        future = self.get_keyword_client.call_async(self.get_keyword_request)
        future.add_done_callback(self.keyword_response_callback)
    
    def keyword_response_callback(self, future):
        try:
            result = future.result()
            # self.get_logger().info(f":흰색_확인_표시: 응답 수신: success={result.success}, message='{result.message}'")        
            if result.success:
                target_list = result.message.split()
                # self.get_logger().info(f":메모: 명령어 파싱: {target_list}")
                self.already_triggered = True            
                threading.Thread(
                    target=self.handle_command,
                    args=(target_list,),
                    daemon=True
                ).start()
            else:
                self.get_logger().warn("음성 인식 실패. 아무 명령도 실행되지 않음")    
        except Exception as e:
            self.get_logger().error(f"keyword_response_callback 에러: {e}")
        
    def clean_hammer(self):
        # 초기 위치로
        movej(init_pos, vel=VELOCITY, acc=ACC)
        # 망치를 +z을 해야 베개랑 충돌 방지
        movej(hammer_up_up, vel=VELOCITY, acc=ACC)
        movej(hammer_up, vel=VELOCITY, acc=ACC)
        movel(hammer_hand, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        time.sleep(1)
        movej(hammer_up, vel=VELOCITY, acc=ACC)
        # 초기 위치로
        movej(init_pos, vel=VELOCITY, acc=ACC)

    def hammer_action(self):
        self.hammer_stop_event.clear()
        movej(init_pos, vel=VELOCITY, acc=ACC)

        def on_pos_ready(target_pos):
            if target_pos is None:
                self.get_logger().warn('사람 객체 탐색 실패')
                self.motion_lock.release()
                return

            try:
                self.get_logger().info(f'탐색된 사람 좌표: {target_pos}')
                gripper.open_gripper()
                time.sleep(1)

                if self.hammer_stop_event.is_set():
                    self.clean_hammer()
                    return

                # 망치 잡기
                movej(hammer_up, vel=VELOCITY, acc=ACC)
                movel(hammer_hand, vel=VELOCITY, acc=ACC)
                gripper.close_gripper()
                time.sleep(1)

                if self.hammer_stop_event.is_set():
                    gripper.open_gripper()
                    time.sleep(1)
                    movej(hammer_up, vel=VELOCITY, acc=ACC)
                    movej(init_pos, vel=VELOCITY, acc=ACC)
                    return

                # 망치 옮기기
                movej(hammer_up_up, vel=VELOCITY, acc=ACC)
                movej(init_pos, vel=VELOCITY, acc=ACC)
                target_pos[0] -= 80
                target_pos[2] += 140
                movel(target_pos, vel=VELOCITY, acc=ACC)
                movej(swing_set, vel=VELOCITY, acc=ACC, mod=1)
                time.sleep(1)

                # 망치질
                for i in range(100):
                    if self.hammer_stop_event.is_set():
                        self.get_logger().info("망치질 중단됨")
                        break
                    movej(swing_1, vel=60, acc=60, mod=1)
                    movej(swing_2, vel=60, acc=60, mod=1)

                self.clean_hammer()

            except Exception as e:
                self.get_logger().error(f"망치 동작 중 예외 발생: {e}")
            finally:
                self.motion_lock.release()

        self.get_target_pos_async("human", on_pos_ready)

    def get_target_pos_async(self, target, callback):
        self.get_position_request.target = target
        future = self.get_position_client.call_async(self.get_position_request)

        def on_result(fut):
            try:
                result = fut.result()
                if result is not None:
                    x = result.depth_position.tolist()[0]
                    theta = fut.result().theta

                    if x ==0.0:
                        target_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    else:
                        td_coord = self.transform_to_base(
                            result.depth_position.tolist(),
                            os.path.join(package_path, "resource", "T_gripper2camera.npy"),
                            get_current_posx()[0]
                        )
                        if td_coord[2] and sum(td_coord) != 0:
                            td_coord[2] += DEPTH_OFFSET
                            td_coord[2] = max(td_coord[2], MIN_DEPTH)
                        target_pos = list(td_coord[:3]) + get_current_posx()[0][3:]
                        target_pos[5] = theta
                    
                    callback(target_pos)
                else:
                    callback(None)
            except Exception as e:
                self.get_logger().error(f"get_target_pos_async error: {e}")
                callback(None)
        future.add_done_callback(on_result)

    def get_glasses_pos_async(self, target, callback):
        self.get_position_request.target = "glasses"
        future = self.get_position_client.call_async(self.get_position_request)
        def on_result(fut):
            try:
                result = fut.result()
                if result is not None:
                    x = result.depth_position.tolist()[0]
                    theta = fut.result().theta

                    if x ==0.0:
                        target_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    else:

                        td_coord = self.transform_to_base(
                            result.depth_position.tolist(),
                            os.path.join(package_path, "resource", "T_gripper2camera.npy"),
                            get_current_posx()[0]
                        )
                        if td_coord[2] and sum(td_coord) != 0:
                            td_coord[2] += DEPTH_OFFSET
                            td_coord[2] = max(td_coord[2], MIN_DEPTH)
                        target_pos = list(td_coord[:3]) + get_current_posx()[0][3:]
                        target_pos[5] = result.theta

                    callback(target_pos)
                else:
                    callback(None)
            except Exception as e:
                self.get_logger().error(f"get_target_pos_async error: {e}")
                callback(None)
        future.add_done_callback(on_result)


    def get_bedding_pos_async(self, target, callback):
        self.get_logger().info("이불 좌표 비동기 요청 중...")
        self.get_position_request.target = "bedding"
        future = self.get_position_client.call_async(self.get_position_request)
        def on_result(fut):

            try:
                result = fut.result()
                if result is not None:
                    td_coord = self.transform_to_base(
                        result.depth_position.tolist(),
                        os.path.join(package_path, "resource", "T_gripper2camera.npy"),
                        get_current_posx()[0]
                    )
                    if td_coord[2] and sum(td_coord) != 0:
                        td_coord[2] += DEPTH_OFFSET
                        td_coord[2] = max(td_coord[2], MIN_DEPTH)
                    target_pos = list(td_coord[:3]) + get_current_posx()[0][3:]
                    target_pos[5] = result.theta
                    callback(target_pos)
                else:
                    callback(None)
            except Exception as e:
                self.get_logger().error(f"get_target_pos_async error: {e}")
                callback(None)

        future.add_done_callback(on_result)

    def init_robot(self):
        movej(init_pos, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        time.sleep(0.1)

    def pick_and_place_target(self, target_pos):
        if target_pos[0] ==0.0:
            self.get_logger().info(f'{target_pos}')
            self.get_logger().info("타겟 포즈가 이상해서 종료합니다.")
            return
        else:
            self.get_logger().info(f'{target_pos}')
        
        robot_posx = get_current_posx()[0]
        robot_posx[5] = target_pos[5]
        movel(robot_posx, vel=VELOCITY, acc=ACC)
        movel(target_pos, vel=VELOCITY, acc=ACC)
        time.sleep(0.5)
        gripper.close_gripper()

        while gripper.get_status()[0]:
            time.sleep(0.5)
        
        time.sleep(0.5)
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)

    def pick_and_place_glasses(self, target_pos, retry=False):
        robot_posx = get_current_posx()[0]
        if target_pos[0] ==0.0:
            self.get_logger().info(f'{target_pos}')
            self.get_logger().info("타겟 포즈가 이상해서 종료합니다. (송주훈이 수정한 부분. 만약에 이게 떠서 작동을 안하면 부르세요)")
            return
        elif target_pos[2] >= robot_posx[2]:
            self.get_logger().info(f'{target_pos}')
            self.get_logger().warn('전달 받은 z값이 너무 높아 default depth 값인 62로 설정됩니다.')
            target_pos[2] = 62
        else:
            self.get_logger().info(f'{target_pos}')
        self.find_glasses=True
        

        if target_pos[2] <=61.5:
            target_pos[2] = 62
       
        robot_posx[5] = target_pos[5]
        movel(robot_posx, vel=VELOCITY, acc=ACC)
        movel(target_pos, vel=VELOCITY, acc=ACC)
        time.sleep(0.5)

        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(1)
        time.sleep(1)

        if gripper.get_status()[1] == 0:  # 물건을 못 잡았을 때
            self.get_logger().warn("물건을 잡지 못했습니다.")
            gripper.open_gripper()
            while gripper.get_status()[0]:
                time.sleep(0.5)
            time.sleep(1)
            movej(init_pos, vel=VELOCITY, acc=ACC)

            if not retry:
                self.get_logger().warn("pick_and_place_glasses 재시도 중...")
                target_pos[2] = 62
                self.pick_and_place_glasses(target_pos, retry=True)
            else:
                self.get_logger().error("재시도에도 물건을 잡지 못했습니다. 작업 종료.")
            return

        # 이동 동작 추가

        movej(glasses_depart_up, vel=30, acc=30)
        movel(glasses_depart, vel=VELOCITY, acc=ACC)


        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        time.sleep(1)
        movej(init_pos, vel=VELOCITY, acc=ACC)

def main(args=None):
    node = RobotController()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        DR_init.__dsr__node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

