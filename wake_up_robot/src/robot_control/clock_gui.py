import time
import multiprocessing
import os
import rclpy
from rclpy.node import Node
from od_msg.srv import ExampleInterface
from .weather_forecast import WeatherForecast

def gui_alarm_process(hour, minute, flag):
    import tkinter as tk
    from tkinter import messagebox
    import threading
    import pygame

    def countdown_thread(seconds):
        while seconds >= 0:
            mins, secs = divmod(seconds, 60)
            hours, mins = divmod(mins, 60)
            time_str = f"{hours:02d}:{mins:02d}:{secs:02d}"
            countdown_label.config(text=time_str)
            time.sleep(1)
            seconds -= 1

        try:
            # pygame.mixer.music.load(sound_path)
            # pygame.mixer.music.play()
            messagebox.showinfo("알람", "알람 시간이 되었습니다!")
        except Exception as e:
            messagebox.showerror("오류", f"알람 사운드를 재생할 수 없습니다.\n{e}")

    def start_countdown():
        try:
            now = time.localtime()
            if flag==0:
                total_seconds = (hour * 3600 + minute * 60) - (now.tm_hour * 3600 + now.tm_min * 60 + now.tm_sec)
            elif flag==1:
                total_seconds = (hour * 3600 + minute * 60) - (now.tm_hour * 3600 + now.tm_min * 60 )
            if total_seconds <= 0:
                raise ValueError("시간은 현재보다 이후여야 합니다.")
            threading.Thread(target=countdown_thread, args=(total_seconds,), daemon=True).start()
        except ValueError:
            messagebox.showerror("입력 오류", "알람 시간이 현재 시각보다 빠르거나 이미 알람이 울리는 중입니다!")

    def update_current_time():
        now = time.strftime("%H:%M:%S")
        current_time_label.config(text=f"현재 시간: {now}")
        root.after(1000, update_current_time)

    def update_weather():
        try:
            new_summary = wf.get_weather_summary(37.495, 126.856)
            weather_label.config(text=new_summary)
        except Exception as e:
            weather_label.config(text=f"날씨 업데이트 오류: {e}")
        # 1시간마다 다시 호출
        root.after(3600000, update_weather)

        # 5분마다 다시 호출
        # root.after(300000, update_weather)
        
        # 5초마다 다시 호출
        # root.after(5000, update_weather)

    # GUI 구성
    pygame.mixer.init()
    base_path = os.path.dirname(os.path.abspath(__file__))
    # sound_path = os.path.join(base_path, 'sound/army.mp3')

    root = tk.Tk()
    root.title("자동 알람")
    root.geometry("420x320")
    root.configure(bg="black")

    current_time_label = tk.Label(root, text="", font=("Helvetica", 16), fg="white", bg="black")
    current_time_label.pack(pady=10)
    countdown_label = tk.Label(root, text="00:00:00", font=("Courier", 48), fg="lime", bg="black")
    countdown_label.pack(pady=10)

    api_key = "V4nblHN9fhu9M6RsJ39UWoUeKYFxqJiCtncteqq12VQm206vlC9z9nGn9WmPAimaan/2YbuluBdNVV5WkrFmXA=="
    wf = WeatherForecast(api_key, use_tts=False)
    weather_summary = wf.get_weather_summary(37.495, 126.856)  # 구로 좌표
    weather_label = tk.Label(root, text=weather_summary, font=("Helvetica", 13), fg="skyblue", bg="black", wraplength=420, justify="left")
    weather_label.pack(pady=10)

    update_current_time()
    update_weather()
    root.after(100, start_countdown)
    root.mainloop()

class AlarmServiceNode(Node):
    def __init__(self):
        super().__init__('alarm_service_node')
        self.srv = self.create_service(ExampleInterface, 'set_alarm_time', self.set_alarm_callback)
        self.get_logger().info('알람 설정 서비스 대기 중...')

    def set_alarm_callback(self, request, response):
        hour = request.hour
        minute = request.minute
        flag = request.flag

        self.get_logger().info(f'알람 요청 수신: {hour}시 {minute}분, flag = {flag}')

        # GUI를 새로운 프로세스로 실행
        p = multiprocessing.Process(target=gui_alarm_process, args=(hour, minute, flag))
        p.start()

        response.success = True
        response.message = f"{hour:02d}:{minute:02d} 알람 GUI가 실행되었습니다."
        return response


def main():
    rclpy.init()
    node = AlarmServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')  # macOS/windows 대응
    main()
