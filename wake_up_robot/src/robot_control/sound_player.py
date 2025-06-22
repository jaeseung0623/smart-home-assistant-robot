import threading
import time
import pygame

class SoundPlayer:
    def __init__(self, logger, label=""):
        self.logger = logger
        self.label = label
        self.stop_event = threading.Event()
        self.sound_thread = None
        self.monitor_thread = None
        self._is_playing = False
        pygame.mixer.init()

    def play(self, sound_path, loop=True, duration=None):
        self.stop()
        self.stop_event.clear()
        self.logger.info(f"[{self.label}] play() called with {sound_path}")

        def play_sound():
            try:
                pygame.mixer.music.load(sound_path)
                pygame.mixer.music.play(loops=-1 if loop else 0)
                self._is_playing = True
                self.logger.info(f"[{self.label}] 사운드 재생 시작")

                if loop:  # loop=True일 때만 모니터링
                    self.logger.info(f"[{self.label}] start_monitor() 호출")
                    self.start_monitor(sound_path)

                if duration:
                    time.sleep(duration)
                    if not self.stop_event.is_set():
                        self.logger.info(f"[{self.label}] 자동 종료됨 (duration={duration})")
                        self.stop()
            except Exception as e:
                self.logger.error(f"[{self.label}] 재생 실패: {e}")
                self._is_playing = False

        self.sound_thread = threading.Thread(target=play_sound, daemon=True)
        self.sound_thread.start()

    def start_monitor(self, sound_path):
        self.logger.info(f"[{self.label}] monitor_thread 생성 및 시작 시도")
        def monitor():
            self.logger.info(f"[{self.label}] monitor() 시작")
            while not self.stop_event.is_set() and self._is_playing:
                if not pygame.mixer.music.get_busy():
                    self.logger.warn(f"[{self.label}] 꺼짐 감지 - 재시작")
                    pygame.mixer.music.play(loops=-1)
                time.sleep(1)
            self.logger.info(f"[{self.label}] monitor() 종료됨")

        self.monitor_thread = threading.Thread(target=monitor, daemon=True)
        self.monitor_thread.start()

    def stop(self):
        self.logger.info(f"[{self.label}] stop() called")

        if self._is_playing:
            self.stop_event.set()
            pygame.mixer.music.stop()

            if self.sound_thread and self.sound_thread.is_alive():
                self.logger.info(f"[{self.label}] waiting for sound_thread to join...")
                self.sound_thread.join(timeout=1.0)
            self._is_playing = False
            self.logger.info(f"[{self.label}] 사운드 재생 종료됨")

    def is_playing(self):
        # 실시간으로 mixer 상태 확인
        if self._is_playing and not pygame.mixer.music.get_busy():
            self._is_playing = False
        return self._is_playing
