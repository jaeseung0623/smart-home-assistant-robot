# import rclpy

class Pickbedding:
    def __init__(self, detect_blanket_func, get_position_client=None, get_position_request=None):
        self.already_triggered = False
        # self.get_logger = logger  # RcutilsLogger 객체
        self.detect_blanket = detect_blanket_func
        self.get_position_client = get_position_client
        self.get_position_request = get_position_request

    def pick_bedding(self):
        if self.already_triggered:
            return None

        # self.get_logger.info("🟡 이불 정리 시작")  # ✅ 괄호 X

        print("이불 정리 시작")

        # YOLO로 인식한 이불 좌표
        print('리스트 받기 전')
        blanket_bbox = self.detect_blanket
        print('리스트 받음')
        print(blanket_bbox)
        if blanket_bbox is None:
            # self.get_logger.error("이불 인식 실패")  # ✅ 괄호 X
            print("이불 인식 실패")
            return None

        u, v = blanket_bbox
        # self.get_logger.info(f"인식된 이불 위치: u={u}, v={v}")
        print(f"인식된 이불 위치: u={u}, v={v}")

        return (u, v)
