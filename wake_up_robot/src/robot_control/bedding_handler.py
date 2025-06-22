class BeddingHandler:
    def __init__(self, logger, detect_blanket_func):
        self.already_triggered = False
        self.get_logger = logger
        self.detect_blanket = detect_blanket_func  # 외부에서 함수 주입

    def pick_bedding(self):
        if self.already_triggered:
            return None

        self.get_logger().info("🟡 이불 정리 시작")

        # YOLO로 인식한 이불 좌표
        blanket_bbox = self.detect_blanket()
        if blanket_bbox is None:
            self.get_logger().error("이불 인식 실패")
            return None

        u, v = blanket_bbox
        self.get_logger().info(f"인식된 이불 위치: u={u}, v={v}")

        return (u, v)


    # 3D 위치 요청
    # self.get_position_request.u = int(u)
    # self.get_position_request.v = int(v)
    # future = self.get_position_client.call_async(self.get_position_request)
    # rclpy.spin_until_future_complete(self, future)

    # if not future.result():
    #     self.get_logger().error("3D 위치 응답 실패")
    #     return None

    # point = future.result().position
    # self.get_logger().info(f"이불 위치: x={point.x:.2f}, y={point.y:.2f}, z={point.z:.2f}")

    # return point
