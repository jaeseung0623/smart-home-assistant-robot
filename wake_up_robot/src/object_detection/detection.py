import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any, Callable, Optional, Tuple
from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from object_detection.realsense import ImgNode
from object_detection.yolo import YoloModel
import cv2

PACKAGE_NAME = 'pick_and_place_text'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)

class ObjectDetectionNode(Node):
    def __init__(self, model_name='yolo'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )
        self.get_logger().info("ObjectDetectionNode initialized.")

    def _load_model(self, name):
        """모델 이름에 따라 인스턴스를 반환합니다."""
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        """클라이언트 요청을 처리해 3D 좌표를 반환합니다."""
        self.get_logger().info(f"Received request: {request}")
        coords, theta = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        response.theta = theta
        self.get_logger().info(f"좌표 전송 완료. {response.depth_position}")
        return response

    def _compute_position(self, target):
        """이미지를 처리해 객체의 카메라 좌표를 계산합니다."""
        rclpy.spin_once(self.img_node)
        theta_rad = 90.0
        use_mask_center = target in ["glasses", "bedding", "pillow"]
        box, score, mask = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None:
            self.get_logger().warn("No detection found.")
            return (0.0, 0.0, 0.0), theta_rad
        self.get_logger().info(f"Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])

        if use_mask_center and mask is not None:
            original_h, original_w = 480, 640
            resized_mask = cv2.resize(mask.astype(np.uint8), (original_w, original_h), interpolation=cv2.INTER_NEAREST)
            binary_mask = (resized_mask > 0.5).astype(np.uint8) * 255
            
            if target == "glasses":
                edges = cv2.Canny(binary_mask, 20, 150, apertureSize=3)
                lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=10)
                if lines is not None:
                    max_len = 0
                    longest_line = None
                    for line in lines:
                        x1, y1, x2, y2 = line[0]
                        length = np.hypot(x2 - x1, y2 - y1)
                        if length > max_len:
                            max_len = length
                            longest_line = (x1, y1, x2, y2)
                    if longest_line:
                        x1, y1, x2, y2 = longest_line
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        theta_rad = np.arctan2(y2-y1, x2-x1)
                        theta_rad = np.degrees(theta_rad)
                        theta_rad -=90.0
                    else:
                        self.get_logger().info("직선이 없어서 theta를 못 받아옴")

                    theta_rad = round(theta_rad, 2)
                    self.get_logger().info(f'theta_rad: {theta_rad}')
            elif target == "pillow":
                edges = cv2.Canny(binary_mask, 50, 150, apertureSize=3)
                lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=30, maxLineGap=10)
                if lines is not None:
                    max_len = 0
                    longest_line = None
                    for line in lines:
                        x1, y1, x2, y2 = line[0]
                        length = np.hypot(x2 - x1, y2 - y1)
                        if length > max_len:
                            max_len = length
                            longest_line = (x1, y1, x2, y2)
                    if longest_line:
                        x1, y1, x2, y2 = longest_line
                        theta_rad = np.arctan2(y2-y1, x2-x1)
                        theta_rad = np.degrees(theta_rad)

                        theta_rad = round(theta_rad, 2)
                    else:
                        self.get_logger().info("직선이 없어서 theta를 못 받아옴")
                    self.get_logger().info('베개 각도 전달 했다')
                    self.get_logger().info(f'theta_rad: {theta_rad}')
                        
            # elif target == "bedding":
            #     indices = np.argwhere(binary_mask > 0)
            #     if len(indices) > 0:
            #         cy, cx = np.mean(indices, axis=0).astype(int)
            elif target == "bedding":
                indices = np.argwhere(binary_mask > 0)
                cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])  # 기본값

                if len(indices) > 0:
                    cy_center, cx_center = np.mean(indices, axis=0).astype(int)

                    # 코너 검출 (Harris corner 사용)
                    corners = cv2.cornerHarris(binary_mask, 2, 3, 0.04)
                    corners = cv2.dilate(corners, None)
                    threshold = 0.01 * corners.max()

                    corner_points = np.argwhere(corners > threshold)

                    if len(corner_points) > 0:
                        max_dist = -1
                        farthest_corner = (cx, cy)
                        for y, x in corner_points:
                            dist = np.hypot(cx_center - x, cy_center - y)
                            if dist > max_dist:
                                max_dist = dist
                                farthest_corner = (x, y)

                        cx, cy = farthest_corner  # 가장 먼 코너로 좌표 갱신
                        theta_rad = 90.0
                    else:
                        self.get_logger().warn("코너가 감지되지 않아 기본 중심점을 사용합니다.")
            elif target =="human":
                theta_rad = 0.0
        if target=="glasses":
            cz = self._get_glasses_depth(cx, cy, binary_mask)
        else:
            cz = self._get_depth(cx, cy)
        
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return (0.0, 0.0, 0.0), theta_rad
        return self._pixel_to_camera_coords(cx, cy, cz), theta_rad

    def _get_depth(self, x, y):
        """픽셀 좌표의 depth 값을 안전하게 읽어옵니다."""
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None
    def _get_glasses_depth(self, cx, cy, binary_mask):
        """중간점 주변 5x5 영역의 depth 값을 2픽셀 간격으로 안전하게 읽어오고, 두 번째로 작은 값을 반환합니다."""
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        depths = []
        region_size = 5
        gap = 2
        half_region = (region_size // 2) * gap

        for dx in range(-half_region, half_region + 1, gap):
            for dy in range(-half_region, half_region + 1, gap):
                px = cx + dx
                py = cy + dy

                if 0 <= px < binary_mask.shape[1] and 0 <= py < binary_mask.shape[0]:
                    if binary_mask[py, px] != 0:
                        try:
                            d = frame[py, px]
                            if d > 0:
                                depths.append(d)
                        except IndexError:
                            self.get_logger().warn(f"Coordinates ({px},{py}) out of range.")

        # 두 번째로 작은 값 반환
        if len(depths) >= 2:
            sorted_depths = sorted(depths)
            cz = sorted_depths[1]  # 두 번째로 작은 값
        elif depths:
            cz = depths[0]  # 값이 하나뿐이면 그 값 사용
        else:
            cz = 0.0  # 유효한 값 없음
    
        return cz


    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        """픽셀 좌표와 intrinsics를 이용해 카메라 좌표계로 변환합니다."""
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
