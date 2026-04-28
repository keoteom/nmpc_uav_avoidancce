import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import px4_msgs.msg
from math import radians

class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_publisher')

        # TF 브로드캐스터 객체 생성
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # VehicleLocalPosition 메시지 구독
        self.vehicle_position_sub = self.create_subscription(
            px4_msgs.msg.VehicleLocalPosition, 
            '/fmu/out/vehicle_local_position_v1', 
            self.vehicle_position_callback, 
            10
        )
        
        # 타이머를 설정하여 주기적으로 변환을 퍼블리시
        self.timer = self.create_timer(0.1, self.broadcast_transform)  # 10Hz로 변환을 브로드캐스트

        # 위치를 저장할 변수
        self.vehicle_position = None

    def vehicle_position_callback(self, msg):
        # VehicleLocalPosition 메시지에서 위치와 자세를 추출
        self.vehicle_position = msg

    def broadcast_transform(self):
        if self.vehicle_position is None:
            return  # 위치 정보가 없으면 변환을 브로드캐스트하지 않음
        
        # TransformStamped 객체 생성
        transform = TransformStamped()

        # 헤더 정보 설정
        transform.header.stamp = self.get_clock().now().to_msg()  # 현재 시간을 stamp로 설정
        transform.header.frame_id = 'odom'  # 부모 좌표계 (예: odom)
        transform.child_frame_id = 'base_link'  # 자식 좌표계 (예: base_link)

        # 위치 정보 (VehicleLocalPosition에서 x, y, z 값 추출)
        transform.transform.translation.x = self.vehicle_position.x
        transform.transform.translation.y = self.vehicle_position.y
        transform.transform.translation.z = self.vehicle_position.z

        # 회전 (쿼터니언)
        # 드론의 yaw 값 사용 (예시로 0으로 설정, 실제로는 `msg.q`나 `msg.yaw` 등을 사용할 수 있음)
        angle = radians(0)  # 예: 0도 회전
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # 회전 값 (단위는 쿼터니언)

        # 변환 브로드캐스트
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info(f"Publishing transform from 'odom' to 'base_link'")

def main(args=None):
    rclpy.init(args=args)
    dynamic_tf_publisher = DynamicTFPublisher()
    rclpy.spin(dynamic_tf_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
