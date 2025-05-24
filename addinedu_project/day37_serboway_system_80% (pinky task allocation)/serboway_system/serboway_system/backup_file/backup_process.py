import rclpy
from rclpy.node import Node
import time

from my_msgs.srv import CheckOrderBuffer
from std_msgs.msg import Bool, String, Int32, Float32

class OrderBufferChecker(Node):
    def __init__(self):
        super().__init__('order_buffer_checker')
        
        # 서비스 클라이언트 생성
        self.cli = self.create_client(CheckOrderBuffer, 'check_order_buffer')
        
        # 상태 관리
        self.is_collecting_mode = False
        self.is_order_mode_active = False
        self.collect_start_time = None
        
        # 모든 구독자들과 발행자들을 노드 초기화 시점에 미리 생성
        self.create_publishers_and_subscribers()
        
        # Pinky 번호 매핑
        self.pinky_command_mapping = {
            'pinky_status': 5,   # Pinky 1
            'pinky_status2': 6,  # Pinky 2  
            'pinky_status3': 7   # Pinky 3
        }
        
        # Pinky 상태 추적을 위한 변수들
        self.pinky_status_values = {
            'pinky_status': None,
            'pinky_status2': None, 
            'pinky_status3': None
        }
        
        # Pinky 거리 추적을 위한 변수들
        self.pinky_distance_values = {
            'pinky_dist': None,
            'pinky_dist2': None,
            'pinky_dist3': None
        }
        
        # 서비스 연결 대기
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('[...] 서비스 서버 대기 중')
        
        # 3초마다 주문 상태 확인
        self.timer = self.create_timer(3.0, self.check_orders)
        self.get_logger().info('[✔] OrderBufferChecker Start')

    def create_publishers_and_subscribers(self):
        """모든 구독자와 발행자를 미리 생성"""
        # 발행자 생성
        self.make_command_pub = self.create_publisher(String, 'make_command', 10)
        
        # Pinky 명령 발행자들 생성
        self.pinky_command_pub = self.create_publisher(Int32, 'pinky_command', 10)
        self.pinky_task_pub = self.create_publisher(Int32, 'pinky_task', 10)
        
        # 상태 구독자들 생성
        self.maker_status_sub = self.create_subscription(
            Bool, 'maker_status', self.maker_status_callback, 10)
        
        self.pinky_status_sub = self.create_subscription(
            Int32, 'pinky_status', self.pinky_status_callback, 10)
        
        self.pinky_status2_sub = self.create_subscription(
            Int32, 'pinky_status2', self.pinky_status2_callback, 10)
        
        self.pinky_status3_sub = self.create_subscription(
            Int32, 'pinky_status3', self.pinky_status3_callback, 10)
        
        # 거리 구독자들 생성
        self.pinky_dist_sub = self.create_subscription(
            Float32, 'pinky_dist', self.pinky_dist_callback, 10)
        
        self.pinky_dist2_sub = self.create_subscription(
            Float32, 'pinky_dist2', self.pinky_dist2_callback, 10)
        
        self.pinky_dist3_sub = self.create_subscription(
            Float32, 'pinky_dist3', self.pinky_dist3_callback, 10)
        
        # 모니터링 상태 플래그들
        self.is_monitoring_pinky_status = False
        self.is_monitoring_pinky_distance = False
        
        # 거리 비교를 위한 대기 중인 Pinky 목록
        self.waiting_pinkys = []
        self.distance_collection_complete = False

    def check_orders(self):
        """주문 상태를 확인하고 적절한 동작 수행"""
        service_future = self.cli.call_async(CheckOrderBuffer.Request())
        service_future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        """서비스 응답 처리 콜백"""
        try:
            response = future.result()
            if response is not None:
                order_count = response.order_count
                is_collectable = (order_count == 0)  # 주문이 0개면 수거 가능
                
                #self.get_logger().info(f'수거 가능: {is_collectable}, 주문 수: {order_count}')
                
                if is_collectable:
                    self.handle_collect_mode()
                else:
                    # 주문 처리 모드가 아직 활성화되지 않았을 때만 실행
                    if not self.is_order_mode_active:
                        self.handle_order_mode()
                        self.is_order_mode_active = True
            else:
                self.get_logger().warn('서비스 응답이 None')
        except Exception as e:
            self.get_logger().error(f'서비스 응답 처리 중 오류: {e}')

    def handle_collect_mode(self):
        """수거 모드 처리"""
        # 주문 처리 모드에서 수거 모드로 전환 시 플래그 리셋
        if self.is_order_mode_active:
            self.is_order_mode_active = False
            self.reset_monitoring_flags()  # 모니터링 플래그 리셋
            
        if not self.is_collecting_mode:
            self.get_logger().info('버퍼 비어있음 - 30초 대기 후 수거 모드 전환')
            self.is_collecting_mode = True
            self.collect_start_time = time.time()
        else:
            # 30초 경과 확인
            if time.time() - self.collect_start_time >= 30:
                self.get_logger().info('30초 경과 - 수거 모드로 전환!')
                self.start_collection()

    def start_collection(self):
        """실제 수거 작업 시작"""
        self.get_logger().info('수거 작업 시작')
        # TODO: 추후 여기에 실제 수거 로직 구현
        # 예: 수거 서비스 호출, 로봇 이동 명령 등
        
        # 수거 완료 후 상태 리셋
        self.is_collecting_mode = False
        self.collect_start_time = None

    def handle_order_mode(self):
        """주문 처리 모드"""
        if self.is_collecting_mode:
            self.get_logger().info('주문 접수됨! 수거 모드 해제')
            self.is_collecting_mode = False
            self.collect_start_time = None
        
        self.get_logger().info('[🥪] 주문 처리 모드 - maker_status 모니터링 시작')
        # maker_status 콜백이 자동으로 호출되므로 별도 작업 불필요

    def maker_status_callback(self, msg: Bool):
        """maker_status 구독 콜백 (통합된 maker_command 기능)"""
        # 주문 모드가 활성화된 경우에만 처리
        if not self.is_order_mode_active:
            return
            
        if msg.data:
            # maker_status가 True (대기중) 이면 샌드위치 제작 명령 발행
            command_msg = String()
            command_msg.data = "Start Make Sandwich"
            self.make_command_pub.publish(command_msg)
            self.get_logger().info('[✔] maker_status: 대기중 → 샌드위치 제작 명령 Publish')
            
            # 샌드위치 제작 명령 발행 후 Pinky 상태 모니터링 시작
            self.start_pinky_status_monitoring()
        else:
            self.get_logger().info('[...] maker_status: 제작중 → 대기 중')

    def start_pinky_status_monitoring(self):
        """Pinky 상태 모니터링 시작"""
        self.get_logger().info('[🛼] Pinky Status Monitoring Start')
        self.is_monitoring_pinky_status = True

    def pinky_status_callback(self, msg: Int32):
        """pinky_status 구독 콜백"""
        if not self.is_monitoring_pinky_status:
            return
            
        self.pinky_status_values['pinky_status'] = msg.data
        status_text = self.get_pinky_status_text(msg.data)
        # self.get_logger().info(f'Pinky Status 1: {status_text} (값: {msg.data})') # Test한 후 주석처리하면 됨!
        self.check_waiting_pinky_count()

    def pinky_status2_callback(self, msg: Int32):
        """pinky_status2 구독 콜백"""
        if not self.is_monitoring_pinky_status:
            return
            
        self.pinky_status_values['pinky_status2'] = msg.data
        status_text = self.get_pinky_status_text(msg.data)
        # self.get_logger().info(f'Pinky Status 2: {status_text} (값: {msg.data})') # Test한 후 주석처리하면 됨!
        self.check_waiting_pinky_count()

    def pinky_status3_callback(self, msg: Int32):
        """pinky_status3 구독 콜백"""
        if not self.is_monitoring_pinky_status:
            return
            
        self.pinky_status_values['pinky_status3'] = msg.data
        status_text = self.get_pinky_status_text(msg.data)
        # self.get_logger().info(f'Pinky Status 3: {status_text} (값: {msg.data})') # Test한 후 주석처리하면 됨!
        self.check_waiting_pinky_count()
    
    def get_pinky_status_text(self, status_value):
        """Pinky 상태 값을 텍스트로 변환"""
        status_map = {
            0: "복귀 중/대기 중",
            1: "table1으로",
            2: "table2으로", 
            3: "table3으로",
            8: "음식수령위치로",
            9: "수거위치로"
        }
        return status_map.get(status_value, f"알 수 없는 상태({status_value})")
    
    def check_waiting_pinky_count(self):
        """대기 중인 Pinky 개수 확인 (Pinky 상태값이 0인 경우)"""
        waiting_count = 0
        waiting_pinkys = []
        
        for pinky_name, status_value in self.pinky_status_values.items():
            if status_value == 0:  # 복귀 중/대기 중
                waiting_count += 1
                waiting_pinkys.append(pinky_name)
        
        if waiting_count >= 1:
            self.get_logger().info(f'[✔] 사용가능한 Pinky: "{waiting_count}개" ({", ".join(waiting_pinkys)})')
            
            # 대기 중인 Pinky 목록 업데이트
            self.waiting_pinkys = waiting_pinkys
            self.distance_collection_complete = False
            
            # 거리 모니터링 시작
            self.start_distance_monitoring()
        
        return waiting_count

    def start_distance_monitoring(self):
        """Pinky 거리 모니터링 시작"""
        self.is_monitoring_pinky_distance = True

    def pinky_dist_callback(self, msg: Float32):
        """pinky_dist 구독 콜백"""
        if not self.is_monitoring_pinky_distance:
            return
            
        self.pinky_distance_values['pinky_dist'] = msg.data
        self.get_logger().info(f'[p1] Pinky-1 Distance: {msg.data:.2f}m')  # Test한 후 주석처리하면 됨!
        self.check_distance_collection_complete()

    def pinky_dist2_callback(self, msg: Float32):
        """pinky_dist2 구독 콜백"""
        if not self.is_monitoring_pinky_distance:
            return
            
        self.pinky_distance_values['pinky_dist2'] = msg.data
        self.get_logger().info(f'[p2] Pinky-2 Distance: {msg.data:.2f}m')  # Test한 후 주석처리하면 됨!
        self.check_distance_collection_complete()

    def pinky_dist3_callback(self, msg: Float32):
        """pinky_dist3 구독 콜백"""
        if not self.is_monitoring_pinky_distance:
            return
            
        self.pinky_distance_values['pinky_dist3'] = msg.data
        self.get_logger().info(f'[p3] Pinky-3 Distance: {msg.data:.2f}m')  # Test한 후 주석처리하면 됨!
        self.check_distance_collection_complete()

    def check_distance_collection_complete(self):
        """모든 대기 중인 Pinky들의 거리 데이터 수집 완료 확인"""
        if self.distance_collection_complete:
            return  # 이미 처리 완료
            
        # 대기 중인 Pinky들의 거리 데이터 매핑
        pinky_distance_mapping = {
            'pinky_status': 'pinky_dist',
            'pinky_status2': 'pinky_dist2', 
            'pinky_status3': 'pinky_dist3'
        }
        
        # 모든 대기 중인 Pinky들의 거리 데이터가 준비되었는지 확인
        all_distances_ready = True
        for waiting_pinky in self.waiting_pinkys:
            distance_key = pinky_distance_mapping.get(waiting_pinky)
            if distance_key and self.pinky_distance_values.get(distance_key) is None:
                all_distances_ready = False
                break
        
        if all_distances_ready and len(self.waiting_pinkys) > 0:
            self.get_logger().info(f'[✔] 모든 대기 중인 Pinky들의 거리 데이터 수집 완료!')
            self.distance_collection_complete = True
            self.analyze_distances()
        else:
            missing_count = 0
            for waiting_pinky in self.waiting_pinkys:
                distance_key = pinky_distance_mapping.get(waiting_pinky)
                if distance_key and self.pinky_distance_values.get(distance_key) is None:
                    missing_count += 1
            
            self.get_logger().info(f'[...] 거리 데이터 대기 중 (남은 대기 Pinky: {missing_count}개)')

    def analyze_distances(self):
        """거리 분석 및 최적 Pinky 선택"""
        # 대기 중인 Pinky들의 거리만 고려
        available_distances = {}
        
        pinky_mapping = {
            'pinky_status': 'pinky_dist',
            'pinky_status2': 'pinky_dist2', 
            'pinky_status3': 'pinky_dist3'
        }
        
        for status_key, dist_key in pinky_mapping.items():
            if (self.pinky_status_values.get(status_key) == 0 and  # 대기 중
                self.pinky_distance_values.get(dist_key) is not None):  # 거리 데이터 있음
                available_distances[status_key] = self.pinky_distance_values[dist_key]
        
        if available_distances:
            # 가장 가까운 Pinky 찾기
            closest_pinky = min(available_distances, key=available_distances.get)
            closest_distance = available_distances[closest_pinky]
            
            # Pinky 번호 올바르게 추출
            if closest_pinky == 'pinky_status':
                pinky_number = '1'
            elif closest_pinky == 'pinky_status2':
                pinky_number = '2'
            elif closest_pinky == 'pinky_status3':
                pinky_number = '3'
            else:
                pinky_number = '1'  # 기본값
            
            self.get_logger().info(f'[✔] 최적 선택: Pinky {pinky_number} (거리: {closest_distance:.2f}m)')
            
            # 디버그용: 모든 대기 중인 Pinky들의 거리 출력
            self.get_logger().info(f'[DEBUG] 대기중인 모든 Pinky 거리: {available_distances}')
            
            # 선택된 Pinky에게 서빙 명령 전송
            self.send_delivery_command(closest_pinky)

    def send_delivery_command(self, selected_pinky):
        """선택된 Pinky에게 서빙 위치로 이동 명령 전송"""
        # Pinky 번호 가져오기
        pinky_command_value = self.pinky_command_mapping.get(selected_pinky)
        
        if pinky_command_value is not None:
            # Pinky 번호 발행 (pinky1=5, pinky2=6, pinky3=7)
            pinky_command_msg = Int32()
            pinky_command_msg.data = pinky_command_value
            self.pinky_command_pub.publish(pinky_command_msg)

            time.sleep(1)
            
            # 서빙 위치로 이동 태스크 발행 (서빙위치=8)
            pinky_task_msg = Int32()
            pinky_task_msg.data = 8
            self.pinky_task_pub.publish(pinky_task_msg)
            
            pinky_name = selected_pinky.replace('pinky_status', '') or '1'
            if selected_pinky == 'pinky_status2':
                pinky_name = '2'
            elif selected_pinky == 'pinky_status3':
                pinky_name = '3'
            else:
                pinky_name = '1'
            
            self.get_logger().info(f'[✅] Pinky {pinky_name}에게 서빙 명령 전송!')
            self.get_logger().info(f'   -> pinky_command: {pinky_command_value} (Pinky {pinky_name} 지정)')
            self.get_logger().info(f'   -> pinky_task: 8 (서빙위치로 이동)')
            
            # 명령 전송 후 모니터링 상태 리셋 (중복 명령 방지)
            self.reset_monitoring_flags()
        else:
            self.get_logger().error(f'알 수 없는 Pinky: {selected_pinky}')

    def reset_monitoring_flags(self):
        """모니터링 플래그들 리셋"""
        self.is_monitoring_pinky_status = False
        self.is_monitoring_pinky_distance = False
        
        # 거리 수집 관련 변수들 리셋
        self.waiting_pinkys = []
        self.distance_collection_complete = False
        
        # 상태값들 초기화
        self.pinky_status_values = {
            'pinky_status': None,
            'pinky_status2': None, 
            'pinky_status3': None
        }
        
        # 거리값들 초기화
        self.pinky_distance_values = {
            'pinky_dist': None,
            'pinky_dist2': None,
            'pinky_dist3': None
        }

def main(args=None):
    rclpy.init(args=args)
    node = OrderBufferChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 신호 수신')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()