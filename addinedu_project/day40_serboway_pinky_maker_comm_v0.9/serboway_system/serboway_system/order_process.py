# serboway_system/order_process.py

import rclpy
from rclpy.node import Node
import time

from my_msgs.msg import OrderInformation, CompleteOrder
from my_msgs.srv import CheckOrderBuffer, PopOrderBuffer
from std_msgs.msg import Bool, String, Int32, Float32

class OrderBufferChecker(Node):
    def __init__(self):
        super().__init__('order_buffer_checker')
        
        # 서비스 클라이언트 생성
        self.cli = self.create_client(CheckOrderBuffer, 'check_order_buffer')
        self.pop_cli = self.create_client(PopOrderBuffer, 'pop_order_buffer')
        
        # 상태 관리
        self.is_collecting_mode = False
        self.is_order_mode_active = False
        self.collect_start_time = None

        # maker 초기 정의
        self.current_order_id = None
        self.current_ingredients = []
        
        # 🟢 maker_status True 중복 처리 방지 플래그 추가
        self.maker_status_true_processed = False
        
        # [MOD.0529] 샌드위치 제조 실패 중복 방지 플래그 추가
        self.sandwich_failed_logged = False
        
        # [MOD.0529] main_handle 토픽 발행 중복 방지 플래그 추가
        self.main_handle_published = False
        
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
        
        # [MOD.0528] 배송 대기 상태 관리 (순서 변경)
        self.selected_pinky_for_delivery = None  # 선택된 Pinky 정보
        self.is_pinky_at_serving_position = False  # Pinky 서빙위치 도착 여부 (1순위)
        self.is_sandwich_ready = False  # 샌드위치 제조 완료 여부 (2순위)
        self.is_monitoring_delivery_status = False  # 배송 모니터링 활성화 여부
        self.current_complete_order = None  # 현재 완료된 주문 메시지 저장
        self.is_waiting_for_push_complete = False  # 밀기 완료 대기 상태
        
        # [MOD.0529] 3초 대기 타이머 추가
        self.delivery_timer = None
        self.pending_table_number = None
        
        # 서빙위치 도착 판단 기준 거리 (미터)
        self.SERVING_POSITION_THRESHOLD = 0.5  # 0.5m 이하면 도착으로 판단
        
        # 서비스 연결 대기
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('[...] CheckOrderBuffer 서비스 서버 대기 중')
        
        while not self.pop_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('[...] PopOrderBuffer 서비스 서버 대기 중')
        
        # 3초마다 주문 상태 확인
        self.timer = self.create_timer(3.0, self.check_orders)
        self.get_logger().info('[✔] OrderBufferChecker Start')

    def create_publishers_and_subscribers(self):
        """모든 구독자와 발행자를 미리 생성"""
        # 발행자 생성
        self.make_maker_pub = self.create_publisher(OrderInformation, 'main_maker', 10)
        self.main_handle_pub = self.create_publisher(CompleteOrder, 'main_handle', 10)
        
        # Pinky 명령 발행자들 생성
        self.pinky_command_pub = self.create_publisher(Int32, 'pinky_command', 10)
        self.pinky_task_pub = self.create_publisher(Int32, 'pinky_task', 10)
        
        # 🟥 상태 구독자들 생성
        self.maker_status_sub = self.create_subscription(
            Bool, 'maker_status', self.maker_status_callback, 10)
            
        self.complete_main_sub = self.create_subscription(
            CompleteOrder, 'complete_main', self.sandwich_ready_callback, 10)

        self.maker_main_sub = self.create_subscription(
            OrderInformation, 'maker_main', self.sandwich_failed_callback, 10)

        # 수정됨: 콜백 함수 이름 추가
        self.maker_handle_sub = self.create_subscription(
            CompleteOrder, 'maker_handle', self.maker_handle_callback, 10)

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
        
        # 배송 상태 확인용 구독자들 추가 - Bool 타입의 sandwich_ready만 남김
        self.sandwich_ready_sub = self.create_subscription(
            Bool, 'sandwich_ready', self.sandwich_ready_status_callback, 10)
        
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
            self.get_logger().info('[-] 버퍼 비어있음 - 30초 대기 후 수거 모드 전환')
            self.is_collecting_mode = True
            self.collect_start_time = time.time()
        else:
            # 30초 경과 확인
            if time.time() - self.collect_start_time >= 30:
                self.get_logger().info('30초 경과 - 수거 모드로 전환!')
                self.start_collection()

    def start_collection(self):
        """실제 수거 작업 시작"""
        self.get_logger().info('[✔] 수거 작업 시작')
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
        
        # 🟢 주문 모드 시작 시 모든 플래그 리셋
        self.maker_status_true_processed = False
        self.sandwich_failed_logged = False  # [MOD.0529] 샌드위치 실패 로그 플래그 리셋
        self.main_handle_published = False  # [MOD.0529] main_handle 발행 플래그 리셋
        
        self.get_logger().info('[👀] Maker Status Monitoring Start')
        # maker_status 콜백이 자동으로 호출되므로 별도 작업 불필요

    def maker_status_callback(self, msg: Bool):
        """maker_status 구독 콜백 (통합된 maker_command 기능)"""
        # 주문 모드가 활성화된 경우에만 처리
        if not self.is_order_mode_active:
            return
            
        if msg.data:
            # 🟢 True 값을 이미 한 번 처리했다면 무시
            if self.maker_status_true_processed:
                return
                
            # 🟢 True 처리 플래그 설정
            self.maker_status_true_processed = True
            
            # OrderInformation 메시지 생성
            order_msg = OrderInformation()
            
            # 🟥 [추후 변경사항] UI에서 json을 가져오면 그 json에서 값을 가져와서 넣어줘야함!!
            order_msg.id = 4
            order_msg.ingredients = ["불고기 샌드위치", "이탈리안", "양상추", "슈레드 치즈"]  # 예: msg.id = 4 / ["불고기 샌드위치", "이탈리안", "양상추", "슈레드 치즈"]

            # 메시지 발행
            self.make_maker_pub.publish(order_msg)
            self.get_logger().info(f'[✅] Maker에게 "샌드위치 제작 명령 전송" → 주문 ID: {order_msg.id}, 재료: {order_msg.ingredients}')

            # Pinky 상태 모니터링 시작
            self.start_pinky_status_monitoring()
        # 🟢 else 부분 제거 - False일 때 로그 출력하지 않음

    def start_pinky_status_monitoring(self):
        """Pinky 상태 모니터링 시작"""
        self.get_logger().info('[👀] Pinky Status Monitoring Start')
        self.is_monitoring_pinky_status = True

    def pinky_status_callback(self, msg: Int32):
        """pinky_status 구독 콜백 - 상태 모니터링만 담당"""
        if not self.is_monitoring_pinky_status:
            return
            
        self.pinky_status_values['pinky_status'] = msg.data
        status_text = self.get_pinky_status_text(msg.data)
        # self.get_logger().info(f'Pinky Status 1: {status_text} (값: {msg.data})') # Test한 후 주석처리하면 됨!
        self.check_waiting_pinky_count()

    def pinky_status2_callback(self, msg: Int32):
        """pinky_status2 구독 콜백 - 상태 모니터링만 담당"""
        if not self.is_monitoring_pinky_status:
            return
            
        self.pinky_status_values['pinky_status2'] = msg.data
        status_text = self.get_pinky_status_text(msg.data)
        # self.get_logger().info(f'Pinky Status 2: {status_text} (값: {msg.data})') # Test한 후 주석처리하면 됨!
        self.check_waiting_pinky_count()

    def pinky_status3_callback(self, msg: Int32):
        """pinky_status3 구독 콜백 - 상태 모니터링만 담당"""
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
            self.get_logger().info(f'[...] 사용가능한 Pinky: "{waiting_count}개" ({", ".join(waiting_pinkys)})')
            
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
        """pinky_dist 구독 콜백 - 거리 모니터링 + 서빙위치 도착 확인"""
        self.pinky_distance_values['pinky_dist'] = msg.data
        
        # 초기 거리 수집 모드일 때
        if self.is_monitoring_pinky_distance:
            self.get_logger().info(f'[p1] Pinky_1 Distance: {msg.data:.2f}m')  # Test한 후 주석처리하면 됨!
            self.check_distance_collection_complete()
        
        # 배송 모니터링 중일 때 서빙위치 도착 확인
        if (self.is_monitoring_delivery_status and 
            self.selected_pinky_for_delivery == 'pinky_status' and
            not self.is_pinky_at_serving_position):
            self.check_serving_position_arrival('pinky_status', msg.data)

    def pinky_dist2_callback(self, msg: Float32):
        """pinky_dist2 구독 콜백 - 거리 모니터링 + 서빙위치 도착 확인"""
        self.pinky_distance_values['pinky_dist2'] = msg.data
        
        # 초기 거리 수집 모드일 때
        if self.is_monitoring_pinky_distance:
            self.get_logger().info(f'[p2] Pinky_2 Distance: {msg.data:.2f}m')  # Test한 후 주석처리하면 됨!
            self.check_distance_collection_complete()
        
        # 배송 모니터링 중일 때 서빙위치 도착 확인
        if (self.is_monitoring_delivery_status and 
            self.selected_pinky_for_delivery == 'pinky_status2' and
            not self.is_pinky_at_serving_position):
            self.check_serving_position_arrival('pinky_status2', msg.data)

    def pinky_dist3_callback(self, msg: Float32):
        """pinky_dist3 구독 콜백 - 거리 모니터링 + 서빙위치 도착 확인"""
        self.pinky_distance_values['pinky_dist3'] = msg.data
        
        # 초기 거리 수집 모드일 때
        if self.is_monitoring_pinky_distance:
            self.get_logger().info(f'[p3] Pinky_3 Distance: {msg.data:.2f}m')  # Test한 후 주석처리하면 됨!
            self.check_distance_collection_complete()
        
        # 배송 모니터링 중일 때 서빙위치 도착 확인
        if (self.is_monitoring_delivery_status and 
            self.selected_pinky_for_delivery == 'pinky_status3' and
            not self.is_pinky_at_serving_position):
            self.check_serving_position_arrival('pinky_status3', msg.data)

    def check_serving_position_arrival(self, pinky_key, distance):
        """거리 기반으로 서빙위치 도착 확인"""
        if distance <= self.SERVING_POSITION_THRESHOLD:
            pinky_name = self.get_pinky_name_from_status(pinky_key)
            self.pinky_arrived_at_serving_position()

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
            
            self.get_logger().info(f'[✅] Pinky {pinky_name}에게 "서빙 대기 명령 전송"')
            self.get_logger().info(f'   -> pinky_command: {pinky_command_value} (Pinky {pinky_name} 지정)')
            self.get_logger().info(f'   -> pinky_task: 8 (서빙위치로 이동)')
            
            # 배송 대기 상태로 전환
            self.start_delivery_monitoring(selected_pinky)
            
            # 초기 모니터링 상태 리셋 (중복 명령 방지)
            self.reset_initial_monitoring_flags()
        else:
            self.get_logger().error(f'알 수 없는 Pinky: {selected_pinky}')

    def start_delivery_monitoring(self, selected_pinky):
        """배송 대기 모니터링 시작 - [MOD.0528] 순서 변경: 서빙위치 도착 먼저 확인"""
        self.selected_pinky_for_delivery = selected_pinky
        self.is_pinky_at_serving_position = False  # 1순위: Pinky 서빙위치 도착 여부
        self.is_sandwich_ready = False  # 2순위: 샌드위치 제조 완료 여부
        self.is_monitoring_delivery_status = True
        
        pinky_name = self.get_pinky_name_from_status(selected_pinky)
        self.get_logger().info(f'[👀] Serving Waiting Monitoring Start - Pinky {pinky_name}')
        self.get_logger().info(f'   -> 1순위: Pinky 서빙위치 도착 대기 중... (기준: {self.SERVING_POSITION_THRESHOLD}m 이하)')
        self.get_logger().info('   -> 2순위: 샌드위치 제조 완료 대기 중...')

    # [MOD.0528] 1순위: Pinky 서빙위치 도착 확인
    def pinky_arrived_at_serving_position(self):
        """Pinky 서빙위치 도착 확인 (거리 기반) - 1순위"""
        if not self.is_monitoring_delivery_status:
            return
            
        self.is_pinky_at_serving_position = True
        pinky_name = self.get_pinky_name_from_status(self.selected_pinky_for_delivery)
        self.get_logger().info(f'[🚚] Pinky {pinky_name} 서빙위치 도착 완료! (1순위 완료)')
        
        # [MOD.0528] 1순위 완료 후 2순위(샌드위치 제조 완료) 확인
        self.check_delivery_ready()

    # [MOD.0528] 2순위: 샌드위치 제조 완료 확인
    def sandwich_ready_callback(self, msg: CompleteOrder):
        """샌드위치 제조 완료 구독 콜백 (CompleteOrder 타입) - 2순위"""
        if not self.is_monitoring_delivery_status:
            return

        # [MOD.0529] 이미 샌드위치 제조 완료 처리했다면 무시 (중복 방지)
        if self.is_sandwich_ready:
            return

        self.is_sandwich_ready = True
        self.current_complete_order = msg  # 메시지 저장
        self.get_logger().info(f'[🥪] 샌드위치 제조 완료! → 주문 ID: {msg.id} (2순위 완료)')
        
        # [MOD.0528] 2순위 완료 후 전체 배송 준비 상태 확인
        self.check_delivery_ready()

    # [MOD.0528] 배송 준비 상태 확인 (1순위 + 2순위 모두 완료 시)
    def check_delivery_ready(self):
        """순서 변경: 1순위(서빙위치 도착) + 2순위(샌드위치 제조 완료) 확인"""
        self.get_logger().info(f'[DEBUG] 배송 준비 상태 체크:')
        self.get_logger().info(f'   -> 1순위 Pinky 서빙위치 도착: {self.is_pinky_at_serving_position}')
        self.get_logger().info(f'   -> 2순위 샌드위치 제조 완료: {self.is_sandwich_ready}')
        
        if self.is_pinky_at_serving_position and self.is_sandwich_ready:
            pinky_name = self.get_pinky_name_from_status(self.selected_pinky_for_delivery)
            self.get_logger().info(f'[✅] 배송 준비 완료! Pinky {pinky_name} 서빙위치 + 샌드위치 제조 완료')
            
            # 3단계: main_handle로 샌드위치 밀기 명령 발행 및 응답 대기 시작
            if self.current_complete_order:
                self.get_logger().info('[📤] 3단계 시작: Maker(jetcobot)에게 샌드위치 밀기 명령 전송')
                self.push_sandwich_and_wait_response(self.current_complete_order)
            else:
                self.get_logger().error('[❌] current_complete_order가 없습니다!')
            
        else:
            self.get_logger().info('[⏳] 배송 준비 대기 중... (조건 미충족)')
            if not self.is_pinky_at_serving_position:
                self.get_logger().info('   -> 1순위 대기: Pinky 서빙위치 도착 중...')
            if not self.is_sandwich_ready:
                self.get_logger().info('   -> 2순위 대기: 샌드위치 제조 완료 중...')

    def push_sandwich_and_wait_response(self, msg: CompleteOrder):
        """샌드위치 밀기 명령 전송 및 maker_handle 응답 대기"""
        # [MOD.0529] 이미 main_handle 토픽을 발행했다면 무시 (중복 방지)
        if self.main_handle_published:
            self.get_logger().info('[⚠️] main_handle 토픽 이미 발행됨 - 중복 발행 방지')
            return
        
        # [MOD.0529] main_handle 발행 플래그 설정
        self.main_handle_published = True
        
        # 밀기 완료 응답 대기 상태 설정
        self.is_waiting_for_push_complete = True
        
        # main_handle 토픽으로 메시지 전송
        self.get_logger().info(f'[📤] main_handle 토픽으로 Maker(jetcobot)에게 밀기 명령 전송 → 주문 ID: {msg.id}')
        self.main_handle_pub.publish(msg)
        
        # 응답 대기 로그
        self.get_logger().info('[⏳] maker_handle 토픽 응답 대기 중... (Maker가 밀기 작업 완료할 때까지)')
        self.get_logger().info('   -> Maker(jetcobot)가 샌드위치를 Pinky에게 밀어주는 작업 진행 중...')

    # 추가됨: Bool 타입의 sandwich_ready 콜백
    def sandwich_ready_status_callback(self, msg: Bool):
        """sandwich_ready Bool 타입 콜백"""
        if msg.data:
            self.get_logger().info('[🥪] 샌드위치 준비 완료 신호 수신 (Bool 타입)')

    # [MOD.0529] 밀기 완료 응답 처리 + 3초 대기 + 중복 방지
    def maker_handle_callback(self, msg: CompleteOrder):
        """maker_handle 메시지 콜백 - 밀기 완료 응답 처리"""
        if self.is_waiting_for_push_complete:
            self.get_logger().info(f'[✅] 4단계 완료: Maker(jetcobot) 밀기 작업 완료! → 주문 ID: {msg.id}')
            self.get_logger().info('[⏳] 3초 후 Pinky에게 테이블 배송 명령을 전송합니다...')
            
            # 대기 상태 해제
            self.is_waiting_for_push_complete = False
            
            # [MOD.0529] 3초 대기 타이머 시작
            # TODO: 실제로는 주문 정보에서 테이블 번호를 가져와야 함
            self.pending_table_number = 1  # 임시로 테이블 1번으로 설정
            
            # 기존 타이머가 있다면 취소
            if self.delivery_timer is not None:
                self.delivery_timer.cancel()
            
            # 3초 후 배송 명령 실행하는 타이머 생성
            self.delivery_timer = self.create_timer(3.0, self.execute_delayed_delivery)
            
        else:
            # [MOD.0529] 대기 중이 아닐 때는 로그 출력하지 않음 (중복 방지)
            pass

    def execute_delayed_delivery(self):
        """3초 대기 후 실행되는 배송 명령"""
        # 타이머 정리
        if self.delivery_timer is not None:
            self.delivery_timer.cancel()
            self.delivery_timer = None
        
        # 배송 명령 실행
        if self.pending_table_number is not None:
            self.get_logger().info('[🚀] 3초 대기 완료! 이제 테이블 배송 명령을 전송합니다!')
            self.send_table_delivery_command(self.pending_table_number)
            self.pending_table_number = None
        else:
            self.get_logger().error('[❌] pending_table_number가 None입니다!')

    # [MOD.0529] 샌드위치 제조 실패 중복 방지
    def sandwich_failed_callback(self, msg: OrderInformation):
        """샌드위치 제조 실패 콜백 - 중복 방지"""
        # 이미 실패 로그를 출력했다면 무시
        if self.sandwich_failed_logged:
            return
            
        # 실패 로그 플래그 설정
        self.sandwich_failed_logged = True
        
        self.get_logger().error(f'[❌] 샌드위치 제조 실패 → 주문 ID: {msg.id}, 재료: {msg.ingredients}')
        # 실패 처리 로직 추가 가능 (e.g., 재주문 요청, 상태 초기화 등)

    def send_table_delivery_command(self, table_number):
        """선택된 Pinky에게 테이블 배송 명령 전송"""
        if table_number not in [1, 2, 3]:
            self.get_logger().error(f'잘못된 테이블 번호: {table_number}')
            return
            
        # Pinky 번호 가져오기
        pinky_command_value = self.pinky_command_mapping.get(self.selected_pinky_for_delivery)
        
        if pinky_command_value is not None:
            self.get_logger().info('[🚀] 5단계 시작: Pinky에게 테이블 배송 명령 전송')
            
            # Pinky 번호 발행
            pinky_command_msg = Int32()
            pinky_command_msg.data = pinky_command_value
            self.pinky_command_pub.publish(pinky_command_msg)
            
            time.sleep(1)
            
            # 테이블 배송 태스크 발행 (table1=1, table2=2, table3=3)
            pinky_task_msg = Int32()
            pinky_task_msg.data = table_number
            self.pinky_task_pub.publish(pinky_task_msg)
            
            pinky_name = self.get_pinky_name_from_status(self.selected_pinky_for_delivery)
            self.get_logger().info(f'[✅] Pinky {pinky_name}에게 테이블 {table_number} "서빙 명령 전송 완료"')
            self.get_logger().info(f'   -> pinky_command: {pinky_command_value} (Pinky {pinky_name} 지정)')
            self.get_logger().info(f'   -> pinky_task: {table_number} (테이블 {table_number}으로 배송)')
            self.get_logger().info('========== 한 주문의 모든 단계 완료 ==========')
            
            # 배송 명령 후 스마트 주문 완료 처리 (올바른 순서)
            self.complete_current_order()
            
            # 배송 모니터링 완료
            self.reset_delivery_monitoring()
        else:
            self.get_logger().error(f'알 수 없는 Pinky: {self.selected_pinky_for_delivery}')

    # ===== 스마트 주문 처리 로직 (올바른 순서) =====
    
    def complete_current_order(self):
        """현재 주문 완료 처리 - 먼저 pop 후 버퍼 확인"""
        self.get_logger().info('[...] 주문 완료 처리: 현재 주문 제거 중')

        try:
            # 1단계: 먼저 현재 완료된 주문을 pop
            service_future = self.pop_cli.call_async(PopOrderBuffer.Request())
            service_future.add_done_callback(self.handle_pop_then_check_buffer)
            
        except Exception as e:
            self.get_logger().error(f'[❌] 주문 완료 처리 중 오류: {e} - 주문 모드 종료')
            self.is_order_mode_active = False

    def handle_pop_then_check_buffer(self, future):
        """1단계: 현재 주문 제거 후 → 2단계: 남은 주문 확인"""
        try:
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info(f'[✔] {response.message}')
                
                # 2단계: pop 완료 후 남은 주문 확인
                remaining_orders = response.remaining_orders
                self.get_logger().info(f'[📊] 남은 주문: {remaining_orders}개')
                
                if remaining_orders == 0:
                    # 더 이상 주문이 없음 - 수거 모드로 전환
                    self.get_logger().info('[✔] 모든 주문 완료! 수거 모드 전환 준비')
                    self.is_order_mode_active = False
                    
                elif remaining_orders >= 1:
                    # 남은 주문이 있음 - 다음 주문 자동 처리
                    self.get_logger().info(f'[🔄] {remaining_orders}개 추가 주문 발견! 다음 주문 자동 처리...')
                    self.continue_next_order_after_pop()
                    
            else:
                # pop 실패 - 안전하게 버퍼 상태 확인
                error_msg = response.message if response else "응답 없음"
                self.get_logger().warn(f'[⚠️] 주문 제거 실패: {error_msg}')
                self.get_logger().info('[🔍] 버퍼 상태 재확인 중...')
                
                # pop 실패 시 기존 방식으로 버퍼 확인
                service_future = self.cli.call_async(CheckOrderBuffer.Request())
                service_future.add_done_callback(self.handle_buffer_check_after_pop_failure)
                
        except Exception as e:
            self.get_logger().error(f'[❌] 주문 제거 응답 처리 중 오류: {e} - 주문 모드 종료')
            self.is_order_mode_active = False

    def handle_buffer_check_after_pop_failure(self, future):
        """pop 실패 시 버퍼 상태 확인"""
        try:
            response = future.result()
            if response is not None:
                order_count = response.order_count
                self.get_logger().info(f'[📊] 버퍼 재확인 결과: {order_count}개 주문 대기 중')
                
                if order_count == 0:
                    self.get_logger().info('[✅] 버퍼 비어있음 - 수거 모드 전환 준비...')
                    self.is_order_mode_active = False
                else:
                    self.get_logger().info('[⚠️] 버퍼에 주문이 남아있지만 처리 중단 (pop 실패로 인해)')
                    self.is_order_mode_active = False
            else:
                self.get_logger().warn('[⚠️] 버퍼 재확인 실패 - 주문 모드 종료')
                self.is_order_mode_active = False
                
        except Exception as e:
            self.get_logger().error(f'[❌] 버퍼 재확인 중 오류: {e} - 주문 모드 종료')
            self.is_order_mode_active = False

    def continue_next_order_after_pop(self):
        """주문 제거 후 다음 주문 자동 시작"""
        self.get_logger().info('[🔄] 다음 주문 자동 처리 시작!')
        
        # 잠시 대기 후 다음 주문 처리 (버퍼 업데이트 시간 고려)
        time.sleep(0.3)
        
        # 주문 모드 재활성화
        self.is_order_mode_active = False  # 리셋
        self.handle_order_mode()           # 다음 주문 모드 시작
        self.is_order_mode_active = True

    # ===== 🔧 유틸리티 함수들 =====

    def get_pinky_name_from_status(self, status_key):
        """상태 키에서 Pinky 이름 추출"""
        if status_key == 'pinky_status':
            return '1'
        elif status_key == 'pinky_status2':
            return '2'
        elif status_key == 'pinky_status3':
            return '3'
        else:
            return '1'  # 기본값

    def reset_delivery_monitoring(self):
        """배송 모니터링 상태 리셋"""
        self.selected_pinky_for_delivery = None
        self.is_pinky_at_serving_position = False  # 1순위 리셋
        self.is_sandwich_ready = False  # 2순위 리셋
        self.is_monitoring_delivery_status = False
        self.current_complete_order = None  # 메시지도 리셋
        self.is_waiting_for_push_complete = False  # 밀기 대기 상태도 리셋
        self.main_handle_published = False  # [MOD.0529] main_handle 발행 플래그도 리셋
        
        # [MOD.0529] 타이머 정리
        if self.delivery_timer is not None:
            self.delivery_timer.cancel()
            self.delivery_timer = None
        self.pending_table_number = None
        
        self.get_logger().info('[↻] 배송 모니터링 완료 - 상태 리셋')

    def reset_initial_monitoring_flags(self):
        """초기 모니터링 플래그들만 리셋 (배송 모니터링은 유지)"""
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

    def reset_monitoring_flags(self):
        """모든 모니터링 플래그들 리셋"""
        self.reset_initial_monitoring_flags()
        self.reset_delivery_monitoring()
        
        # 🟢 maker_status True 처리 플래그도 리셋
        self.maker_status_true_processed = False
        
        # [MOD.0529] 샌드위치 실패 로그 플래그도 리셋
        self.sandwich_failed_logged = False
        
        # [MOD.0529] main_handle 발행 플래그도 리셋
        self.main_handle_published = False
        
        # [MOD.0529] 타이머 정리
        if self.delivery_timer is not None:
            self.delivery_timer.cancel()
            self.delivery_timer = None
        self.pending_table_number = None


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