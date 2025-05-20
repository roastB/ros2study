import rclpy
from rclpy.node import Node
import ingredient_stackbot.recipe as rc
from ingredient_stackbot.robot import get_robot
from my_msgs.msg import OrderInformation, CompleteOrder # 주문 완료 msg 추가
from std_msgs.msg import Int32MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import ingredient_stackbot.check_ingredient as ch

class IngredientSubscriber(Node):
    def __init__(self):
        super().__init__('ingredient_subscriber')
        self.subscription = self.create_subscription(OrderInformation, 'control_jetcobot', self.listener_callback, 10)  # 큐 사이즈
        self.publisher = self.create_publisher(OrderInformation, 'jetcobot_control', 10)
        self.completed_publisher = self.create_publisher(CompleteOrder, 'complete_control', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received id: {msg.id}')

        if ch.check_ingredients_order(msg.ingredients):
            self.get_logger().warn("재료가 존재하지 않아 동작을 실행하지 않습니다.")
            empty_list = ch.check_ingredients_order(msg.ingredients)
            print(empty_list)
            completion_msg = OrderInformation()
            completion_msg.id = msg.id
            completion_msg.ingredients = ["불고기 샌드위치", "이탈리안", "양상추", "슈레드 치즈"]
            self.publisher.publish(completion_msg)
            self.get_logger().info("Published message of processing fail")
            return

        plate_position = rc.get_plate_position("접시")[1]
        topbread_position = rc.get_bread_position("위 빵")[1]
        bottombread_position = rc.get_bread_position("아래 빵")[1]

        meat_position = rc.get_meat_position(msg.ingredients[0])[1]
        if meat_position is not None:
            self.get_logger().info(f'Ingredient: {msg.ingredients[0]}, Position: {meat_position}')
        else:
            self.get_logger().warn(f'No position found for ingredient: {msg.ingredients[0]}')

        
        sauce_position = rc.get_sauce_position(msg.ingredients[1])[1]
        if sauce_position is not None:
            self.get_logger().info(f'Ingredient: {msg.ingredients[1]}, Position: {sauce_position}')
        else:
            self.get_logger().warn(f'No position found for ingredient: {msg.ingredients[1]}')

    
        vegetable_position = rc.get_vegetable_position(msg.ingredients[2])[1]
        if vegetable_position is not None:
            self.get_logger().info(f'Ingredient: {msg.ingredients[2]}, Position: {vegetable_position}')
        else:
            self.get_logger().warn(f'No position found for ingredient: {msg.ingredients[2]}')

    
        cheese_position = rc.get_cheese_position(msg.ingredients[3])[1]
        if cheese_position is not None:
            self.get_logger().info(f'Ingredient: {msg.ingredients[3]}, Position: {cheese_position}')
        else:
            self.get_logger().warn(f'No position found for ingredient: {msg.ingredients[3]}')            
            
        mc = get_robot()

        print("0. 접시로 이동합니다.")
        mc.send_coords(plate_position, 50, 1)
        time.sleep(3)
        print("아래 빵 좌표:", plate_position)

        print("1. 아래 빵으로 이동합니다.")
        mc.send_coords(bottombread_position, 50, 1)
        time.sleep(3)
        print("아래 빵 좌표:", bottombread_position)

        print("2. 채소로 이동합니다.")
        mc.send_coords(vegetable_position, 50, 1)
        time.sleep(3)
        print("채소 좌표:", vegetable_position)

        print("3. 고기로 이동합니다.")
        mc.send_coords(meat_position, 50, 1)
        time.sleep(3)
        print("고기 좌표:", meat_position)

        print("4. 치즈로 이동합니다.")
        mc.send_coords(cheese_position, 50, 1)
        time.sleep(3)
        print("치즈 좌표:", cheese_position)

        print("5. 소스로 이동합니다.")
        mc.send_coords(sauce_position, 50, 1)
        time.sleep(3)
        print("소스 좌표:", sauce_position)

        print("6. 위 빵으로 이동합니다.")
        mc.send_coords(topbread_position, 50, 1)
        time.sleep(3)
        print("위 빵 좌표:", topbread_position)

        # 제조 완료 msg에 맞춰서 직접 수정~!
        completion_msg = CompleteOrder()
        completion_msg.id = msg.id
        completion_msg.order_completed = "Making Completed!"
        self.completed_publisher.publish(completion_msg)
        self.get_logger().info("Published completion message: Robot operation completed")

def main(args=None):
    rclpy.init(args=args)
    ingredient_subscriber = IngredientSubscriber()
    rclpy.spin(ingredient_subscriber)
    ingredient_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
