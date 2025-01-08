### **[2] Python으로 Topic Publish**

**1. my_pub_node 생성**

```python
import rclpy as rp
from geometry_msgs.msg import Twist

rp.init()
my_node_2 = rp.create_node("my_pub_node")
```

**2. defination 확인 및 publish할 defination 값으로 수정**

```python
msg = Twist()
print(msg)
```

```python
msg.linear.x = 3.5
msg.angular.z = 1.9
print(msg)
```

**3. my_pub_node 발행 및 실행**

```python
pub = my_node_2.create_publisher(Twist, "/turtle1/cmd_vel", 10)

pub.publish(msg)
```
![Screenshot from 2025-01-08 21-33-39](https://github.com/user-attachments/assets/14518acb-9dc4-4898-9e3f-1b95a994cba5)
![Screenshot from 2025-01-08 21-24-00](https://github.com/user-attachments/assets/d9ab8ebc-33ac-4180-8bd7-30d38e5c8456)



**4. [반복] my_sub_node 실행**

```python
cnt = 0

# 콜백함수 하나 설정하고
def timer_callback():
    global cnt

    cnt += 1
    print(cnt)

    pub.publish(msg)

    if cnt > 5:
        raise Exception("Publisher Stop!")
```

```python
timer_period = 0.1
timer = my_node_2.create_timer(timer_period, timer_callback)
rp.spin(my_node_2)
```
![Screenshot from 2025-01-08 21-55-48](https://github.com/user-attachments/assets/e07be0ac-54ea-408e-9401-06f32392a789)








