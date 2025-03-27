```markdown
# [ rclpy 사용시 주의점 ]
- 반드시 터미널에서 humble 혹은 ros2study를 실행한 후, jupyther notebook을 실행하여야지 rclpy를 인식한다!!
```

### **[1] Python으로 Topic Subscription**

**1. my_sub_node 생성**

```python
import rclpy as rp
from turtlesim.msg import Pose
```

```python
rp.init() # rp.init()은 딱 한번만 실행할것!
```

```python
my_node = rp.create_node("my_sub_node")
```

**2. callback 함수 생성**

```python
def callback(data):
    print("============")
    print(f"/turtle1/pose : {data}")
    print(f"X : {data.x}")
    print(f"Y : {data.y}")
    print(f"Theta : {data.theta}")
```

**3. my_sub_node 구독**

```python
my_node.create_subscription(Pose, "/turtle1/pose", callback, 10)
```

```python
# # 정리 : my_custom_node를 생성하여 /turtle1/pose를 subscription(구독)!
# import rclpy as rp
# from turtlesim.msg import Pose

# rp.init()
# my_node = rp.create_node("my_sub_node")

# # subscription
# my_node.create_subscription(Pose, "/turtle1/pose", callback, 10)
```
![Screenshot from 2025-01-08 21-33-30](https://github.com/user-attachments/assets/fd4a90e6-254d-4fcd-9405-dfea0b69373e)



**4. my_sub_node 실행**

```python
rp.spin_once(my_node)
```

```python
rp.spin(my_node)
```

![Screenshot from 2025-01-07 21-19-25](https://github.com/user-attachments/assets/c4857f8b-c8fb-485c-a3bd-15eaebd6638f)






