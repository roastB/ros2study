# 기존 내용 삭제 후 아래 내용으로 교체
cmake_minimum_required(VERSION 3.8)
project(jetcobot_teleop)

# 필수 패키지 검색
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 액션 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/OrderJetcobot.action"
)

# Python 패키지 설치 설정
ament_python_install_package(${PROJECT_NAME})

# 추가 파일 설치
install(
  DIRECTORY 
    action 
    jetcobot_teleop 
    msg
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
