cmake_minimum_required(VERSION 3.8)
project(jetcobot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# (Optional but recommended) interface 종속성 지정
find_package(builtin_interfaces REQUIRED)

# action 인터페이스 파일 등록
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/OrderJetcobot.action"
  DEPENDENCIES builtin_interfaces  # 필수! 시간, 문자열 타입 등 사용시
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
