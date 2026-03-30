cmake_minimum_required(VERSION 3.8)
project(my_project_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 의존성 패키지 불러오기 (이 줄이 반드시 있어야 합니다!)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 액션 파일 등록
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Summon.action"
)

# 설치 및 패키징
ament_package()
