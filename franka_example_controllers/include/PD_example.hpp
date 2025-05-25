//이 코드는 연습용으로 작성된 PID 제어기입니다.

#pragma once //해더 중복을 방지하기 위한 전처리기 지시문
#include <string> //문자열 처리를 위한 헤더파일
#include <Eigen/Eigen>

// ros2 패키지 (또는 그 의존성)안에 있는 헤더파일을 포함
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_robot_state.hpp" 

//Lifecycle 콜백 리턴 타입을 짧게 쓸 수 있도록 alias 정의
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// namespace는 소속을 만들어 함수나 구조체의 이름 충돌을 방지하는 역할을 한다. -> 함수이름이 같아도 namespace가 다르면 충돌하지 않음
// cpp와 hpp의 네임스페이스는 동일해야 한다.
namespace franka_example_controllers {

class PDExampleController : public controller_interface::ControllerInterface {
 public:   // → 외부(다른 객체·함수)에서 마음껏 호출·접근할 수 있는 멤버
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:   // → 클래스 내부에서만 쓰이는 멤버. 외부에서는 접근불가
  std::string arm_id_; // 로봇 팔의 ID를 저장하는 문자열 변수
  bool is_gazebo_{false}; // Gazebo 시뮬레이터에서 실행 중인지 여부를 나타내는 불리언 변수
  std::string robot_description_; // 로봇의 URDF(Universal Robot Description Format) 설명을 저장하는 문자열 변수
  const int num_joints = 7; // 로봇의 관절 수를 저장하는 상수 변수
  std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0}; // 로봇 관절의 초기 위치를 저장하는 배열
  double elapsed_time_ = 0.0;
  double initial_robot_time_ = 0.0;
  double robot_time_ = 0.0;
  double trajectory_period_ = 0.001;
  bool initialization_flag_{true};
  rclcpp::Time start_time_; // rclcpp::Time 객체로, ROS2의 시간 정보를 저장하는 변수


  std::array<double, num_joints> prev_error_{{0.0}};  // 이전 오차 저장
  double Kp{10.0};  // 비례 이득
  double Kd{ 1.0};  // 미분 이득


};

}  // namespace franka_example_controllers

/*
여기서 std는 C++의 표준 라이브러리 네임스페이스로, 자료구조/알고리즘/유틸리티 기능 등을 포함합니다.
rclcpp는 ROS2의 C++ 클라이언트 라이브러리로, ROS2의 노드운영, 통신, 시간관련 기능을 제공합니다.
*/