# Franka-emika-panda_ros2

모든 기본적인 설명은 아래의 사이트의 정보에 기반한다.
https://frankaemika.github.io/docs/overview.html

## Install
- Ubuntu 20.04 LTS
- ROS2 Humble
- Gazebo Garden
https://gazebosim.org/docs/fortress/getstarted/

## Overview
- `Franka Control Interface(FCI)`
  - 빠르고 즉각적인 신호전달! Ethernet과 연결된 외부 PC를 통해 direct control 수행
- `libfranka`
  - open source C++ interface로, real-time control value를 보낼 수 있다.(1 kHz, 5 different interfaces ; commands, control value)
  - 다양한 라이브러리를 사용할 수 있다
- `franka_ros`
  - Franka Robotics와 전체 ROS ecosystem 을 연결한다. 이것은 `libfranka`를 ROS Control과 통합시킨다.
  - URDF model 포함하며 visualization (e.g. RViz) and kinematic simulations 가능
  - moveit package에 ROS를 통한 로봇제어 예시가 있다.

`franka_ros`는 로봇 구동에 반드시 필요한 것은 아니고 ROS를 통해 로봇을 제어하는 경우 필요하다.
gazebo 와 같은 시뮬레이터와 연결하기 위해서는 ROS 필요

## Installation on Linux
각 버전호환 확인
https://support.franka.de/docs/compatibility.html
- Robot system version, Robot / Gripper Server version 같은 경우는 로봇에 내장. (아마도.. 변동불가)
- 22.04, humble, kernel nersion 등을 모두 고려해야 한다.

일단 아직은 실제 로봇 버전은 모르니까, 최신버전으로 했다.
- Robot system version : 5.5.0(FR3) 이상
- libfranka version : 0.13.3 이상
- Robot / Gripper Server version : 7 / 3
- franka_ros2 version : 0.1.15 이상
- franka_description : 0.3.0 이상





