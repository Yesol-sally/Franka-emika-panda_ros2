
# Franka-emika-panda_ros2

모든 기본적인 설명은 아래의 사이트의 정보에 기반한다.
https://frankaemika.github.io/docs/overview.html

## Install
https://frankaemika.github.io/docs/installation_linux.html#installation-build-from-source
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Fortress
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

## Building libfranka
1. 종속성 설치
  `sudo apt install build-essential cmake git libpoco-dev libeigen3-dev`

2. git clone
  1) Pandas : `git clone --recursive https://github.com/frankaemika/libfranka # only for panda`
  2) Franka Research 3 : `git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0 # only for FR3`

## Building the ROS packages
1. 폴더생성
  ```
cd /path/to/desired/folder
  mkdir -p catkin_ws/src
  cd catkin_ws
  source /opt/ros/noetic/setup.sh
```

2. clone
```
git clone --recursive https://github.com/frankaemika/franka_ros2.git
```

4. 설치확인
```
rosdep install --from-paths src --ignore-src --rosdistro humble
```

## Setting up the real-time kernel
- FE3의 경우에는 반드시 RT Kernel을 사용해야 API에서 에러가 나지 않는다.
- RealTime Kernel을 사용하지 않는 경우, 아래와 같이 파일을 수정해야 한다.
`libfranka/include/franka/robot.h`파일에서
  66줄의 RealtimeConfig::kEnforce에`kEnforce`를 `kIgnore`로 수정
  파일 전체 `limit_rate = true` 를 `limit_rate = false` 로 수정


1. First, install the necessary dependencies:
```
sudo apt-get install build-essential bc curl ca-certificates gnupg2 libssl-dev lsb-release libelf-dev bison flex dwarves zstd libncurses-dev
```

3. `uname -r` 으로 본인의 현재 kernel 이름을 알아낸다.
  - 본인 : 6.8.0-45-generic
  - https://www.kernel.org/pub/linux/kernel/projects/rt/ 에서 본인의 버전과 가장 가까운 버전을 선택

3. curl 명령어를 통해 다운로드
예시) 22.04 테스트
```
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/v5.x/linux-5.9.1.tar.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/5.9/patch-5.9.1-rt20.patch.sign
```

적용)
```
curl -SLO https://www.kernel.org/pub/linux/kernel/v6.x/linux-6.8.tar.xz
curl -SLO https://www.kernel.org/pub/linux/kernel/v6.x/linux-6.8.tar.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/6.8/older/patch-6.8-rt8.patch.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/6.8/older/patch-6.8-rt8.patch.xz
```

5. 다운된 파일의 압축해제 : `xz -d *.xz`
- xz 명령어는 파일 압축 및 해제를 위한 유틸리티이다.
```
sudo apt update
sudo apt install xz-utils
```

## Verifying file integrity
이번 step은 optional 이지만 권장된다. (.sign 파일을 사용하여 다운로드 한 파일의 손상여부 확인가능)

1. You can use gpg2 to verify the .tar archives:
```
gpg2 --verify linux-*.tar.sign
gpg2 --verify patch-*.patch.sign
```
(gpg2설치 : `sudo apt-get install gnupg2 -y`)

2. 명령어를 입력한 결과가 아래와 같을 것
```
$ gpg2 --verify linux-*.tar.sign
gpg: assuming signed data in 'linux-4.14.12.tar'
gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
gpg: Can't check signature: No public key
```

본인 결과)
```
gpg2 --verify patch-*.patch.sign
gpg: assuming signed data in 'linux-6.8.tar'
gpg: Signature made Mon 11 Mar 2024 06:39:53 AM KST
gpg:                using RSA key 647F28654894E3BD457199BE38DBBDC86092693E
gpg: Can't check signature: No public key
gpg: assuming signed data in 'patch-5.9.1-rt20.patch'
gpg: Signature made Thu 29 Oct 2020 05:04:54 AM KST
gpg:                using RSA key 57892E705233051337F6FDD105641F175712FA5B
gpg: Can't check signature: No public key
```


3. 여기서 public key를 다운로드 한다. -> 위 예시에서는 `6092693E`

4.
- Ubuntu 키 서버에서 키 ID가 2872E4CC인 공개 키를 다운로드하여 로컬 GPG 키링에 추가하는 역할을 합니다. 이 공개 키는 특정한 사람이나 조직이 서명을 했을 때, 그 서명을 검증하거나 메시지 암호화를 위한 공개 키로 사용될 수 있습니다.
이 과정은 주로 소프트웨어 패키지 서명을 검증하거나, 보안 통신을 할 때 GPG 키를 받아서 서명자나 송신자의 신원을 확인하는 데 사용됩니다.
```
gpg2  --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 6092693E
```
5. Having downloaded the keys, you can now verify the sources. Here is an example of a correct output:
```
$ gpg2 --verify linux-*.tar.sign
gpg: assuming signed data in 'linux-4.14.12.tar'
gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
gpg: Good signature from "Greg Kroah-Hartman <gregkh@linuxfoundation.org>" [unknown]
gpg:                 aka "Greg Kroah-Hartman <gregkh@kernel.org>" [unknown]
gpg:                 aka "Greg Kroah-Hartman (Linux kernel stable release signing key) <greg@kroah.com>" [unknown]
gpg: WARNING: This key is not certified with a trusted signature!
gpg:          There is no indication that the signature belongs to the owner.
Primary key fingerprint: 647F 2865 4894 E3BD 4571  99BE 38DB BDC8 6092 693E
```


## Compiling the kernel
1. 이전까지 파일들을 제대로 다운로드 받았다면, source code를 추출할 수 있고 patch를 적용할 수 있다.
```
tar xf linux-*.tar
cd linux-*/
patch -p1 < ../patch-*.patch
```

2. 다음으로는 현재 부팅되어있는 커널구성을 copy한다. ->> 새로운 real time kernel을 위한 default configulation으로 부터
  
3. 새로운 config를 만들 때, 2번의 config를 default로 사용할 수 있다.
```
make olddefconfig
make menuconfig
```

- 위 명령어 중 `make menuconfig`는 preemption(선점) model을 구성할 수 있는 terminal interface 를 가져온다.
- 만약 TUIs 보다 GUIs 를 선호하면, use make `xconfig` instead of `make menuconfig`

4. 아래의 과정을 수행
> General Setup > Preemption Model and select Fully Preemptible Kernel (Real-Time).
> Cryptographic API > Certificates for signature checking (at the very bottom of the list)
> Provide system-wide ring of trusted keys > Additional X.509 keys for default system keyring

> Remove the “debian/canonical-certs.pem” from the prompt and press Ok.
> Save this configuration to .config and exit the TUI.

5. Kernel 컴파일
- 시간이 오래 소요되므로, 멀티스레딩 옵션 `-j`를 CPU 코어 수로 설정한다.
  ```
  make -j$(nproc) deb-pkg
  ```

6. 새로 생성된 패키지 설치
```
sudo dpkg -i ../linux-headers-*.deb ../linux-image-*.deb
```


## Verifying the new kernel
1. 시스템 재부팅
  
2. Grub boot menu에서 새로 설치된 kernel 선택

3. 현재 새로운 Kernel이 사용되고 있는지를 보기 위해 `uname -a` 명령어를 입력
- 이때 명령어에 대한 결과에는 `PREEMPT RT`이라는 문자열과 이전에 선택했던 version number가 들어가야 한다.
- 추가로,`/sys/kernel/realtime` should exist and contain the the number `1`.

+) 만약 오류가 뜬다면 아래 확인..
https://frankaemika.github.io/docs/troubleshooting.html#troubleshooting-realtime-kernel


## Allow a user to set real-time permissions for its processes
1. 새로운 커널이 잘 동작하는 경우, 아래 명령어와 같이 그룹과 그룹에 해당하는 유저 이름을 추가한다.
```
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)
```

2. `/etc/security/limits.conf`의 real time group에 아래의 제한사항을 추가한다.
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```


# franka_ros2
https://github.com/frankaemika/franka_ros2


=======
<h1 style="font-size: 3em;">ROS 2 Integration for Franka Robotics Research Robots</h1>

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

> **Note:** _franka_ros2_ is not officially supported on Windows.

#### Table of Contents
- [About](#about)
- [Caution](#caution)
- [Setup](#setup)
  - [Local Machine Installation](#local-machine-installation)
  - [Docker Container Installation](#docker-container-installation)
- [Test the Setup](#test-the-setup)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

# About
The **franka_ros2** repository provides a **ROS 2** integration of **libfranka**, allowing efficient control of the Franka Robotics arm within the ROS 2 framework. This project is designed to facilitate robotic research and development by providing a robust interface for controlling the research versions of Franka Robotics robots.

For convenience, we provide Dockerfile and docker-compose.yml files. While it is possible to build **franka_ros2** directly on your local machine, this approach requires manual installation of certain dependencies, while many others will be automatically installed by the **ROS 2** build system (e.g., via **rosdep**). This can result in a large number of libraries being installed on your system, potentially causing conflicts. Using Docker encapsulates these dependencies within the container, minimizing such risks. Docker also ensures a consistent and reproducible build environment across systems. For these reasons, we recommend using Docker.

# Caution
This package is in rapid development. Users should expect breaking changes and are encouraged to report any bugs via [GitHub Issues page](https://github.com/frankaemika/franka_ros2/issues).

# Franka ROS 2 Dependencies Setup

This repository contains a `.repos` file that helps you clone the required dependencies for Franka ROS 2.

## Prerequisites

## Local Machine Installation
1. **Install ROS2 Development environment**

    _**franka_ros2**_ is built upon _**ROS 2 Humble**_.  

    To set up your ROS 2 environment, follow the official _**humble**_ installation instructions provided [**here**](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). 
    The guide discusses two main installation options: **Desktop** and **Bare Bones**.

    #### Choose **one** of the following:
    - **ROS 2 "Desktop Install"** (`ros-humble-desktop`)  
      Includes a full ROS 2 installation with GUI tools and visualization packages (e.g., Rviz and Gazebo).  
      **Recommended** for users who need simulation or visualization capabilities.

    - **"ROS-Base Install (Bare Bones)"** (`ros-humble-ros-base`)  
      A minimal installation that includes only the core ROS 2 libraries.  
      Suitable for resource-constrained environments or headless systems.

    ```bash
    # replace <YOUR CHOICE> with either ros-humble-desktop or ros-humble-ros-base
    sudo apt install <YOUR CHOICE>  
    ```
    ---
    Also install the **Development Tools** package:
    ```bash
    sudo apt install ros-dev-tools
    ```
    Installing the **Desktop** or **Bare Bones** should automatically source the **ROS2** environment but, under some circumstances you may need to do this again:
    ```bash
    source /opt/ros/humble/setup.sh
    ```

2. **Create a ROS 2 Workspace:**
   ```bash
   mkdir -p ~/franka_ros2_ws/src
   cd ~/franka_ros2_ws  # not into src
   ```
3. **Clone the Repositories:**
   ```bash
    git clone https://github.com/frankaemika/franka_ros2.git src
    ```
4. **Install the dependencies**
    ```bash
    vcs import src < src/franka.repos --recursive --skip-existing
    ```
5. **Detect and install project dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```
6. **Build**
   ```bash
   # use the --symlinks option to reduce disk usage, and facilitate development.
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
7. **Adjust Enviroment**
   ```bash
   # Adjust environment to recognize packages and dependencies in your newly built ROS 2 workspace.
   source install/setup.sh
   ```

## Docker Container Installation
The **franka_ros2** package includes a `Dockerfile` and a `docker-compose.yml`, which allows you to use `franka_ros2` packages without manually installing **ROS 2**. Also, the support for Dev Containers in Visual Studio Code is provided.

For detailed instructions, on preparing VSCode to use the `.devcontainer` follow the setup guide from [VSCode devcontainer_setup](https://code.visualstudio.com/docs/devcontainers/tutorial).

1. **Clone the Repositories:**
    ```bash
    git clone https://github.com/frankaemika/franka_ros2.git
    cd franka_ros2
    ```
    We provide separate instructions for using Docker with Visual Studio Code or the command line. Choose one of the following options:

    Option A: Set up and use Docker from the command line (without Visual Studio Code).

    Option B: Set up and use Docker with Visual Studio Code's Docker support.

#### Option A: using Docker Compose

  2. **Save the current user id into a file:**
      ```bash
      echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
      ```
      It is needed to mount the folder from inside the Docker container.

  3. **Build the container:**
      ```bash
      docker compose build
      ```
  4. **Run the container:**
      ```bash
      docker compose up -d
      ```
  5. **Open a shell inside the container:**
      ```bash
      docker exec -it franka_ros2 /bin/bash
      ```
  6. **Clone the latests dependencies:**
      ```bash
      vcs import src < src/franka.repos --recursive --skip-existing
      ```
  7. **Build the workspace:**
      ```bash
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```
  7. **Source the built workspace:**
      ```bash
      source install/setup.bash
      ```
  8. **When you are done, you can exit the shell and delete the container**:
      ```bash
      docker compose down -t 0
      ```

#### Option B: using Dev Containers in Visual Studio Code

  2. **Open Visual Studio Code ...**
  
        Then, open folder  `franka_ros2`

  3. **Choose `Reopen in container` when prompted.**

      The container will be built automatically, as required.

  4. **Clone the latests dependencies:**
      ```bash
      vcs import src < src/franka.repos --recursive --skip-existing
      ```

  5. **Open a terminal and build the workspace:**
      ```bash
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```
  6. **Source the built workspace environment:**
      ```bash
      source install/setup.bash
      ```


# Test the build
   ```bash
   colcon test
   ```
> Remember, franka_ros2 is under development.  
> Warnings can be expected.  

# Run a sample ROS2 application

To verify that your setup works correctly without a robot, you can run the following command to use dummy hardware:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true
```


# Troubleshooting
#### `libfranka: UDP receive: Timeout error`

If you encounter a UDP receive timeout error while communicating with the robot, avoid using Docker Desktop. It may not provide the necessary real-time capabilities required for reliable communication with the robot. Instead, using Docker Engine is sufficient for this purpose.

A real-time kernel is essential to ensure proper communication and to prevent timeout issues. For guidance on setting up a real-time kernel, please refer to the [Franka installation documentation](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel).

# Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](https://github.com/frankaemika/franka_ros2/blob/humble/CONTRIBUTING.md) for more details on how to contribute to this project.

## License

All packages of franka_ros2 are licensed under the Apache 2.0 license.

## Contact 

For questions or support, please open an issue on the [GitHub Issues](https://github.com/frankaemika/franka_ros2/issues) page.

See the [Franka Control Interface (FCI) documentation](https://frankaemika.github.io/docs) for more information.


[def]: #docker-container-installation
>>>>>>> humble
