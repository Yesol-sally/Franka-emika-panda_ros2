# Franka-emika-panda_ros2

모든 기본적인 설명은 아래의 사이트의 정보에 기반한다.
https://frankaemika.github.io/docs/overview.html

## Install
https://frankaemika.github.io/docs/installation_linux.html#installation-build-from-source
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
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/6.8/older/patches-6.8-rt8.tar.sign
curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/6.8/older/patches-6.8-rt8.tar.xz
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

3. 여기서 public key를 다운로드 한다. -> 위 예시에서는 `6092693E`

4. 얻은 key를 적용하여 key sesrver를 알아낸다.
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


