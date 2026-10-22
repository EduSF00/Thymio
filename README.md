# Installing ROS 2 Jazzy, ROS-Aseba and ROS-Thymio

This guide describes how to install ROS 2 Jazzy and the ROS-Aseba and ROS-Thymio packages to work with the Thymio robot.

## Table of Contents

1. [Installing ROS 2](#1-installing-ros-2)
2. [Creating the ROS workspace](#2-creating-the-ros-workspace)
3. [Installing ROS-Aseba](#3-installing-ros-aseba)
4. [Installing ROS-Thymio](#4-installing-ros-thymio)

---

## 1. Installing ROS 2

### 1.1 Set up the locale

Make sure you have a locale which supports UTF-8.

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

### 1.2 Add the ROS 2 apt repository

You will need to add the ROS 2 apt repository to your system.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### 1.3 Install the ros2-apt-source package

Installing the `ros2-apt-source` package will configure the ROS 2 repositories for your system.

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### 1.4 Update the apt caches

Update your apt repository caches after setting up the repositories.

```bash
sudo apt update
sudo apt upgrade
```

### 1.5 Install ROS 2

```bash
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-ros-base
```

---

## 2. Creating the ROS workspace

If you don't already have one, create a ROS workspace in a folder `<WORKSPACE>` of your choice.

```bash
mkdir -p <WORKSPACE>/src
cd <WORKSPACE>
source /opt/ros/<ROS-VERSION>/setup.bash
colcon build
```

> In this guide, `<ROS-VERSION>` is `jazzy`.

---

## 3. Installing ROS-Aseba

### 3.1 Basic dependencies

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### 3.2 Install libudev-dev, libxml2-dev, git and ros-jazzy-diagnostic-updater

```bash
sudo apt install libudev-dev libxml2-dev git ros-jazzy-diagnostic-updater
```

### 3.3 Download and build ROS-Aseba

```bash
cd <WORKSPACE>
source install/setup.bash
git clone https://github.com/jeguzzi/ros-aseba.git --branch ros2 --recurse-submodules src/ros-aseba
colcon build
```

### 3.4 Serial port access (Linux)

On Linux, you may need to add the user to the `dialout` group to connect to a Thymio.

```bash
sudo adduser $USER dialout
```

> You need to log out and back in (or reboot) for the group change to take effect.

---

## 4. Installing ROS-Thymio

### 4.1 Install xacro, tf2 and robot-state-publisher

Packages `ros-<ROS-VERSION>-xacro`, `ros-<ROS-VERSION>-tf2` and `ros-<ROS-VERSION>-robot-state-publisher`:

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-tf2 ros-jazzy-robot-state-publisher
```

### 4.2 Download and build ROS-Thymio

```bash
cd <WORKSPACE>
source install/setup.bash
git clone https://github.com/jeguzzi/ros-thymio.git --branch master --recurse-submodules src/ros-thymio
colcon build
```

---

## References

- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
- [ros-aseba](https://github.com/jeguzzi/ros-aseba)
- [ros-thymio](https://github.com/jeguzzi/ros-thymio)
