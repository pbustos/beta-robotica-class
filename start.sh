#!/bin/bash

# WEBOTS
TAB_NAME="webots"
session_id=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle "$session_id" "$TAB_NAME"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "rcnode"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "webots"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.raiseSession $session_id


# BRIDGE
TAB_NAME="bridge"
DIRECTORY_PATH="/home/robocomp/robocomp/components/webots-bridge" # replace with your desired path
session_id=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle "$session_id" "$TAB_NAME"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cd $DIRECTORY_PATH"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "bin/Webots2Robocomp etc/config"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.raiseSession $session_id

# CAMERA
TAB_NAME="camera"
DIRECTORY_PATH="~/robocomp/components/robocomp-robolab/components/hardware/camera/ricoh_omni" # replace with your desired path
session_id=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle "$session_id" "$TAB_NAME"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cd $DIRECTORY_PATH"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cmake . && make -j32"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "bin/RicohOmni etc/config_wb"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.raiseSession $session_id

# LIDAR3D-HELIOS
TAB_NAME="helios"
DIRECTORY_PATH="~/robocomp/components/robocomp-robolab/components/hardware/laser/lidar3D" # replace with your desired path
session_id=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle "$session_id" "$TAB_NAME"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cd $DIRECTORY_PATH"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cmake . && make -j32"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "bin/Lidar3D etc/config_helios_webots"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.raiseSession $session_id

# LIDAR3D-BPEARL
TAB_NAME="pearl"
DIRECTORY_PATH="~/robocomp/components/robocomp-robolab/components/hardware/laser/lidar3D" # replace with your desired path
session_id=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle "$session_id" "$TAB_NAME"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cd $DIRECTORY_PATH"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cmake . && make -j32"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "bin/Lidar3D etc/config_pearl_webots"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.raiseSession $session_id

# RGBD
TAB_NAME="rgbd"
DIRECTORY_PATH="~/robocomp/components/robocomp-shadow/insect/RGBD_360" # replace with your desired path
session_id=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle "$session_id" "$TAB_NAME"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cd $DIRECTORY_PATH"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cmake . && make -j32"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "bin/RGBD_360 etc/config_wb"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.raiseSession $session_id

# joystick pub
TAB_NAME="joy"
DIRECTORY_PATH="~/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish" # replace with your desired path
session_id=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle "$session_id" "$TAB_NAME"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cd $DIRECTORY_PATH"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cmake . && make -j32"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "bin/JoystickPublish etc/config_shadow"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.raiseSession $session_id

# yolo
TAB_NAME="yolo"
DIRECTORY_PATH="~/robocomp/components/robocomp-shadow/insect/environment_object_perception"
session_id=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle "$session_id" "$TAB_NAME"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cd $DIRECTORY_PATH"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cmake . && make -j32"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "src/environment_object_perception.py etc/config_wb"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.raiseSession $session_id

# lidar odometry
TAB_NAME="odom"
DIRECTORY_PATH="~/robocomp/components/robocomp-shadow/insect/lidar_odometry"
session_id=$(qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.addSession)
qdbus org.kde.yakuake /yakuake/tabs org.kde.yakuake.setTabTitle "$session_id" "$TAB_NAME"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cd $DIRECTORY_PATH"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "cmake . && make -j32"
qdbus org.kde.yakuake /yakuake/sessions runCommandInTerminal $session_id "bin/lidar_odometry etc/config_wb"
qdbus org.kde.yakuake /yakuake/sessions org.kde.yakuake.raiseSession $session_id
