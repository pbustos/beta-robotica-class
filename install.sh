#!/bin/bash

# update python3 packages 
sudo pip3 install -U numpy pandas opencv-python PySide2 PySide6 shiboken2 shiboken6 scipy torchvision ultralytics matplotlib

# Install JointBDOE DNN
cd /home/robocomp/software
git clone https://github.com/hnuzhy/JointBDOE
cd JointBDOE
mkdir -p runs/JointBDOE/coco_s_1024_e500_t020_w005/weights

# URL of the element to download
URL="https://huggingface.co/HoyerChou/JointBDOE/resolve/main/coco_s_1024_e500_t010_w005_best.pt?download=true"

# Destination folder
DEST_FOLDER="/home/robocomp/software/JointBDOE/runs/JointBDOE/coco_s_1024_e500_t020_w005/weights"

# Desired filename
FILENAME="best.pt"

# Create the destination folder if it doesn't exist
mkdir -p "$DEST_FOLDER"

# Download the element and save it with the specified filename
curl -L -o "$DEST_FOLDER/$FILENAME" "$URL"

# Check if the download was successful
if [[ $? -eq 0 ]]; then
    echo "Download successful: $DEST_FOLDER/$FILENAME"
else
    echo "Download failed"
fi