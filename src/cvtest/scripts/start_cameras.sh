#!/bin/bash

# Start downward camera (camera 0) for linear regression
echo "Starting downward camera (ID: 0)..."
ros2 run camera_ros camera_node --ros-args \
    -p camera_id:=0 \
    -p frame_id:=downward_camera_frame \
    -p fps:=30.0 \
    -p width:=640 \
    -p height:=480 \
    -r image_raw:=/camera/downward/image_raw \
    -r camera_info:=/camera/downward/camera_info &

DOWNWARD_PID=$!

# Wait a moment before starting the second camera
sleep 2

# Start front camera (camera 1) for color segmentation  
echo "Starting front camera (ID: 1)..."
ros2 run camera_ros camera_node --ros-args \
    -p camera_id:=1 \
    -p frame_id:=front_camera_frame \
    -p fps:=30.0 \
    -p width:=640 \
    -p height:=480 \
    -r image_raw:=/camera/front/image_raw \
    -r camera_info:=/camera/front/camera_info &

FRONT_PID=$!

echo "Both cameras started!"
echo "Downward camera PID: $DOWNWARD_PID"
echo "Front camera PID: $FRONT_PID"
echo "To stop cameras, run: kill $DOWNWARD_PID $FRONT_PID"

# Keep script running until Ctrl+C
trap "echo 'Stopping cameras...'; kill $DOWNWARD_PID $FRONT_PID; exit" INT
wait
