xhost +
docker run -it --rm --net host --ipc host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTHORITY \
    -v ./ros_ws/:/root/ros_workspace \
    --name tb3_autonomous_exploration \
    ros_jazzy:tb3_autonomous_exploration bash
    
    