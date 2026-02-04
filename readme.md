docker run -it \
  --name chess_ros_container \
  --net=host \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/root/ros2_ws \
  chess_ros \
  bash
# This allows the venv to see the ROS 2 libraries in /opt/ros/...
python3 -m venv --system-site-packages .venv