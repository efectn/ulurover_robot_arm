.PHONY: start stop status install_deps build rqt_graph panda_servo_launch panda_servo_keyboard panda_joystick_publisher

STATUS := $(shell docker compose ps | grep ros2_humble_dev > /dev/null && echo 1 || echo 0)

start:
ifeq ($(STATUS), 1)
	$(info Docker compose is already running)
else
	$(info Docker compose is not running)
	docker compose up -d
endif

stop:
ifeq ($(STATUS), 1)
	$(info Docker compose is running)
	docker compose down
else
	$(info Docker compose is not running)
endif

status :
ifeq ($(STATUS), 1)
	$(info Docker compose is running)
else
	$(info Docker compose is not running)
endif

install_deps: start
	docker exec ros2_humble_dev bash -c "apt update && apt install -y python3-rosdep"
	docker exec ros2_humble_dev bash -c "cd /root/ros_ws && rosdep install --from-paths src --ignore-src -r -y"

build:
	docker exec ros2_humble_dev bash -c "source /opt/ros/humble/setup.bash && cd /root/ros_ws && colcon build --symlink-install"

rqt_graph:
	docker exec ros2_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /root/ros_ws/install/setup.bash && ros2 run rqt_graph rqt_graph"

panda_servo_launch:
	docker exec -it ros2_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /root/ros_ws/install/setup.bash && ros2 launch panda_servo_control servo.launch.py"

panda_servo_keyboard:
	docker exec -it ros2_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /root/ros_ws/install/setup.bash && ros2 run panda_servo_control servo_keyboard"

panda_joystick_publisher:
	docker exec -it ros2_humble_dev bash -c "source /opt/ros/humble/setup.bash && source /root/ros_ws/install/setup.bash && ros2 run panda_servo_control joystick_servo_publisher"