# vector_ros_driver
This repository contains a ROS package to control physical Anki Vector home robot. For more information please visit [beta_b0t/vector_ros](https://github.com/betab0t/vector_ros) main repository. In [nilseuropa/vector_description](https://github.com/nilseuropa/vector_description) you can find a detailed URDF model prepared for Gazebo simulation. ( These two repositories are going to be merged soon. )

# Setup
## Docker Image
It's highly recommended to use the supplied Dockerfile instead of installing directly on your machine mainly because of the tricky setup required to run Python 3 properly on ROS. If you wish to do this setup by yourself then [beta_b0t wrote a blog post explaining how](https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674) that you can use, else follow these instructions:
1. Install [Docker](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-16-04) if you dont have it already installed
```sh
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install -y docker-ce
```

2. Run using pre-built image from DockerHub, params are passed view environment variables:
```sh
sudo docker run -e ANKI_USER_EMAIL=<EMAIL> -e ANKI_USER_PASSWORD=<PASSWORD> -e VECTOR_IP=<VECTOR_IP> -e VECTOR_SERIAL=<VECTOR_SERIAL> -e VECTOR_NAME=<VECTOR_NAME> --network host -it betab0t/vector-ros-driver
```
*Notice! Use your [Anki Developer](https://developer.anki.com/) username and password*

3. You can now execute ```rostopic list``` on your host machine the verify everything works

# Topics
* `/vector/camera`  *(sensor_msgs/Image)*

* `/vector/cmd_vel` *(geometry_msgs/Twist)*

* `/vector/gyro` *(geometry_msgs/Vector3)*

* `/vector/accelero` *(geometry_msgs/Vector3)*

# Services

* `/vector/battery_state`

* `/vector/set_head_angle`

* `/vector/set_lift_height`

* `/vector/anim_list`

* `/vector/say_text`

# Actions

* `/vector/play_animation` - Play animation by name.

# FAQ
#### Can’t find robot name
Your Vector robot name looks like “Vector-E5S6”. Find your robot name by placing Vector on the charger and double-clicking Vector’s backpack button.

#### Can’t find serial number
Your Vector’s serial number looks like “00e20142”. Find your robot serial number on the underside of Vector. Or, find the serial number from Vector’s debug screen: double-click his backpack, move his arms up and down, then look for “ESN” on his screen.

#### Can’t find Vector’s IP address
Your Vector IP address looks like “192.168.40.134”. Find the IP address from Vector’s debug screen: double-click his backpack, move his arms up and down, then look for “IP” on his screen.

# Changelog
* IMU wrapper added
