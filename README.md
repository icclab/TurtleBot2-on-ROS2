# turtlebot2

## Initial setup

Udev rules device links are not preserved in the docker container, so you will have to set them up yourself

    sudo chown ros:ros /dev/ttyUSB0
    sudo chown ros:ros /dev/ttyUSB1

    sudo ln -s /dev/ttyUSB0 /dev/kobuki
    sudo ln -s /dev/ttyUSB1 /dev/lidar

Or:

    sudo ln -s /dev/ttyUSB1 /dev/kobuki
    sudo ln -s /dev/ttyUSB0 /dev/lidar