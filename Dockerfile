FROM ros:melodic-robot
RUN apt-get update
RUN apt-get install -y libusb-dev libusb-1.0.0-dev libudev-dev vim libpcl-dev libopencv-dev ros-melodic-pcl-conversions ros-melodic-camera-info-manager
