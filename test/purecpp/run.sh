sudo docker run --privileged=true -it -v /dev/bus:/dev/bus -v /home/quicktron/workspace/src/ros-feynman-camera/test/purecpp/:/purecpp --name=ubuntu --network=host ubuntu:18.04 /bin/bash
