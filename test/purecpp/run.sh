sudo docker run --privileged=true -it -v /dev/bus:/dev/bus -v /home/xxx/workspace:/workspace --name=ubuntu --network=host myros:v1 /bin/bash
