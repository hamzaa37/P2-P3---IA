# I had to run "xhost +Local:*" to allow X server connection -> non-network local connections being added to access control list
# For linux
# xhost +Local:*
# For mac:
/opt/X11/bin/xhost +
docker run -d --name p2 -p 6080:80 -v /dev/shm:/dev/shm -v .:/practica2 dorowu/ubuntu-desktop-lxde-vnc:focal-arm64
#docker run -it -p 6080:80 -v /dev/shm:/dev/shm --network=host --env DISPLAY=docker.for.mac.host.internal:0  --privileged --volume=/Users/hamza/.Xauthority:/root/.Xauthority -v .:/practica2 practica2arm bash
docker exec -it p2 bash