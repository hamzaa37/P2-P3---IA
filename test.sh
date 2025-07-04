
apt update
apt install -y x11vnc xvfb freeglut3-dev libjpeg-dev libopenmpi-dev openmpi-bin openmpi-doc libxmu-dev libxi-dev cmake libboost-all-dev build-essential
cmake .
make
#Xvfb :1 -screen 0 800x600x16 &
#/usr/bin/x11vnc -auth /root/.Xauthority -display :1.0 &
#DISPLAY=:1.0
#export DISPLAY
#./practica2
#firefox 
