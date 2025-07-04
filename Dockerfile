FROM dorowu/ubuntu-desktop-lxde-vnc:focal-arm64

WORKDIR /practica2

COPY . .

# RUN bash install.sh
RUN apt-get update -q 
RUN apt-get install -y freeglut3-dev libjpeg-dev libopenmpi-dev openmpi-bin openmpi-doc libxmu-dev libxi-dev cmake libboost-all-dev build-essential \
    && rm -rf /var/lib/apt/lists/*
