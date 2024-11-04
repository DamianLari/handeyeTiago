

FROM ros:noetic

USER root

RUN apt-get update && apt-get install -y \
    build-essential \
    sudo \
    terminator \
    iproute2 \
    gedit \
    lsb-release \
    lsb-core \
    wget \
    nano \
    python3 \
    python3-pip \
    ros-noetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Assurer que 'python' pointe vers 'python3'
RUN ln -s /usr/bin/python3 /usr/bin/python

# Installer les librairies Python nécessaires
RUN pip3 install --no-cache-dir \
    opencv-python \
    rosbag

# Ajouter les commandes pour maintenir le terminal ouvert pour le développement
CMD ["terminator"]

#FROM ros:noetic

#USER root

#RUN apt-get update

#RUN apt-get install -y build-essential sudo terminator iproute2 gedit lsb-release lsb-core wget nano python