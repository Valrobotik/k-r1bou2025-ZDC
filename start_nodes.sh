#!/bin/bash
# Demande de lIP de la raspy à lutilisateur
read -p "Entrez l'adresse IP de la Raspy: " -i 192.168. -e IP
# Kill de l'ancien processus
echo "Killing old processes..."
pkill -f robotpos_calc.py
pkill -f bluetooth_sound.py
sleep 2
# Récupération de l'IP locale
IP_ADDRESS=$(hostname -I | awk '{print $1}')
# Export de la donnée dans la variable ROS_IP
ROS_IP=$IP_ADDRESS
# Export de la donnée dans la variable ROS_MASTER_URI
ROS_MASTER_URI="http://$IP:11311/"
# Export
export ROS_IP
export ROS_MASTER_URI
# Print des données pour vérif
echo ROS_IP=$ROS_IP
echo ROS_MASTER_URI=$ROS_MASTER_URI

# On se dirige dans le dossier du projet et on build
cd /home/valrob/k-r1bou2025-ZDC
catkin_make

# On source le setup du projet ROS
source /home/valrob/k-r1bou2025-ZDC/devel/setup.bash

# Lance les noeuds camera et bluetooth

rosrun zdc robotpos_calc.py & rosrun zdc bluetooth_sound.py
