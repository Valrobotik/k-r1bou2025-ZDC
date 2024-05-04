#!/usr/bin/env python3
from playsound import playsound
import subprocess
import time
import random
import os

import rospy
from std_msgs.msg import Int8, Bool

# Variable de l'état courant

# Callback pour mettre à jour l'état courant
# On insére par défaut 1 pour jouer le son d'initialisation
list_play = [1]
def callback(data: Int8):
    global list_play
    if len(list_play) < 3:
        list_play.append(data.data)

# Fonctions pour gérer la connexion au haut-parleur
def connect_to_speaker(speaker_name):
    connect_command = f"bluetoothctl connect {speaker_name}"
    subprocess.run(connect_command, shell=True, check=True)

# Fonction pour vérifier si le haut-parleur est connecté
def is_speaker_connected(speaker_name):
    check_command = f"bluetoothctl info {speaker_name}"
    try:
        output = subprocess.check_output(check_command.split(), stderr=subprocess.STDOUT).decode("utf-8")
        return "Connected: yes" in output
    except subprocess.CalledProcessError:
        return False

# Mapping des états aux chemins des dossiers de sons
audio_files = {
    1: "/home/valrob/Music/CDFR_robot_sound/initialise",
    2: "/home/valrob/Music/CDFR_robot_sound/team_yellow",
    3: "/home/valrob/Music/CDFR_robot_sound/team_blue",
    4: "/home/valrob/Music/CDFR_robot_sound/pami",
}

# Choix aléatoire du son à jouer parmis le dossier de son choisi
def choosen_sound():
    # Récupère le nom d'un fichier audio aléatoirement dans le dossier
    file_name = random.choice(os.listdir(audio_files[list_play[0]]))
    # Retourne le chemin complet du fichier audio à jouer
    return f"{audio_files[list_play[0]]}/{file_name}"


# Initialisation du noeud ROS
rospy.init_node("bluetooth_sound")
rospy.loginfo("[START] Bluetooth_sound node has started.")
rospy.Subscriber("speaker_choice", Int8, callback)
speaker_state = rospy.Publisher("speaker_state", Bool, queue_size=1)

rospy.sleep(2)
# Publier l'état du haut-parleur
temp = Bool()
temp.data = True
speaker_state.publish(temp)

# Boucle principale
while not rospy.is_shutdown():
    # Vérifier si l'état a changé
    if len(list_play) > 0:
        # Vérifier si le haut-parleur est toujours connecté
        if is_speaker_connected("21:9E:04:77:33:65"):
            # Jouer le son
            rospy.loginfo(f"(SPEAKER) Playing sound {list_play[0]}...")
            playsound(choosen_sound())
        else:
            # Tenter de se connecter au haut-parleur
            rospy.loginfo("(SPEAKER) Speaker not connected. Trying to connect...")
            try:
                connect_to_speaker("21:9E:04:77:33:65")
                rospy.loginfo("(SPEAKER) Connected to speaker.")
                # Attendre 2.5 secondes pour que la connexion soit établie
                time.sleep(2.5)
                # Jouer le son
                rospy.loginfo(f"(SPEAKER) Playing sound {list_play[0]}...")
                playsound(choosen_sound())
            except subprocess.CalledProcessError:
                rospy.logerr("(SPEAKER) Failed to connect to speaker.")
        list_play.pop(0)
        # Réinitialiser le flag
