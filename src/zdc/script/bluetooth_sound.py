#!/usr/bin/env python3
from playsound import playsound
import subprocess
import time

import rospy
from std_msgs.msg import Int8, Bool

# Variable de l'état courant

# Callback pour mettre à jour l'état courant
list_play = [3]
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

# Mapping des états aux chemins de fichiers audio
audio_files = {
    1: "/home/valrob/catkin_ws/src/zdc/sounds/initialise/1-initialise.wav",
    2: "/home/valrob/catkin_ws/src/zdc/sounds/team_select/2-jaune.mp3",
    3: "/home/valrob/catkin_ws/src/zdc/sounds/team_select/3-rouge.mp3",
}


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
            playsound(audio_files[list_play[0]])
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
                playsound(audio_files[list_play[0]])
            except subprocess.CalledProcessError:
                rospy.logerr("(SPEAKER) Failed to connect to speaker.")
        list_play.pop(0)
        # Réinitialiser le flag
