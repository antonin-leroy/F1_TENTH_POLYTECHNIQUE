#!/usr/bin/env python3
import rospy
import csv
import math
import atexit
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# --- CONFIGURATION ---
FILENAME = 'waypoints.csv'
TOPIC_NAME = '/odom' 

# Ouverture immédiate du fichier
file = open(FILENAME, 'w')
writer = csv.writer(file)

print(f"--- LOGGER SIMULATEUR ---")
print(f"Enregistrement dans : {FILENAME}")
print(f"En attente de données sur : {TOPIC_NAME}")

def save_waypoint(data):
    # 1. Position X, Y
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    # 2. Orientation (Quaternion -> Euler -> Yaw)
    # C'est utile pour savoir dans quelle direction regarde la voiture
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    # 3. Vitesse réelle
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    speed = math.sqrt(vx**2 + vy**2)

    # 4. Écriture : X, Y, Angle, Vitesse
    writer.writerow([x, y, yaw, speed])
    
    # Feedback visuel pour être sûr que ça marche
    # On affiche un point tous les 0.5 mètres environ pour ne pas spammer le terminal
    if x % 0.5 < 0.05: 
        print(f"Capturé -> x={x:.2f}, y={y:.2f}, speed={speed:.2f}")

def shutdown():
    file.close()
    print("\n--- TERMINE ---")
    print(f"Fichier {FILENAME} sauvegardé avec succès.")

def listener():
    rospy.init_node('wp_logger_simu', anonymous=True)
    rospy.Subscriber(TOPIC_NAME, Odometry, save_waypoint)
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    listener()