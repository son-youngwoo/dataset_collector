import rospy
from std_msgs.msg import String
from PIL import Image


# Import the CSV library
import csv
import os

def PixelChanger():
    
    src_path = "/home/son/Desktop/dataset/dataset2/elevation_map_data1/"
    dst_path = "/home/son/Desktop/dataset/dataset2/elevation_map_data1_100/"

    for file_name in os.listdir(src_path):
        # Check if the current file is a .png file
        if file_name.endswith('.png'):
            file_path = os.path.join(src_path, file_name)

            img = Image.open(file_path)

            new_img = img.resize((100,100))

            new_img.save(dst_path + file_name)
            
    print("* [Change pixel of image] Done !\n")

if __name__ == '__main__':
    try:

        PixelChanger()

    except rospy.ROSInterruptException:
        pass