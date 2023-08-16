import rospy
from std_msgs.msg import String
from PIL import Image


# Import the CSV library
import csv
import os


def SizeChanger():
    
    # src_path = "/home/son/Desktop/dataset/dataset2/elevation_map_data1/"
    # dst_path = "/home/son/Desktop/dataset/dataset2/elevation_map_data1_70/"

    # for file_name in os.listdir(src_path):
    #     # Check if the current file is a .png file
    #     if file_name.endswith('.png'):
    #         # Extract the current number from the file name
    #         # num = int(file_name[:-4])  # Remove '.png' extension and convert to int
            
    #         # # Add the constant to the current number to get the new number
    #         # new_num = num
            
    #         # # Create the new file name by concatenating the new number with '.png'
    #         # new_file_name = str(new_num) + '.png'

    #         # change pixel to 100 x 100
    #         file_path = os.path.join(src_path, file_name)

    #         img = Image.open(file_path)

    #         new_img = img.resize((70,70))

    #         # new_img.save("/home/son/Desktop/dataset/dataset3/elevation_map/" + new_file_name)
    #         new_img.save(dst_path + file_name)
    

    # 자를 영역 좌표 설정
    left = 15
    top = 15
    right = 85
    bottom = 85

    # 폴더 경로 설정
    src_path = "/home/son/Desktop/dataset/dataset2/elevation_map_data1_100/"
    dst_path = "/home/son/Desktop/dataset/dataset2/elevation_map_data1_crop/"

    # 폴더 내의 모든 파일 검색
    for filename in os.listdir(src_path):
        if filename.endswith(".png"):
            # 이미지 열기
            image_path = os.path.join(src_path, filename)
            image = Image.open(image_path)

            # 이미지 자르기
            cropped_image = image.crop((left, top, right, bottom))

            # 자른 이미지 저장
            # save_path = os.path.join(dst_path, filename)
            cropped_image.save(dst_path + filename)

    print("* [Change size of image] Done !\n")

if __name__ == '__main__':
    try:

        SizeChanger()

    except rospy.ROSInterruptException:
        pass
