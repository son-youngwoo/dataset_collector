import rospy
from std_msgs.msg import String
from PIL import Image


# Import the CSV library
import csv
import os

def check_dataset():

    csv_num = -1
    map_num = 0

    csv_path = csv_dst_path
    map_path = map_dst_path

    with open(csv_path, 'r') as src_file:
        reader = csv.reader(src_file)
        
        for row in reader:
            csv_num += 1

    for file_name in os.listdir(map_path):
        # Check if the file has the '.png' extension
        if file_name.endswith('.png'):
            
            map_num += 1

            num_str = file_name.split('.')[0]

            file_path = os.path.join(map_path, file_name)

            img = Image.open(file_path)

            size = img.size

            if (size[0] != 70 or size[1] != 70):
                print(num_str + " is not modified")
                print(f'Size: {size[0]} x {size[1]} pixels')

    if csv_num == map_num:
        print("* [Compare the number of map and csv data] Done !")
    else:
        print("* The number of map and csv data does not match.")

def merge_map():

    last_id = 0
    
    map_src_path_list = map_src_path.split(',') # string to list

    dst_path = map_dst_path
    src_paths = map_src_path_list

    for src_path in src_paths:

        for file_name in os.listdir(src_path):
            # Check if the current file is a .png file
            if file_name.endswith('.png'):
                # Extract the current number from the file name
                num = int(file_name[:-4])  # Remove '.png' extension and convert to int
                
                # Add the constant to the current number to get the new number
                new_num = num + last_id
                
                # Create the new file name by concatenating the new number with '.png'
                new_file_name = str(new_num) + '.png'

                # change pixel to 100 x 100
                file_path = os.path.join(src_path, file_name)

                img = Image.open(file_path)

                new_img = img.resize((70,70))

                # new_img.save("/home/son/Desktop/dataset/dataset3/elevation_map/" + new_file_name)
                new_img.save(dst_path + new_file_name)
                
        # Loop over all the files in the folder
        for file_name in os.listdir(src_path):
            # Check if the file has the '.png' extension
            if file_name.endswith('.png'):
                last_id += 1

        print("number of map data: " + str(last_id))

    print("* [Merge map] Done !\n")

def merge_csv():

    last_id = 0

    csv_src_path_list = csv_src_path.split(',') # string to list

    dst_file = open(csv_dst_path, 'w', newline='')
    src_paths = csv_src_path_list

    for src_path in src_paths:
        with open(src_path, 'r') as src_file:

            # Create CSV writer objects
            reader = csv.reader(src_file)
            writer = csv.writer(dst_file)

            if last_id == 0:
                header = next(reader)
                writer.writerow(header)
                
                src_file.seek(0)

            rows = []

            for i, row in enumerate(reader):
                if i > 0:
                    value = int(row[0])

                    value += last_id

                    row[0] = str(value)

                    rows.append(row)
                    
            for row in rows:
                writer.writerow(row)
                last_id = last_id + 1

        print("number of csv data: " + str(last_id))

    dst_file.close()

    print("* [Merge csv file] Done !\n")

if __name__ == '__main__':
    try:

        csv_dst_path = rospy.get_param('/dataset_merge/csv_dst_path')
        csv_src_path = rospy.get_param('/dataset_merge/csv_src_path')

        map_dst_path = rospy.get_param('/dataset_merge/map_dst_path')
        map_src_path = rospy.get_param('/dataset_merge/map_src_path')

        merge_csv()
        merge_map()
        check_dataset()

    except rospy.ROSInterruptException:
        pass