import rospy
from std_msgs.msg import String
from PIL import Image


# Import the CSV library
import csv
import os
import shutil

def check_map_raw():

    folder_path = '/home/son/Desktop/dataset_flat/combine/elevation_map_raw'

    for filename in os.listdir(folder_path):
        # Check if the file has the '.png' extension
        if filename.endswith('.png'):

            num_str = filename.split('.')[0]

            # Construct the file path
            file_path = os.path.join(folder_path, filename)

            # Open the image file
            img = Image.open(file_path)

            # Get the image size (width x height)
            size = img.size

            # Print the file name and image size
            print(f'Size: {size[0]} x {size[1]} pixels')
    print("map_raw check done !")
    print("==========================")

def check_map():

    folder_path = '/home/son/Desktop/dataset_flat/combine/elevation_map'

    for filename in os.listdir(folder_path):
        # Check if the file has the '.png' extension
        if filename.endswith('.png'):

            num_str = filename.split('.')[0]

            # Construct the file path
            file_path = os.path.join(folder_path, filename)

            # Open the image file
            img = Image.open(file_path)

            # Get the image size (width x height)
            size = img.size

            # Print the file name and image size
            # print(f'Size: {size[0]} x {size[1]} pixels')

            if (size[0] != 100 or size[1] != 100):
                print(num_str + " is not modified")
                print(f'Size: {size[0]} x {size[1]} pixels')
            

    print("map check done !")
    print("==========================")

def modify_map():

    folder_path = '/home/son/Desktop/dataset_flat/combine/elevation_map_raw'

    for filename in os.listdir(folder_path):
        # Check if the file has the '.png' extension
        if filename.endswith('.png'):

            num_str = filename.split('.')[0]

            # Construct the file path
            file_path = os.path.join(folder_path, filename)

            # Open the image file
            img = Image.open(file_path)

            new_img = img.resize((100,100))

            new_size = new_img.size

            new_img.save("/home/son/Desktop/dataset_flat/combine/elevation_map/" + num_str + ".png")

    print("modify_map done !")
    print("==========================")

def combine_map():
    dest_path = '/home/son/Desktop/dataset_flat/combine/elevation_map_raw'

    ###### first combine map ############################################################################
    src_path = '/home/son/Desktop/dataset_flat/dataset1/elevation_map_aidin81'

    # Get a list of all PNG files in the source folder
    png_files = [f for f in os.listdir(src_path) if f.endswith('.png')]

    # Copy each PNG file to the destination folder
    for png_file in png_files:
        shutil.copy(os.path.join(src_path, png_file), os.path.join(dest_path, png_file))

    last_id = 0

    # Loop over all the files in the folder
    for filename in os.listdir(dest_path):
        # Check if the file has the '.png' extension
        if filename.endswith('.png'):
            # Increment the counter
            last_id += 1

    print("number of map: " + str(last_id))

    #####################################################################################################

    ###### add this module if you want to combine map and change the src_path ###########################

    src_path = '/home/son/Desktop/dataset_flat/dataset1/elevation_map_aidin82'

    # Loop over each file in the source directory
    for file_name in os.listdir(src_path):
        # Check if the current file is a .png file
        if file_name.endswith('.png'):
            # Extract the current number from the file name
            num = int(file_name[:-4])  # Remove '.png' extension and convert to int
            
            # Add the constant to the current number to get the new number
            new_num = num + last_id
            
            # Create the new file name by concatenating the new number with '.png'
            new_file_name = str(new_num) + '.png'
            
            # Copy the file with the new name to the destination directory
            shutil.copy2(os.path.join(src_path, file_name), os.path.join(dest_path, new_file_name))

    last_id = 0

    # Loop over all the files in the folder
    for filename in os.listdir(dest_path):
        # Check if the file has the '.png' extension
        if filename.endswith('.png'):
            # Increment the counter
            last_id += 1

    print("number of map: " + str(last_id))

    #####################################################################################################

    ###### add this module if you want to combine map and change the src_path ###########################

    src_path = '/home/son/Desktop/dataset_flat/dataset2/elevation_map_aidin81'

    # Loop over each file in the source directory
    for file_name in os.listdir(src_path):
        # Check if the current file is a .png file
        if file_name.endswith('.png'):
            # Extract the current number from the file name
            num = int(file_name[:-4])  # Remove '.png' extension and convert to int
            
            # Add the constant to the current number to get the new number
            new_num = num + last_id
            
            # Create the new file name by concatenating the new number with '.png'
            new_file_name = str(new_num) + '.png'
            
            # Copy the file with the new name to the destination directory
            shutil.copy2(os.path.join(src_path, file_name), os.path.join(dest_path, new_file_name))

    last_id = 0

    # Loop over all the files in the folder
    for filename in os.listdir(dest_path):
        # Check if the file has the '.png' extension
        if filename.endswith('.png'):
            # Increment the counter
            last_id += 1

    print("number of map: " + str(last_id))

    #####################################################################################################

    ###### add this module if you want to combine map and change the src_path ###########################

    src_path = '/home/son/Desktop/dataset_flat/dataset2/elevation_map_aidin82'

    # Loop over each file in the source directory
    for file_name in os.listdir(src_path):
        # Check if the current file is a .png file
        if file_name.endswith('.png'):
            # Extract the current number from the file name
            num = int(file_name[:-4])  # Remove '.png' extension and convert to int
            
            # Add the constant to the current number to get the new number
            new_num = num + last_id
            
            # Create the new file name by concatenating the new number with '.png'
            new_file_name = str(new_num) + '.png'
            
            # Copy the file with the new name to the destination directory
            shutil.copy2(os.path.join(src_path, file_name), os.path.join(dest_path, new_file_name))

    last_id = 0

    # Loop over all the files in the folder
    for filename in os.listdir(dest_path):
        # Check if the file has the '.png' extension
        if filename.endswith('.png'):
            # Increment the counter
            last_id += 1

    print("number of map: " + str(last_id))

    #####################################################################################################

    ###### add this module if you want to combine map and change the src_path ###########################

    src_path = '/home/son/Desktop/dataset_flat/dataset3/elevation_map_aidin81'

    # Loop over each file in the source directory
    for file_name in os.listdir(src_path):
        # Check if the current file is a .png file
        if file_name.endswith('.png'):
            # Extract the current number from the file name
            num = int(file_name[:-4])  # Remove '.png' extension and convert to int
            
            # Add the constant to the current number to get the new number
            new_num = num + last_id
            
            # Create the new file name by concatenating the new number with '.png'
            new_file_name = str(new_num) + '.png'
            
            # Copy the file with the new name to the destination directory
            shutil.copy2(os.path.join(src_path, file_name), os.path.join(dest_path, new_file_name))

    last_id = 0

    # Loop over all the files in the folder
    for filename in os.listdir(dest_path):
        # Check if the file has the '.png' extension
        if filename.endswith('.png'):
            # Increment the counter
            last_id += 1

    print("number of map: " + str(last_id))

    #####################################################################################################

    ###### add this module if you want to combine map and change the src_path ###########################

    src_path = '/home/son/Desktop/dataset_flat/dataset3/elevation_map_aidin82'

    # Loop over each file in the source directory
    for file_name in os.listdir(src_path):
        # Check if the current file is a .png file
        if file_name.endswith('.png'):
            # Extract the current number from the file name
            num = int(file_name[:-4])  # Remove '.png' extension and convert to int
            
            # Add the constant to the current number to get the new number
            new_num = num + last_id
            
            # Create the new file name by concatenating the new number with '.png'
            new_file_name = str(new_num) + '.png'
            
            # Copy the file with the new name to the destination directory
            shutil.copy2(os.path.join(src_path, file_name), os.path.join(dest_path, new_file_name))

    last_id = 0

    # Loop over all the files in the folder
    for filename in os.listdir(dest_path):
        # Check if the file has the '.png' extension
        if filename.endswith('.png'):
            # Increment the counter
            last_id += 1

    print("number of map: " + str(last_id))

    print("combine map done !")
    print("==========================")


    #####################################################################################################


def combine_csv():

    ###### first combine csv file #######################################################################

    with open('/home/son/Desktop/dataset_flat/dataset1/dataset_aidin81.csv', 'r') as src_file, open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'w', newline='') as dest_file:
        # Create CSV writer objects
        reader = csv.reader(src_file)
        writer = csv.writer(dest_file)

        # Read the header row and write it to the destination file
        header = next(reader)
        writer.writerow(header)

        modified_rows = []
        
        # Read and write the remaining rows, modifying the data as needed
        for row in reader:
            modified_rows.append(row)
                
        # Write each modified row to the output file
        for row in modified_rows:
            writer.writerow(row)

    with open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'r') as dest_file:
        
        reader = csv.reader(dest_file)

        row_count = len(list(reader))  # Count the number of row
        last_id = row_count - 1

        print("number of row: " + str(last_id))

    #####################################################################################################

    ###### add this module if you want to combine csv file and change the src_path ######################

    with open('/home/son/Desktop/dataset_flat/dataset1/dataset_aidin82.csv', 'r') as src_file, open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'a', newline='') as dest_file:
        # Create CSV writer objects
        reader = csv.reader(src_file)
        writer = csv.writer(dest_file)

        modified_rows = []
        
        # Read and write the remaining rows, modifying the data as needed
        for i, row in enumerate(reader):
            if i > 0:
                value = int(row[0])

                # Add 10 to the value
                value += last_id

                # Update the value in the second column
                row[0] = str(value)

                # Append the modified row to the list
                modified_rows.append(row)
                
        # Write each modified row to the output file
        for row in modified_rows:
            writer.writerow(row)

    with open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'r') as src_file:
        reader = csv.reader(src_file)

        row_count = len(list(reader))  # Count the number of row
        last_id = row_count - 1

        print("number of row: " + str(last_id))

    #####################################################################################################

    ###### add this module if you want to combine csv file and change the src_path ######################

    with open('/home/son/Desktop/dataset_flat/dataset2/dataset_aidin81.csv', 'r') as src_file, open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'a', newline='') as dest_file:
        # Create CSV writer objects
        reader = csv.reader(src_file)
        writer = csv.writer(dest_file)

        modified_rows = []
        
        # Read and write the remaining rows, modifying the data as needed
        for i, row in enumerate(reader):
            if i > 0:
                value = int(row[0])

                # Add 10 to the value
                value += last_id

                # Update the value in the second column
                row[0] = str(value)

                # Append the modified row to the list
                modified_rows.append(row)
                
        # Write each modified row to the output file
        for row in modified_rows:
            writer.writerow(row)

    with open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'r') as src_file:
        reader = csv.reader(src_file)

        row_count = len(list(reader))  # Count the number of row
        last_id = row_count - 1

        print("number of row: " + str(last_id))

    #####################################################################################################

    ###### add this module if you want to combine csv file and change the src_path ######################

    with open('/home/son/Desktop/dataset_flat/dataset2/dataset_aidin82.csv', 'r') as src_file, open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'a', newline='') as dest_file:
        # Create CSV writer objects
        reader = csv.reader(src_file)
        writer = csv.writer(dest_file)

        modified_rows = []
        
        # Read and write the remaining rows, modifying the data as needed
        for i, row in enumerate(reader):
            if i > 0:
                value = int(row[0])

                # Add 10 to the value
                value += last_id

                # Update the value in the second column
                row[0] = str(value)

                # Append the modified row to the list
                modified_rows.append(row)
                
        # Write each modified row to the output file
        for row in modified_rows:
            writer.writerow(row)

    with open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'r') as src_file:
        reader = csv.reader(src_file)

        row_count = len(list(reader))  # Count the number of row
        last_id = row_count - 1

        print("number of row: " + str(last_id))

    #####################################################################################################

    ###### add this module if you want to combine csv file and change the src_path ######################

    with open('/home/son/Desktop/dataset_flat/dataset3/dataset_aidin81.csv', 'r') as src_file, open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'a', newline='') as dest_file:
        # Create CSV writer objects
        reader = csv.reader(src_file)
        writer = csv.writer(dest_file)

        modified_rows = []
        
        # Read and write the remaining rows, modifying the data as needed
        for i, row in enumerate(reader):
            if i > 0:
                value = int(row[0])

                # Add 10 to the value
                value += last_id

                # Update the value in the second column
                row[0] = str(value)

                # Append the modified row to the list
                modified_rows.append(row)
                
        # Write each modified row to the output file
        for row in modified_rows:
            writer.writerow(row)

    with open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'r') as src_file:
        reader = csv.reader(src_file)

        row_count = len(list(reader))  # Count the number of row
        last_id = row_count - 1

        print("number of row: " + str(last_id))

    #####################################################################################################

    ###### add this module if you want to combine csv file and change the src_path ######################

    with open('/home/son/Desktop/dataset_flat/dataset3/dataset_aidin82.csv', 'r') as src_file, open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'a', newline='') as dest_file:
        # Create CSV writer objects
        reader = csv.reader(src_file)
        writer = csv.writer(dest_file)

        modified_rows = []
        
        # Read and write the remaining rows, modifying the data as needed
        for i, row in enumerate(reader):
            if i > 0:
                value = int(row[0])

                # Add 10 to the value
                value += last_id

                # Update the value in the second column
                row[0] = str(value)

                # Append the modified row to the list
                modified_rows.append(row)
                
        # Write each modified row to the output file
        for row in modified_rows:
            writer.writerow(row)

    with open('/home/son/Desktop/dataset_flat/combine/dataset.csv', 'r') as src_file:
        reader = csv.reader(src_file)

        row_count = len(list(reader))  # Count the number of row
        last_id = row_count - 1

        print("number of row: " + str(last_id))

    print("combine csv file done !")
    print("==========================")

    #####################################################################################################

if __name__ == '__main__':
    try:
        # combine_csv()
        # combine_map()
        # # check_map_raw()
        # modify_map()
        check_map()

    except rospy.ROSInterruptException:
        pass