import os
import math
import numpy as np
#import csv



## Read header from orignial file.
def read_header(original_file_txt):
    header = ""
    with open(original_file_txt) as infile:
        for line in infile:
            line_list = line.split()
            try:
                pos_x = float(line_list[0])
                break
            except ValueError:
                header = header + line

    return header



def header_modifier(file_name_txt, area_mat):
    print(file_name_txt)

    file_name_txt_list = file_name_txt[:-4].split('_')
    area_mat_row = int(file_name_txt_list[-2])
    area_mat_column = int(file_name_txt_list[-1])
    header_to_modify = read_header(original_file_txt)
    # print(header_to_modify)
    header_to_modify_WIDTH_pos = header_to_modify.find('WIDTH') + 6
    header_to_modify_HEIGHT_pos = header_to_modify.find('HEIGHT') - 1
    header_to_modify_1 = header_to_modify[:header_to_modify_WIDTH_pos] + str(area_mat[area_mat_row, area_mat_column]) + header_to_modify[header_to_modify_HEIGHT_pos:]
    # print(header_to_modify_1)



#    header_to_modify[header_to_modify_WIDTH_pos:header_to_modify_HEIGHT_pos] = area_mat[area_mat_row, area_mat_column]

    header_to_modify_POINTS_pos = header_to_modify_1.find('POINTS') + 7
    header_to_modify_DATA_pos = header_to_modify_1.find('DATA') - 1
    modified_header = header_to_modify_1[:header_to_modify_POINTS_pos] + str(area_mat[area_mat_row, area_mat_column]) + header_to_modify_1[header_to_modify_DATA_pos:]


    # print(area_mat_row, area_mat_column)
    # print(modified_header)

    return modified_header




if __name__ == '__main__':
    original_file_pcd = "melaten_ascii.pcd"
    original_road_pcd = "melaten_road_ascii.pcd"
    path = os.getcwd()

    ## Check if the original file is a .pcd file.
    post = str()
    for ch in original_file_pcd[::-1]:
        if ch == '.':
            break
        post += ch

    if post != 'dcp':
        print("Orignial file is not a .pcd file!")
        exit()

    ## Check if the original road file is a .pcd file.
    post_road = str()
    for ch in original_road_pcd[::-1]:
        if ch == '.':
            break
        post_road += ch

    if post_road != 'dcp':
        print("Orignial road file is not a .pcd file!")
        exit()

    ## Rename the original .pcd file to .txt file.
    original_file_name = original_file_pcd[0:-4]
    original_file_txt = original_file_name + ".txt"
    os.rename(original_file_pcd, original_file_txt)

    ## Rename the original .pcd road file to .txt road file.
    original_road_name = original_road_pcd[0:-4]
    original_road_txt = original_road_name + ".txt"
    os.rename(original_road_pcd, original_road_txt)



    ## Read line to line to find max/min x/y.
    ### Init. min_x, max_x, min_y, max_y.
    with open(original_file_txt) as infile:
        for line in infile:
            line_list = line.split()
    #        print(line_list[0])
            try:
                pos_x = float(line_list[0])
                pos_y = float(line_list[1])
                min_x = pos_x
                max_x = pos_x
                min_y = pos_y
                max_y = pos_y
                print(min_x, max_x, min_y, max_y)
                break
            except ValueError:
                pass



    ### Read line to line and update min_x, max_x, min_y, max_y.
    with open(original_file_txt) as infile:
        for line in infile:
            line_list = line.split()
    #        print(line_list)
            try:
                pos_x = float(line_list[0])
                pos_y = float(line_list[1])
                if pos_x < min_x:
                    min_x = pos_x
                if pos_x > max_x:
                    max_x = pos_x
                if pos_y < min_y:
                    min_y = pos_y
                if pos_y > max_y:
                    max_y = pos_y
            except ValueError:
                pass

        print(min_x, max_x, min_y, max_y)

    ## Create a new folder to save small .pcd file, if there are some points in this area.
    ### Create a new folder
    try:
        os.mkdir("splitted_pcd_files") ########################################## here the folder name
    except:
        pass

    ### Cut the whole map to small pieces.
    min_x_area = int(math.floor((min_x + 15)/30))
    max_x_area = int(math.floor((max_x + 15)/30))
    min_y_area = int(math.floor((min_y + 15)/30))
    max_y_area = int(math.floor((max_y + 15)/30))

    print(min_x_area, max_x_area, min_y_area, max_y_area)

    ### Create a matrix to add up point counts in every piece.
    area_mat = np.zeros([(max_y_area - min_y_area + 1),(max_x_area - min_x_area + 1)], int)
    #print(area_mat)

    ### Create a matrix to add up point counts in every piece. (for header.)
    area_mat_points_num = np.zeros([(max_y_area - min_y_area + 1),(max_x_area - min_x_area + 1)], int)
    #print(area_mat_points_num)

    ### Create a matrix to add up road point counts in every piece.
    area_mat_road = np.zeros([(max_y_area - min_y_area + 1),(max_x_area - min_x_area + 1)], int)
    #print(area_mat_road)


##########################################################################################################
    with open(original_file_txt) as infile:  # for header
        for line in infile:
            line_list = line.split()
            #        print(line_list)
            try:
                pos_x = float(line_list[0])
                pos_y = float(line_list[1])
                pos_x_area = int(math.floor((pos_x + 15) / 30))
                pos_y_area = int(math.floor((pos_y + 15) / 30))
                area_mat_points_num[-pos_y_area + max_y_area, pos_x_area - min_x_area] += 1

            except ValueError:
                pass

    print(area_mat_points_num)

##########################################################################################################
    with open(original_road_txt) as infile: # for road restitching
        for line in infile:
            line_list = line.split()
    #        print(line_list)
            try:
                pos_x = float(line_list[0])
                pos_y = float(line_list[1])
                pos_x_area = int(math.floor((pos_x + 15)/30))
                pos_y_area = int(math.floor((pos_y + 15)/30))
                area_mat_road[-pos_y_area + max_y_area, pos_x_area - min_x_area] += 1

            except ValueError:
                pass

    area_mat_road[area_mat_road < 1000] = 0
    area_mat_road[area_mat_road > 0] = 1

    print(area_mat_road)






#    with open(path + "/splitted_pcd_files/" + 'pcd_index.csv', 'w') as file:
#        csv_writer = csv.writer(file)
#        csv_head = ["good", "bad"]
#        csv_writer.writerow(csv_head)


##########################################################################################################

    with open(original_file_txt) as infile:
        for line in infile:
            line_list = line.split()
    #        print(line_list)
            try:
                pos_x = float(line_list[0])
                pos_y = float(line_list[1])
                pos_x_area = int(math.floor((pos_x + 15)/30))
                pos_y_area = int(math.floor((pos_y + 15)/30))
                area_mat[-pos_y_area + max_y_area, pos_x_area - min_x_area] += 1  # read one line (point), find which box it belongs to, this box counter add 1.

                if area_mat[-pos_y_area + max_y_area, pos_x_area - min_x_area] == 1:  # create a new pcl file in .txt

                    if area_mat_road[-pos_y_area + max_y_area, pos_x_area - min_x_area] == 1:  # Do something to road grid:
                        spiltted_pcd_file_txt = original_file_name + "_r_" + str(-pos_y_area + max_y_area) + "_" + str(pos_x_area - min_x_area) + ".txt"
                    else:  #  Do something to non-road grid
                        spiltted_pcd_file_txt = original_file_name + "_n_" + str(-pos_y_area + max_y_area) + "_" + str(pos_x_area - min_x_area) + ".txt"
                    print(spiltted_pcd_file_txt)
                    print(pos_y_area, max_y_area, pos_x_area, min_x_area)
                    print(area_mat[-pos_y_area + max_y_area, pos_x_area - min_x_area])

    #                orignial_file
    #                original_header =

                    spiltted_pcd_file = open(path + "/splitted_pcd_files/" + spiltted_pcd_file_txt, "w")
                    modified_header = header_modifier(spiltted_pcd_file_txt, area_mat_points_num)

                    spiltted_pcd_file.write(modified_header) ## Should be modified after all the points be written.
                    spiltted_pcd_file.write(line)
                    spiltted_pcd_file.close()

                if area_mat[-pos_y_area + max_y_area, pos_x_area - min_x_area] > 1:  #  add a new line in pcl file

                    if area_mat_road[-pos_y_area + max_y_area, pos_x_area - min_x_area] == 1:  # Do something to road grid:
                        spiltted_pcd_file_txt = original_file_name + "_r_" + str(-pos_y_area + max_y_area) + "_" + str(pos_x_area - min_x_area) + ".txt"
                    else:  #  Do something to non-road grid
                        spiltted_pcd_file_txt = original_file_name + "_n_" + str(-pos_y_area + max_y_area) + "_" + str(pos_x_area - min_x_area) + ".txt"



                    spiltted_pcd_file = open(path + "/splitted_pcd_files/" + spiltted_pcd_file_txt, "a")
                    spiltted_pcd_file.write(line)
                    spiltted_pcd_file.close()


            except ValueError:
                pass

    print(area_mat)

    splitted_pcd_files_path = path + "/splitted_pcd_files/"
    file_list = os.listdir(splitted_pcd_files_path)
    for file in file_list:
        portion = os.path.splitext(file)
        if portion[1] == ".txt":
            newname = portion[0] + ".pcd"
            filenamedir = splitted_pcd_files_path + file  # All files are now with .txt in /splitted_pcd_files folder.
            newnamedir = splitted_pcd_files_path + newname  # Change all files name to .pcd
            os.rename(filenamedir, newnamedir)  # Change all files name to .pcd

    os.rename(original_file_txt, original_file_pcd)  # Change back the original map file name to .pcd
    os.rename(original_road_txt, original_road_pcd)  # Change back the original map file name to .pcd





