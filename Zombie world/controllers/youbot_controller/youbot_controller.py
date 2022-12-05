"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor
import numpy as np
import cv2
import math
import pprint




from youbot_zombie import *
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs
    
    


#------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    #health, energy, armour in that order 
    robot_info = [100,100,0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0
    
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")
    
    get_all_berry_pos(robot)
    
    robot_not_dead = 1
    
    #------------------CHANGE CODE BELOW HERE ONLY--------------------------
    
    #COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS
    # accelerometer = robot.getDevice("accelerometer")
    # accelerometer.enable(timestep)
    
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    
    # compass = robot.getDevice("compass")
    # compass.enable(timestep)
    
    camera1 = robot.getDevice("ForwardLowResBigFov")
    camera1.enable(timestep)
    
    # camera2 = robot.getDevice("ForwardHighResSmallFov")
    # camera2.enable(timestep)
    
    # camera3 = robot.getDevice("ForwardHighRes")
    # camera3.enable(timestep)
    
    # camera4 = robot.getDevice("ForwardHighResSmall")
    # camera4.enable(timestep)
    
    camera5 = robot.getDevice("BackLowRes")
    camera5.enable(timestep)
    
    camera6 = robot.getDevice("RightLowRes")
    camera6.enable(timestep)
    
    camera7 = robot.getDevice("LeftLowRes")
    camera7.enable(timestep)
    
    # camera8 = robot.getDevice("BackHighRes")
    # camera8.enable(timestep)
    
    # gyro = robot.getDevice("gyro")
    # gyro.enable(timestep)
    
    # lightSensor = robot.getDevice("light sensor")
    # lightSensor.enable(timestep)
    
    # receiver = robot.getDevice("receiver")
    # receiver.enable(timestep)
    
    # rangeFinder = robot.getDevice("range-finder")
    # rangeFinder.enable(timestep)
    
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    # fr.setPosition(float('inf'))
    # fl.setPosition(float('inf'))
    # br.setPosition(float('inf'))
    # bl.setPosition(float('inf'))
    
    
    i=0
           

    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        
        if(robot_info[0] < 0):
           
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        if(timer%2==0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        if(timer%16==0):
            robot_info = update_robot(robot_info)
            timer = 0
        
        if(robot.step(timestep)==-1):
            exit()
            
            
        timer += 1


        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   
         #called every timestep
        
        
        #possible pseudocode for moving forward, then doing a 90 degree left turn
        #if i <100
            #base_forwards() -> can implement in Python with Webots C code (/Zombie world/libraries/youbot_control) as an example or make your own
        
        #if == 100 
            # base_reset() 
            # base_turn_left()  
            #it takes about 150 timesteps for the robot to complete the turn
                 
        #if i==300
            # i = 0
        
        i += 1

        def detect_zombie_for_camera(image, front=False):
        # convert image to numpy array
            img_float32 = np.float32(image)
            time_step_array = []
            # rotate image 90 degrees right and flip on y axis
            img_float32 = cv2.rotate(img_float32, cv2.ROTATE_90_CLOCKWISE)
            img_float32 = cv2.flip(img_float32, 1)
            image = cv2.cvtColor(img_float32, cv2.COLOR_RGB2HSV)
            original_image = cv2.cvtColor(img_float32, cv2.COLOR_RGB2HSV)
            # print image size
            print(image.shape)
            # write image
            # cv2.imwrite(f"image-{i}.png", original_image)
            # remove red background
            for row in range(image.shape[0]):
                count = 0
                for col in range(image.shape[1]):
                    # if pixel is in range of red color
                    if image[row][col][2] >= 200 and image[row][col][2] <= 230:
                        image[row][col] = [255,255,255]
                    # if not front:
                    #     continue

                    if (col == 126):
                        break
                    # get rgb difference between current pixel and next pixel
                    r1 = image[row][col][2]
                    r2 = image[row][col+1][2]
                    g1 = image[row][col][1]
                    g2 = image[row][col+1][1]
                    b1 = image[row][col][0]
                    b2 = image[row][col+1][0]
                    diff = abs(r1-r2) + abs(g1-g2) + abs(b1-b2)
                    percentage_diff = diff / 765 * 100
                    # print(percentage_diff, 'percentage diff')
                    
                    # if difference is less than 10% add to count
                    # print(percentage_diff, count, 'diff', row, col)
                    # if not front:
                    #     continue
                    if percentage_diff < 5:
                        count += 1
                    else:
                        count = 0
                    if count >= 6:
                        try:
                            if (col-6) < 0:
                                image[row][0:col] = [255,255,255]
                            else:
                                image[row][col-10:col] = [255,255,255]
                        except Exception as e:
                            pass
            # do the same process as above but for columsn
            # for col in range(image.shape[1]):
            #     count = 0
            #     for row in range(image.shape[0]):
            #         if (row == 63):
            #             break
            #         r1 = image[row][col][2]
            #         r2 = image[row+1][col][2]
            #         g1 = image[row][col][1]
            #         g2 = image[row+1][col][1]
            #         b1 = image[row][col][0]
            #         b2 = image[row+1][col][0]
            #         diff = abs(r1-r2) + abs(g1-g2) + abs(b1-b2)
            #         percentage_diff = diff / 765 * 100
            #         print(percentage_diff, count, 'diff', row, col)
            #         if diff < 5:
            #             count += 1
            #         else:
            #             count = 0
            #         if count >= 20:
            #             try:
            #                 print('removed pixels between', row-10, row, col)
            #                 # error -IndexError: index 9 is out of bounds for axis 0 with size 9
            #                 # if (row-10 < 0):
            #                 #     image[0:row][col] = [255,255,255]
            #                 #     count = 0
            #                 # else:
            #                 #     image[row-10:row][col] = [255,255,255]
            #                 #     count = 0
            #             except Exception as e:
            #                 print(e)
            #                 pass


 
                   
   
            
                
            

                
            # lower_red = np.array([0, 0, 0])
            # upper_red = np.array([255, 255, 160])
            # mask = cv2.inRange(image, lower_red, upper_red)
            # image[mask > 0] = (255,255,255)
            image[0] = (255,255,255)
            image[-1] = (255,255,255)
            image[:,0] = (255,255,255)
            image[:,-1] = (255,255,255)
            # cv2.imwrite(f"filtered-{i}.png", image)
            img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            img = img.astype(np.uint8)
            # lowpass = cv2.GaussianBlur(img, (7, 7), 0)
            thresh = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,5,10)
            # remove last column
            thresh = thresh[:, :-1]
            thresh = thresh[:, :-1]

            # cv2.imwrite(f"thresh-{i}.png", thresh)
            # if most of thresh is white, then return true
            white_pixels = 0
            for row in range(thresh.shape[0]):
                for col in range(thresh.shape[1]):
                    if thresh[row][col] == 255:
                        white_pixels += 1
            if (white_pixels / (thresh.shape[0] * thresh.shape[1])) > 0.99:
                wall_obj = {
                    "type": 'wall',
                }
                time_step_array.append(wall_obj)


            contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            bounding_boxes = [cv2.boundingRect(c) for c in contours]
            # remove bounding box with (0, 0, 128, 64) dimensions
            bounding_boxes = [b for b in bounding_boxes if b != (0, 0, 128, 64)]
            # remove bounding boxes whose area is less than 50
            bounding_boxes = [b for b in bounding_boxes if b[3] * b[2] > 50]
            # bounding_boxes = [b for b in bounding_boxes if b[2] >  and b[3] > 10]
            unused_boxes = bounding_boxes.copy()
            used_boxes = []
            def remove_box_inside(boxes):
                for idx, box in enumerate(boxes):
                    x, y, w, h = box
                    for box2 in boxes:
                        x2, y2, w2, h2 = box2
                        if box != box2:
                            if x2 < x + w and x2 + w2 > x and y2 < y + h and y2 + h2 > y:
                                boxes.remove(box2)
                return boxes
            # bounding_boxes = merge_boxes(bounding_boxes)
            bounding_boxes = remove_box_inside(bounding_boxes)
            # draw bounding boxes on image in black

            # remove any bounding box with area greater than 100 * 50
            bounding_boxes = [b for b in bounding_boxes if b[3] * b[2] < 50 * 50]



            for box in bounding_boxes:               
                 # if box width is greater than height, and it is in the lower half of the image, and the area is less than 500 then it is probably a foot
                is_foot = False
                x, y, w, h = box
                if w >= h and y > 20 and w * h < 500:
                    is_foot = True
                    cv2.rectangle(original_image, (x, y), (x + w, y + h), (0, 0, 0), 1)
                else:
                    cv2.rectangle(original_image, box, (0,0,0), 1)
                center = (box[0] + box[2]//2, box[1] + box[3]//2)
                likely_color = "unknown"
                # get mean of pixel value around the center of the bounding box
                mean = np.mean(image[center[1]-5:center[1]+5, center[0]-5:center[0]+5])
                # get mean rgb pixel value around the center of the bounding box
                mean_rgb = np.mean(image[center[1]-5:center[1]+5, center[0]-5:center[0]+5], axis=(0,1))
                # get most likely color
                type = 'zombie'
                if (mean_rgb[0] > 245 and mean_rgb[1] > 180):
                    likely_color = "blue" 
                elif (mean_rgb[0] > 225 and mean_rgb[1] > 170):
                    likely_color = "green" 
                elif (mean_rgb[0] > 245 and mean_rgb[1] < 100):
                    likely_color = "purple" 
                elif (mean_rgb[0] > 245 and mean_rgb[1] > 101):
                    likely_color = "turquoise"
                elif(mean_rgb[0] < 220 and mean_rgb[1] < 100):
                    likely_color = "purple"
                    type = 'tree'
        
                # if the pixel value is not black, draw a circle at the center of the bounding box
                if mean != 0:
                    cv2.circle(original_image, center, 1, (0,0,0), 1)
                    cv2.circle(img, center, 1, (0,0,0), 1)
                    # get avg pixel value around center of bounding box
                    bounding_box_x_center = center[0]
                    bounding_box_lowest_y_coordinate = box[1] + box[3]
                    angle_pixel_coordinate = (bounding_box_x_center, bounding_box_lowest_y_coordinate)
                    center_pixel_coordinates = (64,64)
                    angle = math.degrees(math.atan2(angle_pixel_coordinate[1] - center_pixel_coordinates[1], angle_pixel_coordinate[0] - center_pixel_coordinates[0]))
                zombie_obj = {
                    "type": type,
                    "zombie_color_guess": likely_color,
                    "is_close": is_foot,
                    "angle": angle,
                    "meanrgb": mean_rgb,

                }
                # cv2.imwrite(f"sample-{i}.png",original_image)
                time_step_array.append(zombie_obj)
                # uncomment to show images
                
            return time_step_array
        # robot detection
        camera1information = detect_zombie_for_camera(camera1.getImageArray(), True)
        camera5information = detect_zombie_for_camera(camera5.getImageArray())
        camera6information = detect_zombie_for_camera(camera6.getImageArray())
        camera7information = detect_zombie_for_camera(camera7.getImageArray())

        pprint.pprint({
            "camera1 ForwardLowResBigFov": camera1information,
            "camera5 BackLowRes": camera5information,
            "camera6 RightLowRes": camera6information,
            "camera7 LeftLowRes": camera7information
        })


        print(i)

        if (i == 15):
            break

        cv2.waitKey(1)



        
        #make decisions using inputs if you choose to do so
         
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()

