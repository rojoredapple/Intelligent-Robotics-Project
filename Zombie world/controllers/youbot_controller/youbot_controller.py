"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor
import numpy as np
import cv2
import math




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
    
    # camera1 = robot.getDevice("ForwardLowResBigFov")
    # camera1.enable(timestep)
    
    # camera2 = robot.getDevice("ForwardHighResSmallFov")
    # camera2.enable(timestep)
    
    # camera3 = robot.getDevice("ForwardHighRes")
    # camera3.enable(timestep)
    
    # camera4 = robot.getDevice("ForwardHighResSmall")
    # camera4.enable(timestep)
    
    camera5 = robot.getDevice("BackLowRes")
    camera5.enable(timestep)
    
    # camera6 = robot.getDevice("RightLowRes")
    # camera6.enable(timestep)
    
    # camera7 = robot.getDevice("LeftLowRes")
    # camera7.enable(timestep)
    
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

        
        # different thresholding methods to try
        # cv2.ADAPTIVE_THRESH_MEAN_C
        # cv2.ADAPTIVE_THRESH_GAUSSIAN_C
        # cv2.THRESH_BINARY
        # cv2.THRESH_BINARY+cv.THRESH_OTSU


        # using webots camera object take image every timestep and interpret with opencv and don't get errors
        image = camera5.getImageArray()
        # convert image to numpy array
        img_float32 = np.float32(image)
        # rotate image 90 degrees right and flip on y axis
        img_float32 = cv2.rotate(img_float32, cv2.ROTATE_90_CLOCKWISE)
        img_float32 = cv2.flip(img_float32, 1)
        image = cv2.cvtColor(img_float32, cv2.COLOR_RGB2HSV)
        original_image = image.copy()
        cv2.imwrite(f"original-sample-{i}.png",image)





        # iterate through every pixel
        for row in range(image.shape[0]):
            for col in range(image.shape[1]):
                # if pixel is in range of red color
                if image[row][col][2] >= 200 and image[row][col][2] <= 225:
                    image[row][col] = [255,255,255]

        

        # if pixel

        # find and remove the most common colour in the image
        

        # change pixels between rgb(205,0,15) and rgb(222,0,23) to white
        # lower_red = np.array([0, 0, 220])
        # upper_red = np.array([255, 255, 240])
        # mask = cv2.inRange(image, lower_red, upper_red)
        # image[mask > 0] = (255, 255, 255)

        # change pixels between rgb(108,1,39) and rgb(108,1,38) to white
        lower_red = np.array([0, 0, 0])
        upper_red = np.array([255, 255, 160])
        mask = cv2.inRange(image, lower_red, upper_red)
        image[mask > 0] = (255,255,255)



        
        

        
        image[0] = (255,255,255)
        image[-1] = (255,255,255)


        image[:,0] = (255,255,255)
        image[:,-1] = (255,255,255)
       
        



        




        


        # gaussain blur to remove noise
        # image = cv2.GaussianBlur(image, (5, 5), 0)

        # zombies are vertical creatures that have a humanoid shape, identify them by their shape
        # use opencv to find contours of the zombies
        img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        

        img = img.astype(np.uint8)

        # remove darkest pixels from image
        # img[img < 64] = 0
        # img[img < 45] = 0
        
        # detect most common color in image and remove it

        
        

   
        
        cv2.imwrite(f"bw-{i}.png",img)
        # ret,thresh = cv2.threshold(img,127,0,cv2.THRESH_BINARY)
        # thresh1 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)

        thresh = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
        # thresh0 = cv2.Canny(img,50,100)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        print('countors',len(contours))
        
        # remove largest contour
        # get avergae pixel value of each contour

        # merge countours that are close together

        # draw contours on image as blue lines

        # cv2.drawContours(original_image, contours, -1, (255,0,0), 1)
        # cv2.drawContours(img, contours, -1, (255,255,255), 1)
        cv2.imwrite(f"thresh-{i}.png",thresh)
        # find the bounding box of each contour
        bounding_boxes = [cv2.boundingRect(c) for c in contours]
        # remove bounding box with (0, 0, 128, 64) dimensions
        bounding_boxes = [b for b in bounding_boxes if b != (0, 0, 128, 64)]
        for box in bounding_boxes:
            print('before merge', box)

        # merge bounding boxes that overlap
        unused_boxes = bounding_boxes.copy()
        used_boxes = []

        def merge_boxes(boxes):
            merged_boxes = []
            for idx, box in enumerate(boxes):
                x, y, w, h = box
                for box2 in boxes:
                    x2, y2, w2, h2 = box2
                    if box != box2 and box2 not in used_boxes:
                        if x2 < x + w and x2 + w2 > x and y2 < y + h and y2 + h2 > y:
                            x = min(x, x2)
                            y = min(y, y2)
                            w = max(x + w, x2 + w2) - x
                            h = max(y + h, y2 + h2) - y
                            print('merged box', (x, y, w, h), 'from', box, box2)
                            merged_boxes.append((x, y, w, h))
                            used_boxes.append(box)
                            if (box in unused_boxes):
                                unused_boxes.remove(box)
                            if (box2 in unused_boxes):
                                unused_boxes.remove(box2)
            for box in unused_boxes:
                merged_boxes.append(box)
            return merged_boxes
        
        
        # bounding_boxes = merge_boxes(bounding_boxes)
        # if a box is inside another box, remove it
        def remove_box_inside(boxes):
            for idx, box in enumerate(boxes):
                x, y, w, h = box
                for box2 in boxes:
                    x2, y2, w2, h2 = box2
                    if box != box2:
                        if x2 < x + w and x2 + w2 > x and y2 < y + h and y2 + h2 > y:
                            boxes.remove(box2)
            return boxes
        # bounding_boxes = remove_box_inside(bounding_boxes)
        # bounding_boxes = remove_box_inside(bounding_boxes)

        # remove bounding boxes that are too small
        bounding_boxes = [b for b in bounding_boxes if b[2] > 10 and b[3] > 10]
        for box in bounding_boxes:
            print('after merge', box)




        # draw bounding boxes on image in black
        for box in bounding_boxes:
            cv2.rectangle(original_image, box, (0,0,0), 1)
            cv2.rectangle(img, box, (0,0,0), 1)
        # find the center of each bounding box

        # get the the center pixel value of each bounding box
        for box in bounding_boxes:
            # get the center of the bounding box
            center = (box[0] + box[2]//2, box[1] + box[3]//2)
            # get mean of pixel value around the center of the bounding box
            mean = np.mean(image[center[1]-5:center[1]+5, center[0]-5:center[0]+5])
            print('pixel value', i ,mean)
            # if the pixel value is not black, draw a circle at the center of the bounding box
            if mean != 0:
                cv2.circle(original_image, center, 1, (0,0,0), 1)
                cv2.circle(img, center, 1, (0,0,0), 1)






        
        # # draw centers on image in black
        # for center in centers:
        #     cv2.circle(image, center, 1, (0,0,0), 2)
        #     cv2.circle(img, center, 1, (0,0,0), 2)



        print(i)
        cv2.imwrite(f"sample-{i}.png",original_image)
        cv2.waitKey(1)
        if (i == 15):
            break


        
        #make decisions using inputs if you choose to do so
         
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
