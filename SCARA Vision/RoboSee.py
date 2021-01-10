# import the necessary packages
import re
import PyCmdMessenger
import math
from pyimagesearch.shapedetector import ShapeDetector
from pyimagesearch.colorlabeler import ColorLabeler
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
import cv2
import time

#//////////------initializing the robot's kinematics and control------//////////

#coordinate of refObject WRT to robot's co-ordinate frame
refObjX=-175
refObjY=245
width=40#mm of the referance object


#Initial Position

homeX=390
homeY=0

def moveInvKin(objCoords,refObjX,refObjY):  #function for inverse kinemics motor control
    arduino = PyCmdMessenger.ArduinoBoard("COM3",baud_rate=115200)

    commands = [["motor_steps","fff"],
                ["motor_run",""],
                ["motor_state_prep","f"],
                ["motor_state_run","f"]
                ]

    # Initialize the messenger
    c = PyCmdMessenger.CmdMessenger(arduino,commands) 

    Len1= 250  #length of link 1 in mm
    Len2= 140#139  #length of link 2 in mm
    theta_S_prev=0
    theta_E_prev=0
    z_prev=0
    #some constants
    SumLenSqrd = Len1*Len1+Len2*Len2
    ProdOfLens = 2*Len1*Len2
    DiffLenSqrd= Len1*Len1-Len2*Len2
    

    #steps per degree*(SPD)
    microstepping= 1/16
    SPD_1=(16/1.8)*(40/20) #40/20 is the gear ratio
    SPD_2=(16/1.8)*(40/16)

    #steps per mm(for motor MZ,z-axis)
    SPM=800/8 #since lead is 8 mm     (goes 8 mm in 800 steps(1/4 micro stepping) .Therefore, Z-axis accuracy is 0.01mm)
    #c.send("motor_steps",0,0,15000)
    #c.send("motor_run")        
    for i in range(len(objCoords)):
        objX,objY=objCoords[i]
        objX=refObjX+objX
        objY=refObjY+objY
        print(objX,objY)
    
    
        XkinPos=objX
        YkinPos=objY
        z=0
        Temp1= XkinPos*XkinPos+YkinPos*YkinPos
        Temp2= (Temp1- SumLenSqrd)/ProdOfLens
    
        lefthand=0
        if (abs(Temp2)<=1) :
            #Inverse Kinematics 
            if(XkinPos>0): #always gives right hand calculation
                XkinPos=-XkinPos
                lefthand=1
            theta_E= math.acos(Temp2) 
            theta_Q= math.acos((Temp1+DiffLenSqrd)/(2*Len1*math.sqrt(Temp1)))
            arctan=math.atan2(YkinPos,XkinPos)
            theta_S= arctan - theta_Q
		
		
            theta_E=(180/(22/7))*theta_E
            theta_S=((180/(22/7))*theta_S)
            if(YkinPos<0 and lefthand==0):
                theta_S=360+theta_S
            if(lefthand==1):
                theta_E=-theta_E
                theta_S=180-theta_S
                if(YkinPos<0):
                    theta_S=theta_S-360
                lefthand=0
            if(theta_S<0 or theta_S>180):
                print("Joint0 limit exceeded! Try lowering y coordinate")
		#motor control
		
		#z=z_prev-ZkinPos
            theta_S_new=theta_S-theta_S_prev
            theta_E_new=theta_E-theta_E_prev
            steps_M0=theta_S_new*SPD_1 #positive is clockwise and negative is anti-clockwise 
            steps_M1=theta_E_new*SPD_2
            steps_M2=0#z*SPM
		
            c.send("motor_steps",steps_M0,steps_M1,steps_M2)
            msg= c.receive()
            print(msg)
        
            print("Moving")
            c.send("motor_run")
            msg = c.receive()
            print(msg)
        
		#z_prev=ZkinPos
            theta_E_prev=theta_E
            theta_S_prev=theta_S	
		#-------------
	
	
        else :
            print("Co-ordinate of object is beyond reachable workspace")
        time.sleep(2)
    #c.send("motor_steps",0,0,-15000)
    #c.send("motor_run")
def midpoint(ptA, ptB):
	return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)
# load the image, convert it to grayscale, and blur it slightly

#path = r'C:\Users\Manoj PC\Desktop\Poject SCARA\CODE\SCARA_VISION-master\shapes_and_colors.png'
#image = cv2.imread(path)
#cv2.imshow("Image_original", image)
cam = cv2.VideoCapture(1) 
ret,image=cam.read()
blurred = cv2.GaussianBlur(image, (7, 7), 0)
#cv2.imshow("Image_blurred", blurred)
lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
#cv2.imshow("lab", lab)
gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
#cv2.imshow("Image_gray", gray)
# perform edge detection, then perform a dilation + erosion to
# close gaps in between object edges
edged = cv2.Canny(gray, 50, 100)
#cv2.imshow("Image_edged", edged)
edged = cv2.dilate(edged, None, iterations=1)
#cv2.imshow("Image_dilated", edged)
edged = cv2.erode(edged, None, iterations=1)
#cv2.imshow("Image_eroded", edged)
# find contours in the edge map
cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)
sd = ShapeDetector()
cl = ColorLabeler()
# sort the contours from left-to-right and, then initialize the
# distance colors and reference object
(cnts, _) = contours.sort_contours(cnts)

refObj = None
#initializing the array which will contain the midpoint co-ordinates of objects with measured w.r.t the referance image
objCoords=[]
# loop over the contours individually
orig = image.copy()
for c in cnts:
	# if the contour is not sufficiently large, ignore it
    if cv2.contourArea(c) < 50:
        continue
	# compute the rotated bounding box of the contour
    shape = sd.detect(c)
    color = cl.label(lab, c)
    box = cv2.minAreaRect(c)
    box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
    box = np.array(box, dtype="int")
	# order the points in the contour such that they appear
	# in top-left, top-right, bottom-right, and bottom-left
	# order, then draw the outline of the rotated bounding
	# box
    box = perspective.order_points(box)
	# compute the center of the bounding box
    cX = np.average(box[:, 0])
    cY = np.average(box[:, 1])
		# if this is the first contour we are examining (i.e.,
	# the left-most contour), we presume this is the
	# reference object
	#orig = image.copy()
    if refObj is None:
		# unpack the ordered bounding box, then compute the
		# midpoint between the top-left and top-right points,
		# followed by the midpoint between the top-right and
		# bottom-right
        rcX=cX
        rcY=cY
        X=box[0,0]
        Y=box[0,1]
        cv2.putText(orig, "Referance Object", (int(X), int(Y - 70)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        (tl, tr, br, bl) = box
        (tlblX, tlblY) = midpoint(tl, bl)
        (trbrX, trbrY) = midpoint(tr, br)
		
		
		#(tlblX, tlblY) = midpoint(tl, bl)
		#(trbrX, trbrY) = midpoint(tr, br)
		#cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
		#cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
		#cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
		#cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)
	# draw lines between the midpoints
		#cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
		#	(255, 0, 255), 2)
		#cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
		#	(255, 0, 255), 2)
		#dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
		#dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
		# compute the Euclidean distance between the midpoints,
		# then construct the reference object
        D = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
        refObj = (box, (cX, cY), D / width)
		
		
	# draw the contours on the image
	
    cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)
    cv2.drawContours(orig, [refObj[0].astype("int")], -1, (0, 255, 0), 2)
	#appending caliberated co-ordinate to objCoords
    objX=(cX-rcX)/refObj[2]
    objY=(rcY-cY)/refObj[2]
    objCoords.append((objX,objY))
	
    text = "{} {}".format(color, shape)
    cv2.circle(orig, (int(cX), int(cY)), 5, (0, 255, 0), -1)
    cv2.putText(orig, text, (int(cX), int(cY + 15)), cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (255, 200, 255), 2)
    cv2.putText(orig, "({:.1f},{:.1f})mm".format(objX,objY), (int(cX), int(cY - 50)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
	
    cv2.imshow("Image", orig)
    cv2.waitKey(0)
objCoords.append((-refObjX+homeX,-refObjY+homeY))    
print(objCoords)
moveInvKin(objCoords,refObjX,refObjY)
