import re
import PyCmdMessenger
import math
import time


arduino = PyCmdMessenger.ArduinoBoard("COM3",baud_rate=115200)

commands = [["motor_steps","fff"],
            ["motor_run",""],
			["motor_state_prep","f"],
			["motor_state_run","f"]
			]

# Initialize the messenger
c = PyCmdMessenger.CmdMessenger(arduino,commands)

Len1= 250#248  #length of link 1 in mm
Len2= 140#147#139  #length of link 2 in mm

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

#Initial Position
theta_S_prev=0
theta_E_prev=0
z_prev=0

while(1):

	#NextPosition(give input)

	XkinPos=float(input("Enter X coodinate (mm): "))
	YkinPos=float(input("Enter Y coodinate (mm): "))
	ZkinPos=float(input("Enter Z coodinate (mm): "))
	
	if (ZkinPos<0 or ZkinPos>320):
		print("\n ERROR! Z-axis(height) limit is from 0 mm to 320 mm\n")
		continue

	#Temp Variables

	Temp1= XkinPos*XkinPos+YkinPos*YkinPos
	Temp2= (Temp1- SumLenSqrd)/ProdOfLens


	lefthand=0
	if (abs(Temp2)<=1) :
		#Inverse Kinematics 
		if(XkinPos>0): #always gives right hand calculation
			XkinPos=-XkinPos
			lefthand=1
		'''if(XkinPos<0): #do right hand calculation(positive arc cos)
			XkinPos=-XkinPos
			righthand=0
		'''
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
			continue
		#motor control
		
		z=z_prev-ZkinPos
		theta_S_new=theta_S-theta_S_prev
		theta_E_new=theta_E-theta_E_prev

		print("theta_E",theta_E)
		print("theta_S",theta_S)
		print("Z_new",-z)
			
		steps_M0=theta_S_new*SPD_1 #positive is clockwise and negative is anti-clockwise 
		steps_M1=theta_E_new*SPD_2
		steps_M2=z*SPM
		
		c.send("motor_steps",steps_M0,steps_M1,steps_M2)
		msg= c.receive()
		print(msg)
		
		print("Moving")
		c.send("motor_run")
		msg = c.receive()
		print(msg)
		
		z_prev=ZkinPos
		theta_E_prev=theta_E
		theta_S_prev=theta_S	
		#-------------
	
	
	else :
		print("Co-ordinate is beyond reachable workspace")
