# This program runs automatically as the raspberry pi starts up. If there are no problems, it can be controlled through the joystick and toggle switch. 
 # The Toggle switch allows the user to change the mode of operation from USB to Manual. 
 # USB means that the raspberry is expecting a USB to be plugged in with a jpg with the name "image.jpg", if this is satisfied then the image will be drawn automaticaly. 
 # Manual means that the user can use the joystick to move the gondola, also by pressing the Joystick then the pen moves in or out. 
 import os, sys, subprocess 
 os.chdir('/home/pi/Desktop/FinalVPlotter_ver2') 
 from USB_Auto import USB_Class 
 #from Joystick_Manual import Joystick_Manual_Class 
 from FunctionsPool import * 
 import RPi.GPIO as GPIO            
 import time 
  
 import bluetooth 
  
  
  
 def connect(): 
     # Connect to the ESP32 that is in the gondola to controll the depth of the pen. 
     # Only needs to run once evry time the raspberry restarts. 
     try: 
         bd_addr = "AC:67:B2:36:2C:32" # the mac adress of he ESP32_Gondola 
         port = 1 
         sock=bluetooth.BluetoothSocket(bluetooth.RFCOMM) 
         sock.connect((bd_addr, port)) 
         print("Gondola Connected.") 
         sock.send("1") 
         time.sleep(5) 
         sock.send("0") 
         time.sleep(5) 
         return sock 
     except: 
         print("Gondola Not Connected.") 
         return None 
  
  
  
  
 # Initial Setup 
 GPIO.setwarnings(False) 
 GPIO.setmode(GPIO.BCM)           # BCM GPIO numbering not the pin numbering 
  
 # Read the toggle push button. On:USB_Auto , Off:Joystick 
 input_switch = 16                # Input pin, button to GPIO  
 GPIO.setup(input_switch, GPIO.IN, pull_up_down = GPIO.PUD_UP)# Set our input pin to be an input 
 switch_value = False             # False: Run the USB, True: Run the Manual 
  
 resolution = 0.3 # 0 is 0% and 1 is 100% 
 penWidthInMM = 2 # in mm 
 numberGrays = 5 # Takes values grater than 0. This makes the time of excecution O^3, so it must be as small as possible. 
 # Also, if numberGrays is 1 then there are two gray segments (white and black) 
  
 # Starting parameters of the drawing area. 
 d = 2800                         # mm, Distance between centers 
  
 # 4:3 Aspect ratio (width:1200 height:900) 
 max_X = 2000                     # mm 
 min_X = 800                      # mm 
 max_Y = 1600                     # mm 
 min_Y = 700                      # mm 
  
 xStart = min_X + 1               # mm 
 yStart = min_Y + 1               # mm 
  
 x_now = xStart                   # mm 
 y_now = yStart                   # mm 
 L1, L2 = lengths(xStart, yStart, d) # mm 
  
 stepper1, stepper2 = setup2StepperMotors(xStart, yStart, d) 
 output_LED = 26                  # LED output GPIO 26 
 soc = connect() 
 myUSB_Class = USB_Class(stepper1, stepper2, xStart, yStart, d, max_X, min_X, max_Y, min_Y, output_LED, soc, resolution, penWidthInMM, numberGrays)        # Initialize the USB_Class 
 # myJoystick_Manual_Class = Joystick_Manual_Class(stepper1, stepper2, xStart, yStart, d, max_X, min_X, max_Y, min_Y, output_LED, soc)    # Initialize the Joystick_Manual_Class 
  
  
 # Endless loop 
 running = True 
 while running: 
 # To stop the loop shutdown the raspberry, in terminal press ctrl+x. 
  
     switch_value = GPIO.input(input_switch) # Read the switch, High:True, Low:False 
  
     if switch_value: 
         print("USB") 
         myUSB_Class.detect_USB() 
         time.sleep(0.05) 
         # print("joystick") 
         # x_now, y_now, L1, L2 = myJoystick_Manual_Class.draw_manualy(x_now, y_now, L1, L2) 
     else: 
         print("Idle") 
         time.sleep(0.05) 
  
  
 GPIO.cleanup() 
 soc.close()
