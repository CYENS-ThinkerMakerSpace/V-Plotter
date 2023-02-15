from FunctionsPool import * 
import RPi.GPIO as GPIO 
import usb.core 
import usb.util 
import time 
import os 

os.chdir('/home/pi/Desktop/FinalVPlotter_ver2') 

class USB_Class: 
    DELAY_TO_MOVE_THE_PEN_SECONDS = 0.6 
    DELAY_TO_MOVE_THE_GONDOLA_SECONDS = 0.3 
    
    def __init__(self, stepper1, stepper2, xStart, yStart, d, max_X, min_X, max_Y, min_Y, output_LED, soc, resolution, penWidthInMM, numberGrays): 
        self.stepper1 = stepper1 
        self.stepper2 = stepper2 
        self.xStart = xStart 
        self.yStart = yStart 
        self.d = d 
        self.max_X = max_X 
        self.min_X = min_X 
        self.max_Y = max_Y 
        self.min_Y = min_Y 
        self.output_LED = output_LED 
        self.soc = soc # to controll the esp32 
        self.resolution = resolution 
        self.penWidthInMM = penWidthInMM 
        self.numberGrays = numberGrays 
        print(f'resolution-{self.resolution}') 

        GPIO.setwarnings(False) 
        GPIO.setmode(GPIO.BCM)   # Use physical pin numbering 
        GPIO.setup(self.output_LED, GPIO.OUT, initial = GPIO.LOW) # LED 


    def detect_USB(self): 
        # Blink an LED indicating that it's waiting for the user to connect a USB to the raspberry 
        GPIO.output(self.output_LED, GPIO.HIGH) 
        dev = None   
        #dev = usb.core.find(idVendor=0x0951, idProduct=0x1643) 
        dev = usb.core.find(idVendor=0x18A5, idProduct=0x0251)  
        print('Waiting for USB.') 
    
        time.sleep(1) 
        GPIO.output(self.output_LED, GPIO.LOW) 
        if dev is not None: 
            self.draw_the_image_from_USB() 


    def draw_the_image_from_USB(self): 
        print('Moved to the draw the image from usb method') 
        time.sleep(3) # wait for the usb name to appear 
        # Gather the image from the inserted USB 
        name_of_USB = os.listdir("/media/pi")[0] 
        image_path = f"/media/pi/{name_of_USB}/image.jpg" 
        GPIO.output(self.output_LED, GPIO.HIGH) 

        try: 
            x, y = self.xStart, self.yStart 
            L1, L2 = lengths(x, y, self.d) 
            
            # Steps = translateImageIntoXY(self.max_X, self.min_X, self.max_Y, self.min_Y, x=x, y=y, hatch_size=16, contour_simplify=1, myPath=image_path, resolution=self.resolution)       
            Steps = makeImageIntoLines(maxX=self.max_X, minX=self.min_X, maxY=self.max_Y, minY=self.min_Y,  
            resolutionPercentage = self.resolution, lineWidthInMM=self.penWidthInMM, imagePath=image_path, numberGrays=self.numberGrays) 

            print(f'Estimation: Time duration for completion: {calculateTimeToDraw(Steps, avgSpeed = 0.025/1.2)} hours.') 
#             prevI2 = False 
            
            for i in Steps: 
                if self.soc: 
                    # Wait for the last movement to stop 
                    time.sleep(self.DELAY_TO_MOVE_THE_GONDOLA_SECONDS) 
                    
                    if i[2]: 
                        # put the pen on the drawing surface 
                        self.soc.send("1") 
                        # Wait for the bluetooth message, translation, and excecution 
                        # of the movement to finish and then move to the next step 
                        time.sleep(self.DELAY_TO_MOVE_THE_PEN_SECONDS) 
                        print("Pen Down.") 
                    else:  
                        # lift the pen off of the drawin surface 
                        self.soc.send("0") 
                        time.sleep(self.DELAY_TO_MOVE_THE_PEN_SECONDS) 
                        print("Pen Up.") 

#                     if (i[2] != prevI2): 
#                         # Small delay for the pen to move 
#                         time.sleep(0.15) 
#                 prevI2 = i[2] 

                x, y, L1, L2 = moveToXY(i[0], i[1], L1, L2, self.d, self.stepper1, self.stepper2, self.xStart, self.yStart, self.max_X, self.min_X, self.max_Y, self.min_Y) 
                
#                 print(f"Drawnig progress step-{i}/{len(Steps[0])}") 


        except Exception as ex: 
            print(ex) 
            GPIO.output(self.output_LED, GPIO.LOW) 
            #goHome(self.stepper1, self.stepper2, self.xStart, self.yStart, L1, L2, self.d, self.max_X, self.min_X, self.max_Y, self.min_Y) 

            print("Error Alex") 
            time.sleep(2) 
            #deenergiseTwoMotors(self.stepper1, self.stepper2) 
            f = open('/home/pi/Desktop/FinalVPlotter_ver2/errorLog.txt','w') 
            f.write(str(ex)) 
            f.close() 

        finally: 
            GPIO.output(self.output_LED, GPIO.LOW) 
            time.sleep(2)
