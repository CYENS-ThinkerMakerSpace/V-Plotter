# This file is the backbone of the stepper motor movement. It has only functions that are called to perform a more abstract operation on the stepper motors. 

from smbus2 import SMBus 
from ticlib import TicI2C, SMBus2Backend 
from time import sleep 
import math 
from ticlib.ticlib import Settings 
from PIL import Image 
from numpy import * 
import os, sys, subprocess 
import cv2 
os.chdir('/home/pi/Desktop/FinalVPlotter_ver2') 

def setup2StepperMotors(startx, starty, D): 
    L1, L2 = lengths(startx, starty, D) 
    step1, step2 = steps(L1), steps(L2) 
    # Open a handle to "/dev/i2c-1", representing the I2C bus. 

    bus = SMBus(4)  # Represents /dev/i2c-4 

    # Select the I2C address of the Tic (the device number). 
    address_1 = 15 
    backend_1 = SMBus2Backend(bus, address_1) 
    tic1 = TicI2C(backend_1) 
    tic1Settings = Settings(tic1) 

    address_2 = 14 
    backend_2 = SMBus2Backend(bus, address_2) 
    tic2 = TicI2C(backend_2) 
    tic2Settings = Settings(tic2) 
    
    timeWaiting = 0.5 
    tic1.reset() 
    tic1.exit_safe_start() 
    tic2.reset() 
    tic2.exit_safe_start() 
    tic1.energize() 
    tic2.energize() 
    
    sleep(timeWaiting) 
    tic1.halt_and_set_position(step1) 
    sleep(timeWaiting) 
    tic2.halt_and_set_position(step2) 
    sleep(timeWaiting) 
    return tic2, tic1 

def deenergiseTwoMotors(motor1, motor2): 
    motor1.deenergize() 
    motor2.deenergize() 

def xPos(L1, L2, D): 
    return math.floor((L1*L1 - L2*L2 + D*D) / (2*D)) 

def yPos(L1, L2, D): 
    return math.floor(L1*L1 + ((L1*L1 - L2*L2 + D*D) / (2*D))**2) 

def lengths(x, y, D): 
    # l1 (left) and l2 (right) 
    return math.floor(math.sqrt(x**2 + y**2)), math.floor(math.sqrt((D - x)**2 + y**2)) 

def steps(l, DiameterOfGear=25, numberOfStepsInRevolution=800): 
    return math.floor((numberOfStepsInRevolution * l) / (math.pi * DiameterOfGear))  

def lengthNow(currentStepPosition, DiameterOfGear=2.5, numberOfStepsInRevolution=800): 
    return math.floor(currentStepPosition * math.pi * DiameterOfGear / numberOfStepsInRevolution) 

def mm_to_steps(mm): 
    steps_per_mm = 10000/1 # calibrate 
    return mm * steps_per_mm 

def pixels_to_mm(pixels): 
    mm_per_pixels = 1/10 # calibrate 
    return pixels * mm_per_pixels 

def balance_the_speeds(L1, L2, L1prev, L2prev, maxSpeed): 
    speed1, speed2 = maxSpeed, maxSpeed 
    if L1 != L1prev and L2 != L2prev: 
        speed2 = int(maxSpeed * (L1 - L1prev) / (L2 - L2prev)) 
        if speed2 > maxSpeed*1.5: 
            speed1 = int(maxSpeed * (L2 - L2prev) / (L1 - L1prev)) 
            speed2 = maxSpeed 

    # print(f'Speeds 1, 2: {speed1}, {speed2}') 
    return abs(speed1), abs(speed2) 


def moveToXY(x, y, L1prev, L2prev, D, motor1, motor2, x_start, y_start, max_X, min_X, max_Y, min_Y): 
    # The motor can move a full 360 degrees with 800 steps given to the set_targer_position(800). 
    DiameterOfGear = 25 # mm 
    maxSpeed = 2000000 #10000000 
    
    L1, L2 = lengths(x,y,D) 
    speed1, speed2 = balance_the_speeds(L1, L2, L1prev, L2prev, maxSpeed) 

    step1 = steps(L1,DiameterOfGear) 
    step2 = steps(L2,DiameterOfGear) 
    
    motor2.set_max_speed(speed2) 
    motor1.set_max_speed(speed1) 

    motor2.set_target_position(step1) # right motor # with - it turns lefty 
    motor1.set_target_position(step2) 

    while motor1.get_current_position() != motor1.get_target_position() or motor2.get_current_position() != motor2.get_target_position(): 
        if not isBounds(x, y, max_X, min_X, max_Y, min_Y): 
            print("Out of bounds position... Heading back to the origin!") 

        sleep(0.01) 
    
    return x, y, L1, L2 



def isBounds(x, y, max_X, min_X, max_Y, min_Y): 
    if (x >= min_X and x <= max_X) or (y >= min_Y and y <=  max_Y): 
        return True 
    else: 
        return False 


def max_min_normalization(value, max, min, targetMax, targetMin): 
    # return value 
    back = ((value - min) / (max - min)) * (targetMax - targetMin) + targetMin 
    if back < 0: 
        back = 0 
    return back 

def max_X_value(inputlist): 
    return max([sublist[0] for sublist in inputlist]) 

def max_Y_value(inputlist): 
    return max([sublist[1] for sublist in inputlist]) 

def min_X_value(inputlist): 
    return min([sublist[0] for sublist in inputlist]) 

def min_Y_value(inputlist): 
    return min([sublist[1] for sublist in inputlist]) 
    
def goHome(motor1, motor2, x_start, y_start, L1, L2, D, max_X, min_X, max_Y, min_Y): 
    moveToXY(x_start, y_start, L1, L2, D, motor1, motor2, x_start, y_start, max_X, min_X, max_Y, min_Y) 

def open_file(filename): 
    if sys.platform == 'win32': 
        os.startfile(filename) 
    else: 
        opener = "open" if sys.platform == "darwin" else "xdg-open" 
        subprocess.run([opener, filename]) 




################################################################################### 

def calculateTimeToDraw(linePath, avgSpeed): 
    print('Moved to the calculateTimeToDraw') 
    # avgPath in m/s 
    lengthOfEachStep = math.sqrt((linePath[0][0] - linePath[1][0])**2 + (linePath[0][1] - linePath[1][1])**2)/1000 
    print(f'The length of each step is: {lengthOfEachStep} m.') 
    duration = (lengthOfEachStep / avgSpeed) * len(linePath) * len(linePath[0]) 
    return round(duration/3600,2) # in minutes 


def flipLinesOfMatrix(path4DMatrix): 
    print('Moved to the flipLinesOfMatrix') 
    newMatrix = [] 
    newLine = [] 
    newPixel = [] 

    for i, line in enumerate(path4DMatrix): 
        for pixel in line: 
            if i % 2 != 0: 
                newPixel = pixel[::-1] 
            else: 
                newPixel = pixel 
            newLine.append(newPixel) 
        newPixel = [] 
        if i % 2 != 0: 
            newLine = newLine[::-1] 
        newMatrix.append(newLine) 
        newLine = [] 

    return newMatrix 




def convertToActualCanvasProportionsTheXYs(path4DMatrix, maxX, minX, maxY, minY): 
    print('Moved to the convertToActualCanvasProportionsTheXYs') 
    max_X = path4DMatrix[-1][-1][-1][0] 
    max_Y = path4DMatrix[-1][-1][-1][1] 
    for i, iV in enumerate(path4DMatrix): 
        for j, jV in enumerate(iV): 
            for z, zV in enumerate(jV): 
                path4DMatrix[i][j][z][0] = round(max_min_normalization(zV[0], max_X, 0, maxX, minX),3) 
                path4DMatrix[i][j][z][1] = round(max_min_normalization(zV[1], max_Y, 0, maxY, minY),3) 
                
    return path4DMatrix 



def skipUndrwingSteps(Steps): 
    print('Moved to the skipUndrwingSteps method') 
    newSteps = [Steps[0]] 
    for i, step in enumerate(Steps[0:]): 
        if step[2] == False: 
            if Steps[i+1][2] == True: 
                newSteps.append(step) 
        if step[2]: 
            newSteps.append(step) 
        if Steps[i-1][2]: 
            if step == False: 
                newSteps.append(step) 
    return newSteps 
            
def makeStepsForGrays(grayValue, x1, y1, x2, y2, h): 
    # print('Moved to the makeStepsForGrays method') 
    newSteps = [] 
    if grayValue == 0: 
        if x1 == 0 and y1 == 0: 
            newSteps = [[x2, y2, False]] 
            return newSteps 

    if grayValue == 1: 
        if x1 == 0 and y1 == 0: 
            newSteps = [[x2, y2, True]] 
            return newSteps 


    distance = math.sqrt((x2-x1)**2 + (y2-y1)**2) 
    a = distance / (grayValue + 1) 
    for i in range(0, grayValue): 
        newX = x1 + a * (i+1) 
        if i % 2 == 0: 
            newY = y1 + h/2 
        else: 
            newY = y1 - h/2 
            if newY <= 0: 
                newY = 0 
        
        newSteps.append([newX, newY, True]) 
    newSteps.append([x2, y2, True]) 
    return newSteps 

def averageColorWithinRectangles(img, numberOfHorizontalLines, numberOfVerticalLines): 
    print('Moved to the averageColorWithinRectangles method') 
    imgOnResolution = [[0 for i in range(numberOfVerticalLines)] for j in range(numberOfHorizontalLines)] 
    # kernel is the box that will move around the image averaging the colors, meassured in pixels 
    kernelY = round(len(img)/numberOfHorizontalLines) 
    kernelX = round(len(img[0])/numberOfVerticalLines) 
    if kernelX <= 0: 
        kernelX = 1 
    if kernelY <= 0: 
        kernelY = 1 

    i, j = 0, 0 
    for a in range(0,len(img)-kernelY,kernelY): 
        for b in range(0,len(img[0])-kernelX,kernelX):  
            if j < len(imgOnResolution) and i < len(imgOnResolution[0]): 
                avgColor = mean(img[a:a+kernelY,b:b+kernelX]) 
                if avgColor > 255: 
                    imgOnResolution[j][i] = 255 
                elif avgColor < 0: 
                    imgOnResolution[j][i] = 0 
                else: 
                    imgOnResolution[j][i] = avgColor 
            i += 1 
        i = 0 
        j += 1 
    
    return imgOnResolution 

def constructThePath(imgOnResolution, numberGrays): 
    print('Moved to the constructThePath method') 
    newMatrix = [] 
    newLine = [] 
    newPixel = [] 
    
    graySegmentsDistance = 255 / (numberGrays + 1) 
    print(f'Number of Gray Values: {numberGrays},  graySegmentsDistance: {graySegmentsDistance}.') 
    
    x, y = 0, 0 
    maxX_Before = 0 
    maxY_Before = 0 
    
    for line in imgOnResolution: 
        for pixel in line: # pixel is the average color of that area of the image 
            for i in range(0, numberGrays+1): # range(start, stop) 
                # This for loop will make only one newPixel. So there is no need to append. 
                if pixel >= i * graySegmentsDistance and pixel < (1+i) * graySegmentsDistance: 
                    newPixel = makeStepsForGrays(grayValue=grayValueFlipper(i,numberGrays) , x1=x, y1=y, x2=x+1, y2=y, h=1) 
                    
            newLine.append(newPixel) 
            newPixel = [] 
            x += 1 
            if (y == 0): 
                maxX_Before += 1 
        newMatrix.append(newLine) 
        newLine = [] 
        x = 0 
        y += 1 
        maxY_Before += 1 
        
    return newMatrix 

def grayValueFlipper(originalGrayValue, maxGrayValue): 
    newGrayValue = maxGrayValue - originalGrayValue 
    if newGrayValue > maxGrayValue: 
        return maxGrayValue 
    if newGrayValue < 0: 
        return 0 
    return maxGrayValue - originalGrayValue 


def makeMatrixIntoArray(path4DMatrix): 
    print('Moved to the makeMatrixIntoArray') 
    newArray = [] 
    for line in path4DMatrix: 
        for pixel in line: 
            for point in pixel: 
                newArray.append(point) 
    return newArray 

def makeImageIntoLines(maxX, minX, maxY, minY, resolutionPercentage, lineWidthInMM, imagePath, numberGrays): 
    # The return form will be a matrix of x,y,drawOrNot.  
    print('Moved to the makeImageIntoLines method') 
    widthOfDrawingAreaInMeters = (maxX - minX ) /1000 
    heightOfDrawingAreaInMeters = (maxY - minY ) /1000 

    numberOfVerticalLines = round(widthOfDrawingAreaInMeters * resolutionPercentage / (lineWidthInMM / 1000)) 
    numberOfHorizontalLines = round(heightOfDrawingAreaInMeters * resolutionPercentage / (lineWidthInMM / 1000)) 
    if numberOfVerticalLines <= 0: 
        numberOfVerticalLines = 1 
    if numberOfHorizontalLines <= 0: 
        numberOfHorizontalLines = 1 

    # Start at the top left corner with the marker not touching 
    lines = [ ]  

    # Read grayscale image 
    img = cv2.imread(imagePath, 0)  
    
    # Flip vertically the image  
    img = cv2.flip(img, 1) 
    
    print('0') 
    # Change the resolution 
    imgOnResolution = averageColorWithinRectangles(img, numberOfHorizontalLines, numberOfVerticalLines) 

    print('1') 
    # Create the path but the x and y are in spaces not actual position in meters 
    lines4D = constructThePath(imgOnResolution, numberGrays) 
    if not isMatrixGood(lines4D): 
        return None 
    
    print('3') 
    # Convert the x and y coordinates into actual dimentions that can be drawn 
    correctedLinesInMatrixForm = convertToActualCanvasProportionsTheXYs(lines4D, maxX=maxX, minX=minX, maxY=maxY, minY=minY) 
    if not isMatrixGood(correctedLinesInMatrixForm): 
        return None 
    
    print('4') 
    # Flip each other line of the matrix so that the gondola follows a continues path, 
    # so it will not go over lines with no reason 
    flippedLinesInMatrixForm = flipLinesOfMatrix(correctedLinesInMatrixForm) 
    if not isMatrixGood(flippedLinesInMatrixForm): 
        return None 
    
    print('5') 
    flippedCorrectedLinesInArrayForm = makeMatrixIntoArray(flippedLinesInMatrixForm) 

#     print('6') 
#     PathWithOutTheNoDrawSteps = skipUndrwingSteps(flippedCorrectedLinesInArrayForm) 
    
    print('Finished the path calcultions correctly.') 
    print(flippedCorrectedLinesInArrayForm) 
    return flippedCorrectedLinesInArrayForm 

def isMatrixGood(myMatrix): 
    for line in myMatrix: 
        if not isinstance(line, list): 
            print('Matrix is not correct, line') 
            return False 
        for pixel in line: 
            if not isinstance(pixel, list): 
                print('Matrix is not correct, pixel') 
                return False 
            for point in pixel: 
                if not isinstance(point, list): 
                    print('Matrix is not correct, point') 
                    print(f'The erroneus point is: {point}.') 
                    print(f'The erroneus pixel is: {pixel}.') 
                    print(f'The erroneus line is: {line}.') 
                    print(f'The erroneus myMatrix is: {myMatrix}.') 
                    return False 
    return True
