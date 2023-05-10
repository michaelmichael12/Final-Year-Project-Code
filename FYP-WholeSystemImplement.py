#SAME AS FINAL 5 BUT INTEGRATING BUTTONS

#CW = LEFT
#ACW = RIGHT
import numpy as np
import cv2
from array import *
import time
import RPi.GPIO as GPIO
from time import sleep
import random
import board
from digitalio import DigitalInOut
from adafruit_character_lcd.character_lcd import Character_LCD_Mono

#setup for stepper motor-------------------
DIR = 21 # Direction GPIO pin
STEP = 20 # Step GPIO pin
CW = 1 # clockwise rotation
ACW = 0 # anti clockwise rotation
SPR = 200 # steps per revolution (360/1.8 = 200)

GPIO.setmode(GPIO.BCM) # set to broat com memory 
GPIO.setup(DIR, GPIO.OUT) # sets direction pin to output
GPIO.setup(STEP, GPIO.OUT) # sets step pin to output
GPIO.output(DIR, CW) # sets the initial direction to clockwise
#------------------------------------------------------------

#setup for servo-----------------------------------
GPIO.setmode(GPIO.BCM) # set to broat com memory
GPIO.setup(2, GPIO.OUT)
servo = GPIO.PWM(2,50) #2 IS PIN, 50 IS 50hZ PULSE
servo.start(0)
#-------------------------------------------------
#setup for BUTTONS
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(14, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#-----------------------------------------------------------
#setup for LCD DISPLAY-------------------------
lcd_columns = 16
lcd_rows = 2
lcd_rs = DigitalInOut(board.D26)
lcd_en = DigitalInOut(board.D19)
lcd_d4 = DigitalInOut(board.D13)
lcd_d5 = DigitalInOut(board.D6)
lcd_d6 = DigitalInOut(board.D5)
lcd_d7 = DigitalInOut(board.D11)
# Initialise the LCD class
lcd = Character_LCD_Mono(
    lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows
)
#--------------------------------------------------------------------------
def movePlungerLeft(unit):#moves plunger from tray to tray
    print("PLUNGER LEFT")
    GPIO.setmode(GPIO.BCM) # set to broat com memory 
    GPIO.setup(DIR, GPIO.OUT) # sets direction pin to output
    GPIO.setup(STEP, GPIO.OUT) # sets step pin to output
    GPIO.output(DIR, CW) # sets the initial direction to clockwise

    step_count = SPR # first test set to one full rotation
    delay = 0.001 # 1 second / 200SPR = 0.005

    print("EXECUTINg",25*unit," rotations to the left")
    for x in range(int(unit*25*step_count)): # for each step(turn 1 into 22 after testing)
        GPIO.output(STEP, GPIO.HIGH) # toggles step pin high, moving motor
        sleep(delay) 
        GPIO.output(STEP, GPIO.LOW) # toggles step pin low, stopping motor
        sleep(delay) 
#=======================================================
#=======================================================
def movePlungerRight(unit):#moves plunger from tray to tray
    print("moving plunger right")
    GPIO.setmode(GPIO.BCM) # set to broat com memory 
    GPIO.setup(DIR, GPIO.OUT) # sets direction pin to output
    GPIO.setup(STEP, GPIO.OUT) # sets step pin to output
    
    GPIO.output(DIR, ACW) # sets  direction to anticlockwise

    step_count = SPR # first test set to one full rotation
    delay = 0.001 # 1 second / 200SPR = 0.005

    print("EXECUTING   ",25*unit," rotations to the right")
    for x in range(int(unit*25*step_count)): # for each step CHANGE 1 TO 22 WHEN DONE WITH TESTING
        GPIO.output(STEP, GPIO.HIGH) # toggles step pin high, moving motor
        sleep(delay) 
        GPIO.output(STEP, GPIO.LOW) # toggles step pin low, stopping motor
        sleep(delay) 
#==============================================================================
#=========================================================================
def movePlungerDown():
    print("MOVE PLUNGER DOWN")

    time.sleep(2)
#---------------------------------------------
    print("rotate to desired starting angle")
    angle = 20
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

#----------------------------------------------
    time.sleep(2)
#---------------------------------------------
    print("lower plunger")
    angle = 100
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    print("plunger grabs card")
    #-------------------------------------------
#==========================================================================
#========================================================================================
def movePlungerUp():
    print("MOVE PLUNGER UP")
    time.sleep(2)
    #---------------------------------------------
    angle = 20
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    print("plunger lifts card up")
#===========================================================================
#====================================================================
def dropCard():#assumes plunger is directly over desired tray, this function rotates plunger down and up, dropping card in the process
    print("DROP CARD")
    step_count = SPR # first test set to one full rotation
    delay = 0.001 # 1 second / 200SPR = 0.005
    GPIO.output(DIR, CW) # sets the initial direction to clockwise
    for x in range(1*step_count): # for each step
        GPIO.output(STEP, GPIO.HIGH) # toggles step pin high, moving motor
        sleep(delay) 
        GPIO.output(STEP, GPIO.LOW) # toggles step pin low, stopping motor
        sleep(delay)
    #-----------------------------------------------------------------
#---------------------------------------------
    time.sleep(2)
#---------------------------------------------
    print("rotate to lower plunger")
    angle = 45
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    print("plunger grabs card")
    #------------------------------------------------------------------------------------------
    #this function attempts to move plunger ever soslighly to it goes underr the tray lip, dropping card in between the vplunger being lowered and being raised
 #move stepper motor slighly so card goes under lip of card tray-----------
    GPIO.output(DIR, ACW) # sets the initial direction to clockwise

    #loop to spin stepper motor clockwise 1 rotation---------------------------
    #print("rotating 1 roation clockwise")
    for x in range(1*step_count): # for each step
        GPIO.output(STEP, GPIO.HIGH) # toggles step pin high, moving motor
        sleep(delay) 
        GPIO.output(STEP, GPIO.LOW) # toggles step pin low, stopping motor
        sleep(delay)
    #-----------------------------------------------------------------
        
    time.sleep(2)
    #---------------------------------------------
    angle = 0
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    print("plunger lifts card up")
    #-------------------------------------------


def movePlungerDown():
    print("MOVE PLUNGER DOWN")

    time.sleep(2)
#---------------------------------------------
    print("rotate to desired starting angle")
    angle = 0
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
#----------------------------------------------
    time.sleep(2)
#---------------------------------------------
    print("rotate to lower plunger")
    angle = 100
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    print("plunger grabs card")
#======================================================================================================
#===================================================================================================
def PickCardUp():#function used when camera is over thew tray
    print("PICK CARD UP")
    #initiate horizontal movement to position plunger over card
    GPIO.output(DIR, ACW) # sets the initial direction to clockwise
    step_count = SPR # first test set to one full rotation
    delay = 0.001 # 1 second / 200SPR = 0.005
    for x in range(14*step_count): # for each step
        GPIO.output(STEP, GPIO.HIGH) # toggles step pin high, moving motor
        sleep(delay) 
        GPIO.output(STEP, GPIO.LOW) # toggles step pin low, stopping motor
        sleep(delay) 
    print("moving plunger down")
    time.sleep(2)
#---------------------------------------------
    print("rotate to desired starting angle")
    angle = 0
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    print("setup rotation finished")
#----------------------------------------------
    time.sleep(2)
#---------------------------------------------
    print("rotate to lower plunger")
    angle = 100
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    time.sleep(2)
    #print("rotate to lift plunger")
    angle = 0
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    print("plunger lifts card up")

#==========================================================================
#========================================================================================
def PutCardDown():#assumes that plunger has grabbed card & camera is over tray DONT THINK THIS IS NEEDED
    #initiate horizontal movement to position plunger over card--------------
    GPIO.output(DIR, CW) # sets the initial direction to clockwise
    step_count = SPR # first test set to one full rotation
    delay = 0.005 # 1 second / 200SPR = 0.005
    for x in range(14*step_count): # for each step
        GPIO.output(STEP, GPIO.HIGH) # toggles step pin high, moving motor
        sleep(delay) 
        GPIO.output(STEP, GPIO.LOW) # toggles step pin low, stopping motor
        sleep(delay)
#---------------------------------------------------------------
    print("moving plunger down")
    time.sleep(2)
#---------------------------------------------
    print("rotate to desired starting angle")
    angle = 20
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    print("setup rotation finished")
#----------------------------------------------
    time.sleep(2)
#---------------------------------------------
    print("rotate to lower plunger")
    angle = 50
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    time.sleep(2)
    #move stepper motor slighly so card goes under lip of card tray-----------
    GPIO.output(DIR, CW) # sets the initial direction to clockwise
    step_count = SPR # first test set to one full rotation
    delay = 0.005 # 1 second / 200SPR = 0.005
    #loop to spin stepper motor clockwise 1 rotation---------------------------
    #print("rotating 1 roation clockwise")
    for x in range(1*step_count): # for each step
        GPIO.output(STEP, GPIO.HIGH) # toggles step pin high, moving motor
        sleep(delay) 
        GPIO.output(STEP, GPIO.LOW) # toggles step pin low, stopping motor
        sleep(delay) 
    #---------------------------------------------------------------------
    print("rotate to lift plunger, dropping card in the process")
    angle = 20
    servo.ChangeDutyCycle(2+(angle/18))
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)
    print("plunger lifted and without card")

#===========================================================================
#====================================================================
def SuitSort():
            #CODE TO SORT BY SUIT
        print("suit sort function activated")
        #---------------------------------------------------------------------------------
        if 'H' in BestMatchedCard:
            print("card is a heart, moving card from card slot 0 to card slot 1")
            PickCardUp()
            movePlungerLeft(1)#0 to 1
            dropCard()
            movePlungerRight(0.5)#camera over tray 0 
            #----------------------------------------------------------------------------
        elif 'D' in BestMatchedCard:
            print("card is a diamond, moving card from card slot 0 to card slot 2")
            PickCardUp()
            movePlungerLeft(2)
            dropCard()
            movePlungerRight(1.5)#camrea over tray 0
        #------------------------------------------------------------------------
        elif 'S' in BestMatchedCard:
            print("card is a spade, moving card from card slot 0 to card slot 3")
            PickCardUp()
            movePlungerLeft(3)
            dropCard()
            movePlungerRight(2.5)
        #---------------------------------------------------------------------
        elif 'C' in BestMatchedCard:
            print("card is a club, moving card from card slot 0 to card slot 4")
            PickCardUp()
            movePlungerLeft(4)
            dropCard()
            movePlungerRight(3.5)
            #------------------------------------------------------------------
        else:
            print("card not defined correctly")
            #---------------------------------------------------------------- 
        print("card moved to its tray")

#======================================================================================
desiredNum = 1
numCardsLeft = 52
#=================================================================================
def NumSort(desiredNum):#worth changing number of stepper motor roations into an argument rather than for loops
        print("number sort function activated")
        print("number the selection sort algorithm is looking for is ", desiredNum)
        if len(str(desiredNum)) + 1 != len(BestMatchedCard):#scenario for 12 (queen) has 1 in it, but should sstill be wrong
            desiredNum = desiredNum + 50 #make it wrong but ensuring it doesnt change from odd to even or visa versa
        if (desiredNum % 2) == 1:#if even
            if str(desiredNum) in BestMatchedCard:
                print("1 detected, moving card to tray 4")
                #move card in place-----------------------
                PickCardUp()
                movePlungerLeft(4)
                dropCard()
                movePlungerRight(3.5)
                #----------------------------------------
                #----------------------------------------------------------------------------
            elif str(desiredNum + 1) in BestMatchedCard:
                PickCardUp()
                movePlungerLeft(3)
                dropCard()
                movePlungerRight(2.5)
            #---------------------------------------------------------------
            elif str(desiredNum + 2) in BestMatchedCard:
                PickCardUp()
                movePlungerLeft(2)
                dropCard()
                movePlungerRight(1.5)
            #-------------------------------------------------------------
            else:
                print("not a card we want, moving it to tray 1")
                PickCardUp()
                movePlungerLeft(1)
                dropCard()
                movePlungerRight(0.5)
        elif (desiredNum % 2) == 0:#if odd, means that deck is in traY 1
            if str(desiredNum) in BestMatchedCard:
                print("1 detected, moving card to tray 4")
                #move card in place-----------------------
                PickCardUp()
                movePlungerLeft(3)
                dropCard()
                movePlungerRight(2.5)
                #----------------------------------------
                #----------------------------------------------------------------------------
            elif str(desiredNum + 1) in BestMatchedCard:
                PickCardUp()
                movePlungerLeft(2)
                dropCard()
                movePlungerRight(1.5)
            #---------------------------------------------------------------
            elif str(desiredNum + 2) in BestMatchedCard:
                PickCardUp()
                movePlungerLeft(1)
                dropCard()
                movePlungerRight(0.5)
            #-------------------------------------------------------------
            else:
                print("not a card we want, moving it to tray 0")
                PickCardUp()
                movePlungerRight(1)
                dropCard()
                movePlungerLeft(1.5)
                #----------------------------------------------------------------  
#===============================================================================================
#=================================================================================================
def NumSortCycleCleanup():
    #moving cards from tray 3 to 4
    #assuming holder is currently at tray 0 
    # move to 3   
    movePlungerLeft(3)#0 to 3
    for x in range(4):#repeat 4 times for each card
        PickCardUp()
        movePlungerLeft(1)#3 to 4
        dropCard()
        movePlungerRight(0.5)#4 to 3
    #now need to move cards from tray 2 to 4
    movePlungerRight(1)#3 to 2
    for x in range(4):
        PickCardUp()
        movePlungerLeft(2)
        dropCard()
        movePlungerRight(1.5)
    #move plunger back to 0 now that cleanup is complete
    movePlungerLeft(2)
#==================================================================================
#==========================================================================================
def randomSort():
    oneCounter = 0
    twoCounter = 0
    threeCounter = 0
    fourCounter = 0
    print("random sort function activated")
    for x in range(52):
        print("card Count ",x)
        randomNum = random.randint(1,4)#generate a number 1,2,3 or 4. number determines what tray the card goes into
        print("random number generated is",randomNum)
        if randomNum == 1:
            oneCounter = oneCounter + 1
            print("moving card to tray 1")
        elif randomNum == 2:
            twoCounter = twoCounter + 1
            print("moving card to tray 2")
        elif randomNum == 3:
            threeCounter = threeCounter +1
            print("moving card to tray 3")
        elif randomNum == 4:
            fourCounter = fourCounter + 1
            print("moving card to tray 4")
        else:
            print("error: random number generator has not worked properly")
            stream.release()
            cv2.destroyAllWindows()
        PickCardUp()
        movePlungerLeft(randomNum)
        dropCard()
        movePlungerRight(float(randomNum)-0.5)
    #execute cleanup so all cards endup in tray 4
    #camera should be at position 0
    #cleaning up tray 1
    print("randomising complete, now time for cleanup")
    movePlungerLeft(1)

    for x in range(oneCounter):
        print(x, " cleaning traY 1 ")
        movePlungerDown()
        movePlungerUp()
        movePlungerLeft(3)#1 to 4
        dropCard()
        movePlungerRight(3)
    #plunger finishes over 1
    #now cleanup tray 2
    movePlungerLeft(1)
    for x in range(twoCounter):
        print(x, " cleaning traY 2 ")
        movePlungerDown()
        movePlungerUp()
        movePlungerLeft(2)#2 to 4
        dropCard()
        movePlungerRight(2)
    #plunger finishes over 2
    #now cleanup tray 3
    movePlungerLeft(1)
    for x in range(twoCounter):
        print(x, " cleaning traY 3 ")
        movePlungerDown()
        movePlungerUp()
        movePlungerLeft(1)#3 to 4
        dropCard()
        movePlungerRight(1)
    print("finished randomasing deck of cards")
#================================================================================================
#======================================================================================
#START OF MAIN CODE!!!!!!
stream = cv2.VideoCapture(0)#opens video on pi camera
#------------------------------------------------------------------------------------------
if not stream.isOpened():
    print("no stream")
    exit()
#-------------------------------------------------------------------------------
number = array('i' , [1,2,3,4,5,6,7,8,9,10,11,12,13])
Redsuit = array('u', ['H', 'D'])
Blacksuit = array('u', ['S', 'C'])

numCount = 0#start counters at 0, first card it will compare image to is ace of hearts
suitCount = 0
numOfCards = 0
desiredNum = 1

lowerRed = np.array([141, 87, 90])
upperRed = np.array([180, 255, 255])
lowerBlack = np.array([0, 0, 0])
upperBlack = np.array([255, 255, 70])
highestNumOfMatches = -1

#-------------------------------------------------------------------------
suitSortBool = False
NumSortBool = False
print("choose mode by pressing one of the buttons!")
lcd.message = 'CHOOSE MODE'
while True: 
    rand_input_state = GPIO.input(18)
    suit_input_state = GPIO.input(15)
    num_input_state = GPIO.input(14)
    if rand_input_state == False:
        print("Random sort Selected")
        lcd.message = 'RANDOM SORT'
        randomSort()
        time.sleep(0.2)
        break
    elif suit_input_state == False:
        print("suit sort selected")
        lcd.message = 'SUIT SORT'
        suitSortBool = True#change to true to activate suit sort function when needed
        break
    elif num_input_state == False:
        print("number sort selected")
        lcd.message = 'NUMBER SORT'
        NumSortBool = True
        break
#-------------------------------------------------------------------------------------
while(True):
     ret, frame = stream.read()# read stream
     #---------------------------------
     if not ret:
        print("no more stream")
        break
     #---------------------------------

     cycle = False
     image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
     greyimage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
     Rmask = cv2.inRange(image, lowerRed, upperRed)
     Bmask = cv2.inRange(image, lowerBlack, upperBlack)
     redArea = 0
     blackArea = 0
     #time.sleep(5)
     contours, hierarchy = cv2.findContours(Rmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#create contour for red colour
     for i, contour in enumerate(contours):
        redArea = cv2.contourArea(contour)

     contours, hierarchy = cv2.findContours(Bmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#create contour for black colour
     for i, contour in enumerate(contours):
        blackArea = cv2.contourArea(contour)
     cv2.imshow("PiCamera", frame)#pi camera video is shown
     cv2.imshow("Red Mask", Rmask)
     cv2.imshow("black Mask", Bmask)
     if (redArea or blackArea > 100):#if there is a card
        numOfCards = numOfCards + 1
        print("checking for best match...")
        while(cycle == False):
            if redArea > blackArea:
                cv2.imwrite('picToCompare.jpg', Rmask)#saves picture taken to a jpg file, this is contantly ovedrriden when a new picture is taken
                card = str(number[numCount]) + Redsuit[suitCount]
            elif blackArea > redArea:
                cv2.imwrite('picToCompare.jpg', Bmask)
                card = str(number[numCount]) + Blacksuit[suitCount]
            else:
                print("CANNOT DETECT IF CARD IS RED OR BLACK")
                break   

            img1 = cv2.imread('PcardsBandW/' + card + '.jpg')
            img2 = cv2.imread('picToCompare.jpg')

            #---------------------------------------------------------------------------------------
            orb =cv2.ORB_create(1000)#nfeaures = 1000, make it find a min amount of features
            kp1, des1 = orb.detectAndCompute(img1, None)
            kp2, des2 = orb.detectAndCompute(img2, None)

            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True) #brute force matcher
          
            matches = bf.match(des1, des2)
            matches = sorted(matches, key = lambda X:X.distance)
            #-------------------------------------------------
            
            if (len(matches) > highestNumOfMatches):
                highestNumOfMatches = len(matches) #setting new targer to beat
                BestMatchedCard = card
                BestMatchedImage = cv2.imread('PcardsBandW/' + card + '.jpg')
            
            numCount = numCount + 1  
            if  numCount == 13 and suitCount == 0:#if all numbers in suit have been compared, move to next suit
                suitCount = suitCount + 1#move to next suit array element
                numCount = 0#set number array back to 0 now that all numbers in suit have been checked
            elif numCount == 13 and suitCount == 1:#gone through all cards, now time to reveal which card had most amount of matches
                print("card with most matches is", BestMatchedCard, " with ", highestNumOfMatches, " matches") 
                #reset
            
                numCount = 0
                suitCount = 0
                highestNumOfMatches = 0
                cycle = True
        matchingResult = cv2.drawMatches( BestMatchedImage, kp1, img2, kp2, matches[:200], None, flags = 2)

        if suitSortBool == True:
            SuitSort()
        elif NumSortBool == True:
            NumSort(desiredNum)

        stream.release()#restert camera so it sees new card
        time.sleep(5)
        stream = cv2.VideoCapture(0)
        
     else:
        print("no card left in deck")
        print("there were", 52 - numOfCards, "missing")
        lcd.message =  str(52 - numOfCards) ,' MISSING'
        if NumSortBool == True:
            NumSortCycleCleanup()
            desiredNum = desiredNum + 3
  
        stream.release()
        cv2.destroyAllWindows()
     if cv2.waitKey(1) == ord('a'):#by pressing a, stream ends
        print("stream ended, goodbye")
        break
stream.release()
cv2.destroyAllWindows()
