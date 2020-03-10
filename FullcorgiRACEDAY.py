from threading import Thread
import liboCams
import cv2
import time
import sys
import numpy as np
import time
import serial
from PID import *



allthreadbreak = False
activatelidar = False
Trafficlightsystem = False
Pinklinesystem = False
 
#### DONAT PARAMETER TF LIGHT & GOAL LINE####
greenstate =0
drawing = False
point1 = ()
point2 = ()
x1=0
y1=0
x2 =0
y2=0
startvelocity =int(0)
countpink = np.zeros(10)
numpink =np.zeros(10)
count =0
collectorcounter = 0
flag = 0
flaglidar = 0
ccounter = 0
timerecord = 0
finished=0
startcorgi=0
warpedforpink = []
maskpink = 0
############################################

## corgi Parameter #######

arduino = serial.Serial('/dev/ttyUSB0',115200)
laneposition = int(60)
motoraxis = int(90)
senddata = int(90)
velocity = int(0)
corgispeed = int(0)
carstate = False
pidoutmotorgoal = int(90)
markpidout = float(90)
############################################
#### IPP PARAMETER  LIDAR PROCESS############################
if activatelidar == True:
    arduinoLidar = serial.Serial('/dev/ttyACM0',115200)
lidarvalue =int(400) 
############################################################

#### GAW PARAMETER PID PROCESS ###############################
expectlaneposition = 60
corgilanegoal = 60
#pid = PID(0.07, 17, 0.01)
#pid = PID(0.70, 12.6, 0.001)
pid = PID(0.07,34,0.01255)
#pid = PID(1, 0, 0)
pid.SetPoint=90.0
pid.setSampleTime(0.01)
feedback = 90
gaw = 1
outputpid = 90

arrsetpoint = []
arroutput = []
arrlaneshift = []
############################################################


##

##

def lidar():                         #lidar define function
    global lidarvalue
    global allthreadbreak
    while True:
        
        try:
            raw = arduinoLidar.readline()
            lidarVal = int(raw)
        except:
            lidarVal = int(400)
            pass
        lidarvalue = int(lidarVal)
        #time.sleep(0.05) 
        #print(lidarVal,"lidarthread")
        if allthreadbreak == True:
            break


def initiallizeocam(printstat):# 0 not print 1 print
  global allthreadbreak
  devpath = liboCams.FindCamera('oCam')
  if devpath is None:
    print 'Plese connect to camera'
    allthreadbreak = True
    exit()


  test = liboCams.oCams(devpath, verbose=1)

  if printstat == 1:
    print 'Format List'
    fmtlist = test.GetFormatList()
    for fmt in fmtlist:
      print '\t', fmt
      
    print 'Control List'
    ctrlist = test.GetControlList()
    for key in ctrlist:
      print '\t', key, '\tID:', ctrlist[key]


  test.Close()
  test = liboCams.oCams(devpath, verbose=0)
  test.Set(fmtlist[1])
  ##Description Format list indicate by number
  test.Start()

  #example code for camera control
  val = test.GetControl(ctrlist['Exposure (Absolute)'])
  test.SetControl(ctrlist['Exposure (Absolute)'],370)#586#200
  return test 
      
def perspectivetransformautoformat(frame):
    xaxmiddlepixcel = int(frame.shape[1]/2)
    yaxmiddlepixcel = int(frame.shape[0]/2) + int(frame.shape[0]/4)
    spanding = int(xaxmiddlepixcel/2)
    pts = np.array( [
        [xaxmiddlepixcel-spanding,yaxmiddlepixcel], 
        [xaxmiddlepixcel+spanding,yaxmiddlepixcel],  
        [frame.shape[1],frame.shape[0]],
        [0,frame.shape[0]]
    ], np.int32)
    pts = pts.reshape((-1,1,2))
    points_A = np.float32([
        [xaxmiddlepixcel-spanding,yaxmiddlepixcel], 
        [xaxmiddlepixcel+spanding,yaxmiddlepixcel], 
        [0,frame.shape[0]], 
        [frame.shape[1],frame.shape[0]]])
    points_B = np.float32([[0,0], 
                           [120,0], 
                           [0,213], 
                           [120,213]])
    M = cv2.getPerspectiveTransform(points_A, points_B)
    warped = cv2.warpPerspective(frame, M, (120,213))#120,213
    cv2.polylines(frame, [pts], True, (255,0,0), 3)
    return warped,frame

def datatoarduino(servoVal,motorVal):
    try:
        servoVal = str(servoVal)
        motorVal = str(motorVal)
        servoVal = servoVal.encode('utf-8')
        arduino.write(servoVal)
        arduino.flush()
        arduino.write(b'?')
        arduino.flush()
    
        motorVal = motorVal.encode('utf-8')
        arduino.write(motorVal)
        arduino.flush()
        arduino.write(b'\n')
        arduino.flush()
    except:
		pass

def maskyellowline(frame):
    #parameter
    yellow_sat = 30
    s = 139
    v = 108
    error = 30
    error2 = 50
    error3 = 40
    #kernel = np.ones((5,5),np.float32)/25
    #frame = cv2.filter2D(frame,-1,kernel)
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    avgcalculatesat = np.array(hsv)
    #print(np.average(avgcalculatesat[1]),np.average(avgcalculatesat[2]))
    # define range of blue color in HSV
    lower_yellow = np.array([(yellow_sat-error),(s-error2),(v-error3)])
    upper_yellow = np.array([(yellow_sat+error),(s+error2),(v+error3)])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Bitwise-AND mask and original image
    #linedetect
    res = cv2.bitwise_and(frame,frame, mask= mask)
    bluryellowline = res
    kernel = np.ones((9,9),np.uint8)
    yellowdilation = cv2.dilate(bluryellowline,kernel,iterations = 1)
    #kernel = np.ones((15,15),np.float32)/25
    #bluryellowline = cv2.filter2D(res,-1,kernel)
    return res,yellowdilation


def maskwhiteline(image):
    kernel = np.ones((3,3),np.float32)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #image = cv2.filter2D(image,-1,kernel)
    #ret,thresh = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
    #thresh = cv2.adaptiveThreshold(image, 127, cv2.ADAPTIVE_THRESH_MEAN_C, 
    #                           cv2.THRESH_BINARY, 3, 5) 
    _, thresh = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    #ret,thresh = cv2.threshold(image, 130, 255, cv2.THRESH_BINARY)
    thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
    return thresh

def bloblane(image):
    xp = int(image.shape[1])
    yp1 = int(image.shape[0] * 0.75) 
    yp2 = int(image.shape[0] * 0.80) 
    crop = image[yp1:yp2,0:xp]
    crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    params = cv2.SimpleBlobDetector_Params()
    params.filterByInertia = False
    params.filterByConvexity = False
    params.filterByColor = False
    params.minArea = 15 #min pixcel filtered out
    detector = cv2.SimpleBlobDetector(params)
    keypoints = detector.detect(crop)
    blank = np.zeros((1,1)) 
    blobs = cv2.drawKeypoints(crop, keypoints, blank, (0,0,255),
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return blobs,keypoints

def pidcontroller(currentpos,goal):
	Pgain = 0.7
	currentpos = int(currentpos)
	goal = int(goal)
	error = (goal-currentpos) * Pgain
	output = currentpos + (error)
	return output



def mouse_drawing(event, x, y, flags, params):
    global point1, point2, drawing,x1,y1,x2,y2
    if event == cv2.EVENT_LBUTTONDOWN:
        if drawing is False:
            drawing = True
            point1 = (x, y)
            x1=int(point1[0])
            y1=int(point1[1])
        else:
             drawing = False
    elif event == cv2.EVENT_LBUTTONUP:
        if drawing is True :
            point2 = (x, y)
            x2=int(point2[0])
            y2=int(point2[1])
    return x1,y1,x2,y2


def trafficgreen(frame):

    kernel = np.ones((5,5),np.float32)/25
    #print(y1,y2,x1,x2)
    crop_frame = frame[y1:y2,x1:x2]  
    if point1 and point2:
        h =70
        s=196
        v=183
        errorh=20
        slo = 150
        shi = 255
        vlo = 150
        vhi = 255
        res = cv2.filter2D(frame,-1,kernel)
        hsv = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([h-errorh,slo,vlo])
        upper_green = np.array([h+errorh,shi,vhi])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        greennumber = np.sum(mask)/255
        if greennumber >= 2.0:
            greenstate = 1
        else:
            greenstate = 0
        return mask,greenstate
    else:
        return crop_frame, 0

def finishline(blurpink):
    xc = int(blurpink.shape[1]/2)
    x1 =int(blurpink.shape[1]*0.45)
    x2 =int(blurpink.shape[1]*0.55)
    yp1 = int(blurpink.shape[0] * 0.90) 
    yp2 = int(blurpink.shape[0] * 0.99)
    croppink = blurpink[yp1:yp2,x1:x2]
    hval = 9
    errorh = 9
    sval = 105
    vval = 82
    serror = 20
    verror = 20
    kernel = np.ones((5,5),np.float32)/25
    kernel1 = np.ones((5,5),np.uint8)
    hsv = cv2.cvtColor(croppink, cv2.COLOR_BGR2HSV)
    lower_pink = np.array([hval-errorh,(sval-serror),(vval-verror)])
    upper_pink = np.array([hval+errorh,(sval+serror),(vval+verror)])
    mask = cv2.inRange(hsv, lower_pink, upper_pink)
    mask = cv2.dilate(mask,kernel1,iterations = 1)
    res1 = cv2.bitwise_and(croppink,croppink, mask= mask)
    pinknumber = np.sum(mask)/255.0
    print('pinknumber',pinknumber)
    #if (pinknumber/228.0)*100.0 >= 10.0:
    if pinknumber >40:
        pink = 1
        print('pink',pink)
    else:
        pink = 0
        print('pink',pink)
    return mask, pink

def carmotoraxis(lanepositionf):
    shift = 13
    centerpixcel = float(corgilanegoal)#expectlaneposition#60
    minpixcel = float(0)
    maxpixcel = float(120)
    mostleftaxis = float(50-shift)
    mostrightaxis = float(130+shift)
    inipixcel = float(centerpixcel - (maxpixcel-centerpixcel))
    if (centerpixcel == 60):
        slope = float( float(mostleftaxis-mostrightaxis)/float(maxpixcel-minpixcel))
        yintercept = (-1)* float( float( slope*float(maxpixcel-minpixcel))  - mostleftaxis )
    elif(centerpixcel < 60):
        slope = float(mostleftaxis-mostrightaxis)/float(centerpixcel*2)
        yintercept = (-1)* float( float( (slope*(centerpixcel*2))  - mostleftaxis ))
    elif(centerpixcel > 60):
        slope = float(mostleftaxis-mostrightaxis)/float(maxpixcel-inipixcel)
        yintercept = float((-1) *  ( float(slope*(maxpixcel-0))-mostleftaxis))
    servoaxis = (slope*lanepositionf) + yintercept
    if (servoaxis > 130):
        servoaxis = 130
    elif (servoaxis<50):
        servoaxis = 50

    #print (slope,yintercept,servoaxis)
    return servoaxis


def outputtomotoraxis(pidoutput):
    centerpixcel = expectlaneposition
    motorcenterangle = 90
    motorgoal = pidoutput + motorcenterangle
    return motorgoal


def markpidcontroller(currentpos,goal):
	Pgain = 0.5
	currentpos = int(currentpos)
	goal = int(goal)
	error = (goal-currentpos) * Pgain
	output = currentpos + (error)
	return output
    
def carspeedcal(servoangle):
	#y
	minspeed = 160
	maxspeed = 170
	motorcenterangle = 90
	if servoangle < 50:
		servoangle = 50
	elif servoangle > 130:
		servoangle = 130
	#x
	angleerrormin = 0
	angleerrormax = 40
	error = abs(motorcenterangle - servoangle)
	slope = float((maxspeed - minspeed) / (angleerrormin-angleerrormax)) #-2.625
	carcalculationspeed =  (slope*error) + maxspeed
	return carcalculationspeed

def greenlightloop():
    global greenstate
    global startcorgi
    global allthreadbreak
    while True:
        if point1 and point2:
            cv2.rectangle(res, (x1,y1), (x2,y2), (255, 0, 0))
            #print(x1,x2,y1,y2)
            if x2 > x1 and y2 > y1:
                maskgreen,greenstate = trafficgreen(originalframe)
                cv2.imshow("maskgreen", maskgreen)
            print('greenstate',greenstate)
        if greenstate == 1:
            startcorgi = 1
            break
        else:
            pass
        if allthreadbreak == True:
            break

def finishlineloop():
    global pink
    global maskpink
    global warpedforpink
    global flag
    global collectorcounter
    global allthreadbreak
    global valof1
    global countpink
    global percentof1
    global finished
    global startcorgi
    while True:
        print ("pinkloop")
        maskpink, pink= finishline(warpedforpink)
        cv2.imshow('warpedforpink',warpedforpink)
        if allthreadbreak == True:
            break
        countpink[collectorcounter] = int(pink)
        #print('countpink',countpink)
        valof1 = 0
        for i in range(0,len(countpink)):
            if countpink[i] == 1:
                valof1 = valof1 +1
                #print('valof1',valof1)
            else:
                pass
        percentof1 = int(valof1 * 10)
        #print('percentof1',percentof1)
        if percentof1 <= 0 and flag == 0:
            flag = 1
            print('Can not DetectPink')
        elif flag == 1 and percentof1 >= 10:
            finished = finished+1
            print('DetectPink')
            flag = 0
        if(finished>=1):
            startcorgi = 0
            print('Finished')
    
        collectorcounter = collectorcounter + 1
        if collectorcounter == 10:
            collectorcounter = 0
        print("finifhcounter",finished)
        break
     


fpms = []
if activatelidar == True:
    tr1 = Thread(target = lidar,args = ())
    tr1.start()
if Trafficlightsystem == True:
    tr2 = Thread(target = greenlightloop,args = ())
    tr2.start()
else:
    startcorgi = 1
    

startallprogloop = 0
camera = initiallizeocam(1)
while(True):
    millfirst1 = time.time() * 1000
    print(lidarvalue,"lidar", corgilanegoal,"corgilanegoal")
    millfirst1 = time.time() * 1000
    readcamera = camera.GetFrame()
    frame = cv2.cvtColor(readcamera, cv2.COLOR_BAYER_GB2BGR)
    height, width = frame.shape[:2]
    res = cv2.resize(frame,(int(width/3),int(height/3)), interpolation = cv2.INTER_CUBIC)
    originalframe = res
    warped,frame = perspectivetransformautoformat(res)
    warpedforpink = warped
    whiteline = maskwhiteline(warped)
    res,bluryellowline = maskyellowline(warped)
    #combineline = cv2.addWeighted(bluryellowline,1,whiteline,0,1)
    kernell = np.ones((3,3),np.uint8)
    combineline = cv2.morphologyEx(whiteline, cv2.MORPH_OPEN, kernell)
    kernell = np.ones((3,3),np.uint8)
    combineline = cv2.morphologyEx(combineline, cv2.MORPH_OPEN, kernell)
    kernell = np.ones((3,3),np.uint8)
    combineline = cv2.morphologyEx(combineline, cv2.MORPH_OPEN, kernell)
    blobs,keypoints = bloblane(combineline)
    #blobs2,keypoints2 = bloblane(whiteline)
    if Pinklinesystem == True:
        finishlineloop()

 #   if startallprogloop == 0:
 #       tr3 = Thread(target = finishlineloop,args = ())
 #       tr3.start()
 #       startallprogloop = 1

    ###################Green Light Activate code Start#################

    ###################Green Light Activate code End#################


    ###################################PINK CODE START HERE##########################

######################################################################

    if activatelidar == True:
        if int(lidarvalue) <= 150 and int(flaglidar) == 0:
             print("I SEE")
             corgilanegoal = int(85)
             millfirst = time.time() * 1000
             flaglidar = 1
        
        if flaglidar == 1:
             timerecord = (time.time()*1000)-millfirst
             if timerecord > 3000:
                  corgilanegoal = int(60)  
                  flaglidar = 0        
        else:
             corgilanegoal = int(60)
######################### END Ultrasonic ##################s##########    
    if len(keypoints) <= 0:
        #no lane
        lockcond = 0
        pass
    else:
        blob_x = keypoints[0].pt[0] #i is the index of the blob you want to get the position
        blob_y = keypoints[0].pt[1]
        laneposition = blob_x
        motoraxis = carmotoraxis(laneposition)  
        lockcond = 1
#    if lockcond == 0:
#        print("loccond")
#        if len(keypoints2) <= 0:
#            
#            pass
#        else:
#            blob2_x = keypoints2[0].pt[0] #i is the index of the blob you want to get the position
#            blob2_y = keypoints2[0].pt[1]
#            laneposition = blob2_x
#            motoraxis = carmotoraxis(laneposition)     
    

#blobs2,keypoints2   
                                                                                                                                               
    #print(motoraxis)
    ############## PID CODE START HERE 	######################     
	pid.SetPoint = motoraxis
    pid.update(feedback)
    outputpid = pid.output
    feedback += (outputpid - (1/gaw))
    gaw+=1
    senddata = feedback                                                                                                  
    #########################################################  
    #################DATA COMMUNICATION ######################
    if startcorgi ==1:
        datatoarduino(int(senddata),int(carspeedcal(senddata)))
    elif startcorgi ==0:
        #pass
        datatoarduino(int(senddata),0)
    #datatoarduino(int(senddata),int(carspeedcal(senddata)))
    ###########################################################

    ################# IMSHOW ##################################
    cv2.imshow('warped',warped)
    cv2.imshow('blobs',blobs)
    cv2.imshow('whiteline',whiteline)
    cv2.imshow("combineline",combineline)
    #cv2.imshow("frame",frame)
    #cv2.imshow("maskpink",maskpink)
    cv2.imshow("originalframe", originalframe)
    cv2.namedWindow("originalframe")
    cv2.setMouseCallback("originalframe", mouse_drawing)
    ##########################################################
    if cv2.waitKey(1) & 0xFF == ord('q'):
        allthreadbreak = True
        break
    
    millseceond = time.time() * 1000
    print("",millseceond-millfirst1)
#    time.sleep(0.2)
#fpms = np.array(fpms)

datatoarduino(90,0)
savedata1 = np.array(arrsetpoint)
savedata2 = np.array(arroutput)
savedata3 = np.array(arrlaneshift)
np.savetxt("setpoint.csv",savedata1, delimiter=",")
np.savetxt("output.csv",savedata2, delimiter=",")
np.savetxt("laneshift.csv",savedata3, delimiter=",")
camera.Stop()  
cv2.destroyAllWindows()
camera.Close()
