from time import sleep
import cv2 as cv
import argparse
import sys
import numpy as np
import os.path
from glob import glob
import serial


serialPort = "COM4"
baudRate = 9600
ser = serial.Serial(serialPort , baudRate, timeout=0, writeTimeout=0) #ensure non-blocking

#from PIL import image
frame_count = 0             # used in mainloop  where we're extracting images., and then to drawPred( called by post process)
frame_count_out=0           # used in post process loop, to get the no of specified class value.
# Initialize the parameters
confThreshold = 0.5  #Confidence threshold
nmsThreshold = 0.4   #Non-maximum suppression threshold
confThresholdP = 0.5  #Confidence threshold
nmsThresholdP = 0.4   #Non-maximum suppression threshold
inpWidth = 416 #Width of network's input image
inpHeight = 416      #Height of network's input image


# Load names of classes

classesFileP = "coco.names";
classes = None
classesP=None

    
with open(classesFileP, 'rt') as f:
    classesP = f.read().rstrip('\n').split('\n')



modelConfigurationP = "yolov3.cfg";
modelWeightsP = "yolov3.weights";



netP = cv.dnn.readNetFromDarknet(modelConfigurationP, modelWeightsP)
netP.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
netP.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)


noseats=9

    
def postprocessP(frame, outs):
    frameHeight = frame.shape[0]
    frameWidth = frame.shape[1]
    global frame_count_outP
    global count_person
    frame_count_outP=0
    classIds = []
    confidences = []
    boxes = []
    # Scan through all the bounding boxes output from the network and keep only the
    # ones with high confidence scores. Assign the box's class label as the class with the highest score.
    classIds = []               #have to fins which class have hieghest confidence........=====>>><<<<=======
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThresholdP:
                center_x = int(detection[0] * frameWidth)
                center_y = int(detection[1] * frameHeight)
                width = int(detection[2] * frameWidth)
                height = int(detection[3] * frameHeight)
                left = int(center_x - width / 2)
                top = int(center_y - height / 2)
                classIds.append(classId)
                #print(classIds)
                confidences.append(float(confidence))
                boxes.append([left, top, width, height])

    # Perform non maximum suppression to eliminate redundant overlapping boxes with
    # lower confidences.
    indices = cv.dnn.NMSBoxes(boxes, confidences, confThresholdP, nmsThresholdP)
    
    count_person=0 # for counting the classes in this loop.
    for i in indices:
        i = i[0]
        box = boxes[i]
        left = box[0]
        top = box[1]
        width = box[2]
        height = box[3]
               #this function in  loop is calling drawPred so, try pushing one test counter in parameter , so it can calculate it.
        frame_count_outP = drawPredP(classIds[i], confidences[i], left, top, left + width, top + height)
         #increase test counter till the loop end then print...

        #checking class, if it is a person or not

        my_class='person'                   #======================================== mycode .....
        unknown_class = classesP[classId]

        if my_class == unknown_class:
            count_person += 1
    #if(frame_count_out > 0):
    print('Person'+str(count_person))  



def loadnetwork(label,weightspath,configpath):
    #label = label.read().strip().split("\n")
    net = cv.dnn.readNetFromDarknet(configpath, weightspath)
    return net

def setInput(image,net,ln):
    
        
    blob = cv.dnn.blobFromImage(image, 1 / 255.0, (121, 121),swapRB=True, crop=False)
    net.setInput(blob)
    
    layerOutputs = net.forward(ln)
    print('Layer outputs',layerOutputs)
    return layerOutputs

def detectBoxes(image,layerOutputs):
    (W, H) = (None, None)
    if( W is None or H is None):
        (H, W) = image.shape[:2]
    boxes = []
    confidences = []
    classIDs = []
    for output in layerOutputs:
        # loop over each of the detections
        
        for detection in output:
            # extract the class ID and confidence
            
            scores = detection[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]
            # filter out weak predictions by ensuring the detected
			# probability is greater than the minimum probability
            if confidence > 0.8:
                # scale the bounding box coordinates back relative to
				# the size of the image
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")
                
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))
                
                # update our list of bounding box coordinates,
				# confidences, and class IDs
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)
    # apply non-maxima suppression to suppress weak, overlapping
	# bounding boxes
    idexs = cv.dnn.NMSBoxes(boxes, confidences, 0.5,0.5)
    return idexs,boxes,classIDs                                              
                                             

cam = cv.VideoCapture(0)     

while True:
    ret,frame=cam.read()
    frame_count =0
    network=loadnetwork(classesFileP,modelWeightsP,modelConfigurationP)

    ln = network.getLayerNames()
    ln = [ln[i[0] - 1] for i in network.getUnconnectedOutLayers()]
    

    layerout=setInput(frame,network,ln)

    ids,boxes,classIDs=detectBoxes(frame,layerout)
#print(labels[classIDs])
    count_person=0
    print(len(ids))
    if( len(ids) > 0):
        for i in ids.flatten():
            if classesP[classIDs[i]]=="person":
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                color = (255,255,0)
        #print(classesFileP[classIDs[i]])
                cv.rectangle(frame, (x, y), (x + w, y + h), color, 2)
       #  text = "{}: {:.4f}".format(LABELS[classIDs[i]],
		#			confidences[i])
                print(classesP[classIDs[i]])
                count_person+=1
              
                cv.putText(frame, "person", (x, y - 5),cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
    print("person"+str(noseats-count_person))    
    
    c=ser.read()
    #print("from serial"+str(c))

    nohelmet='no.of persons'+str(count_person)
    #print(label)

  
    #print(label)
   
    count_helmet=0
    frame=cv.resize(frame,(1000,500))    
    cv.putText(frame, nohelmet, (0, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0))

    cv.imshow('img',frame)
    ser.write(str.encode(str(noseats-count_person)))
    count_person=0

    if cv.waitKey(30) & 0xFF == ord('q'):
        ser.write(str.encode(str(noseats-count_person)))
        count_person=0
        break
    
        
