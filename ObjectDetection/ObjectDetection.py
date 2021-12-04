###############################################################################
# Object Detection of the Shopping Cart Car
# The program below consists of the following aspect:
# - Establish bluetooth connection with Flutter Mobile Application
# - Based on user input , Arrow Detection begins with the use of Dataset for both Vegetable and Meat
# - Establish Serial Connection with MSP432
# - Passing data (direction L:LEFT R:RIGHT) to determine the car direction to MSP


#################################################################################
# IMPORT FOR OPENCV
import cv2
import numpy as np
import os

# IMPORT FOR MSP CONNECTION
import serial

# IMPORT FOR BLUETOOTH CONNECTION (WITH FLUTTER MOBILE APPLICATION)
# btcomm API for sending and receiving data over Bluetooth.
from bluedot.btcomm import BluetoothServer
from signal import pause
import time

# ESTABLISH CONNECTION BETWEEN MSP AND RASPBERRY PI (SERIAL CONNECTION)
ser = serial.Serial(
    "/dev/ttyACM0",
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1,
)
# PROGRAM FOR ARROW DETECTION (VEGETABLE, MEAT SECTION)
def data_received(data):
    # print data received from the mobile application (USER INPUT FROM MOBILE APPLICATION)
    print(data)
    # Check if the data received is Veg
    if "Veg" in data:
        # Path to Vegetable DATASET
        path = "Dataset"
        # Initiate Oriented FAST and Rotated BRIEF (ORB) detector
        # nFeatures which denotes maximum number of features to be retained
        orb = cv2.ORB_create(nfeatures=1000)

        # Import Images (Vegetables)
        # IMAGE DATASET
        imagesDataSet = []
        # IMAGE NAME (SAME AS FILE NAME)
        classNames = []
        myList = os.listdir(path)
        print("Total Classes Detected", len(myList))
        for cl in myList:
            imgCur = cv2.imread(f"{path}/{cl}", 0)
            imagesDataSet.append(imgCur)
            classNames.append(os.path.splitext(cl)[0])

        def findDescriptor(imagesDataSet):
            # Create list of descriptors
            desList = []
            for img in imagesDataSet:
                # None for Mask
                keypoint, description = orb.detectAndCompute(img, None)
                desList.append(description)
            print(desList)
            return desList

        # send in the descriptor of the Dataset and the video stream
        # return the id with the most matches (above 15 matches )
        def findMatchID(img, desList, thres=15):
            # None for Mask
            keyboard2, description2 = orb.detectAndCompute(img, None)
            # create the BFMatcher object
            bf = cv2.BFMatcher()
            matchList = []
            finalVal = -1
            try:
                for des in desList:
                    # Brute Force Match ( take the videostream  and matches with the other descriptors in the Dataset, k is the no of values used to compare)
                    matches = bf.knnMatch(des, description2, k=2)
                    # list of good matches between the video stream and the dataset
                    goodmatch = []
                    for m, n in matches:
                        # low distance = good match
                        if m.distance < 0.70 * n.distance:
                            goodmatch.append([m])
                    matchList.append(len(goodmatch))
            except:
                pass
            if len(matchList) != 0:
                if max(matchList) > thres:
                    finalVal = matchList.index(max(matchList))
            return finalVal

        desList = findDescriptor(imagesDataSet)

        # print(len(desList))
        cap = cv2.VideoCapture(0)
        while True:
            success, img2 = cap.read()
            imgOriginal = img2.copy()
            # grayscale the video stream for feature detection and matching
            img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            # find the id that has the highest matches with the video stream shown
            id = findMatchID(img2, desList)
            if id != -1:
                # Showing L or R in the Video Stream based on the matches result
                cv2.putText(
                    imgOriginal,
                    classNames[id],
                    (50, 50),
                    cv2.FONT_HERSHEY_COMPLEX,
                    1,
                    (0, 0, 255),
                    2,
                )
                print(classNames[id])
                # send information of ('l' / 'r' send to the MSP for right and left direction)
                x = classNames[id]
                # SEND FROM SERIAL CONNECTION
                ser.write(bytes(x, "utf-8"))
            #  time.sleep(5)
            # display colored version of the Video Stream through OPENCV
            cv2.imshow("img2", imgOriginal)

            cv2.waitKey(1)
    # Check if the data received is Meat
    elif "Meat" in data:
        # Path to Meat DATASET
        path = "Dataset2"
        # Initiate Oriented FAST and Rotated BRIEF (ORB) detector
        # nFeatures which denotes maximum number of features to be retained
        orb = cv2.ORB_create(nfeatures=1000)

        ##Import Images (MEAT)
        imagesDataSet = []
        classNames = []
        myList = os.listdir(path)
        print("Total Classes Detected", len(myList))
        for cl in myList:
            imgCur = cv2.imread(f"{path}/{cl}", 0)
            imagesDataSet.append(imgCur)
            classNames.append(os.path.splitext(cl)[0])

        def findDescriptor(imagesDataSet):
            desList = []
            for img in imagesDataSet:
                keypoint, description = orb.detectAndCompute(img, None)
                desList.append(description)
            return desList

        def findMatchID(img, desList, thres=15):
            keypoint2, description2 = orb.detectAndCompute(img, None)
            bf = cv2.BFMatcher()
            matchList = []
            finalVal = -1
            try:
                for des in desList:
                    matches = bf.knnMatch(des, description2, k=2)
                    goodmatch = []
                    for m, n in matches:
                        # approximation of 0.75
                        if m.distance < 0.63 * n.distance:
                            # low distance = good match
                            goodmatch.append([m])
                    matchList.append(len(goodmatch))
            except:
                pass
            if len(matchList) != 0:
                if max(matchList) > thres:
                    finalVal = matchList.index(max(matchList))
            return finalVal

        desList = findDescriptor(imagesDataSet)

        # print(len(desList))
        cap = cv2.VideoCapture(0)
        while True:
            success, img2 = cap.read()
            imgOriginal = img2.copy()
            img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            id = findMatchID(img2, desList)
            if id != -1:
                cv2.putText(
                    imgOriginal,
                    classNames[id],
                    (50, 50),
                    cv2.FONT_HERSHEY_COMPLEX,
                    1,
                    (0, 0, 255),
                    2,
                )
                print(classNames[id])
                x = classNames[id]
                ser.write(bytes(x, "utf-8"))
                # time.sleep(3)
            cv2.imshow("img2", imgOriginal)
            cv2.waitKey(1)


# Establish Connection using
s = BluetoothServer(data_received)
