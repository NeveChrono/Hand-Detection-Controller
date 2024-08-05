import cv2
import mediapipe as mp
import time


class handDetector():

    # Initialization
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands

        '''
        While giving the hands these parameters you need to mention the original parameters values used 
        as apparently the system gets confused if u used self.mode for instance js like that.
        All which can be found in if u press control over the function Hands        
        '''
        self.hands = self.mpHands.Hands(static_image_mode=self.mode, max_num_hands=self.maxHands,
                                        min_detection_confidence=self.detectionCon,
                                        min_tracking_confidence=self.trackCon)

        self.mpDraw = mp.solutions.drawing_utils

    # Detection of Hands
    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handlms in self.results.multi_hand_landmarks:
                if draw:
                    # Method to draw and plot those points using a default function and show it in our real-time image
                    # Drawing Connections using mpHands.HAND_CONNECTIONS
                    self.mpDraw.draw_landmarks(img, handlms, self.mpHands.HAND_CONNECTIONS)

        return img

    # Position of the landmarks in pixels

    def findPosition(self, img, handNo=0, draw=True):

        # To check if any lmList is detected or not

        lmList = []

        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                # To get the pixel value js multiply the coordinates with the width and height of the image
                h, w, c = img.shape  # Using this to get the values

                # Pixel values for each landmark

                # We can use a specific id number so we can work with some particular part of the hand
                # In this case it is the landmark no.0 ie the point on the edge of the palm will be filled
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmList.append([id, cx, cy])

        return lmList


def main():
    ptime = 0
    ctime = 0

    cap = cv2.VideoCapture(0)
    detector = handDetector()

    while True:
        success, img = cap.read()
        img = detector.findHands(img)
        lmList = detector.findPosition(img)

        if len(lmList) != 0:
            print(lmList[0])

        ctime = time.time()
        fps = 1 / (ctime - ptime)
        ptime = ctime

        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_DUPLEX, 3, (255, 0, 255), 3)
        cv2.imshow("Image", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()

'''
So now u can use this module for hand detection as all u have to do is js copy the code in the main()
import the libraries 
and also u can import the module in this case as:

import HandTrackingModule as htm

and while creating object js add the name ie htm.object_name()

'''
