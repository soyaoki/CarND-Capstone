#!/usr/bin/env python

from styx_msgs.msg import TrafficLight
from yolo import YOLO
from keras.models import load_model
import numpy as np
import os
import cv2
from timeit import default_timer as timer
PATH = os.getcwd()

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # Traffic light Detector -> YOLO
        self.detector = YOLO()
        
        # Light state classifier - > pretrained CNN
        self.classifier = load_model(PATH + '/light_classification/tl_classifier.h5',compile=False)
        print('Classifier was loaded correctly.')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        start = timer()
        # (1) Get ROI (Detect traffic lights)
        image, tl_imgs = self.detector.detect_image(image)
        if (tl_imgs):
            # For each tdetected lights
            for i in range(0,len(tl_imgs)-1):
                # (2) Input CNN
                img = np.asarray(tl_imgs[i])
                img = cv2.resize(img, (32,32))
                prd = np.argmax(self.classifier.predict(img.reshape([1,32,32,3])))
                print(prd)
        end = timer()
        print(end-start)
        return TrafficLight.GREEN
