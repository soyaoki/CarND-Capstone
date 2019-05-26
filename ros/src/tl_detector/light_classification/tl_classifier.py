#!/usr/bin/env python

from styx_msgs.msg import TrafficLight
from yolo import YOLO

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # Traffic light Detector -> YOLO
        self.detector = YOLO()
        
        # Light state classifier - > pretrained CNN
        #self.classifier = model.load()
        
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        # (1) Get ROI 
        image, tl_imgs = self.detector.detect_image(image)
        for i in range(0,len(tl_imgs)-1):
            print(i)
            # (2) Input CNN
        
        return TrafficLight.GREEN
