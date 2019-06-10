#!/usr/bin/env python

from styx_msgs.msg import TrafficLight
from yolo import YOLO
from keras.models import load_model
import numpy as np
import os
import cv2
from timeit import default_timer as timer
from PIL import Image, ImageFont, ImageDraw
PATH = os.getcwd()

class TLClassifier(object):
    def __init__(self):
        self.count = 0
        self.imgs_for_gif = []
        #TODO load classifier
        # Traffic light Detector -> YOLO
        self.detector = YOLO()
        
        # Light state classifier - > pretrained CNN
        self.classifier = load_model(PATH + '/light_classification/tl_classifier.h5',compile=False)
        print('Classifier was loaded correctly.')
        
    def detect_by_hsv_feature(self, rgb_image):

        # Convert image to HSV color space
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

        # Create and return a feature value and/or vector
        feature = []
        lower_S = np.array([0,0,0]) 
        upper_S = np.array([200,120,200])
        mask = cv2.inRange(hsv, lower_S, upper_S)
        masked_image = np.copy(hsv)
        masked_image[mask != 0] = [0, 0, 0]
        ss = masked_image[:,:,1]

        feature_r = np.sum(ss[0:11][:])
        feature_y = np.sum(ss[12:21][:])
        feature_b = np.sum(ss[22:32][:])
        feature = [feature_r, feature_y ,feature_b]
        
        label = np.argmax(feature)

        return label

    def detect_by_rgb_feature(self, rgb_image):        
        
        ## Create and return a feature value and/or vector
        feature = []

        r_hist = np.histogram(rgb_image[5:10,14:19,0], bins=32, range=(0, 255))
        g_hist = np.histogram(rgb_image[15:18,12:17,1], bins=32, range=(0, 255))
        b_hist = np.histogram(rgb_image[25:28,12:17,2], bins=32, range=(0, 255))

        bin_edges = r_hist[1]
        bin_centers = (bin_edges[1:]  + bin_edges[0:len(bin_edges)-1])/2

        fullest_r = np.argmax(r_hist[0])
        fullest_g = np.argmax(g_hist[0])
        fullest_b = np.argmax(b_hist[0])
        feature = [fullest_r, fullest_g ,fullest_b]
        
        label = np.argmax(feature)

        return label

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        # (1) Get ROI (Detect traffic lights)
        state_predicted = TrafficLight.UNKNOWN
        image, tl_imgs = self.detector.detect_image(image)
        draw = ImageDraw.Draw(image)
        
        #print(len(tl_imgs))
        if (len(tl_imgs)>0):
            # (2) Input CNN
            # For each tdetected lights
            states = []
            if (len(tl_imgs) == 1):
                img = np.asarray(tl_imgs[0]) # CV img
                img = cv2.resize(img, (32,32))
                #prd = np.argmax(self.classifier.predict(img.reshape([1,32,32,3]))) # CNN
                #prd_hsv = self.detect_by_hsv_feature(img) # HSV
                prd_rgb = self.detect_by_rgb_feature(img)
                
                prd = prd_rgb
                
            else :
                for i in range(0,len(tl_imgs)-1):
                    img = np.asarray(tl_imgs[i]) # CV img
                    img = cv2.resize(img, (32,32))
                    #prd = np.argmax(self.classifier.predict(img.reshape([1,32,32,3]))) # CNN
                    #prd_hsv = self.detect_by_hsv_feature(img) # HSV
                    prd_rgb = self.detect_by_rgb_feature(img) # RGB
                    
                    prd = prd_rgb
                    states.append(prd)
                count = np.bincount(states)
                prd = np.argmax(count)
                
            print(prd)
            if (prd == 2):
                state_predicted = TrafficLight.GREEN
            elif (prd == 1):
                state_predicted = TrafficLight.YELLOW
            elif (prd == 0):
                state_predicted = TrafficLight.RED
                
            ########### Save images ###########
            #if (len(tl_imgs)>0):
                #w, h = image.size
                #font = ImageFont.truetype("light_classification/yolo-config/font/FiraMono-Medium.otf", 32)
                #text = "CNN Predcit : " + str(prd)
                #draw.text((0,h-150),text, fill="#fff", font=font)
                #text = "HSV Predcit: " + str(prd_hsv)
                #draw.text((0,h-100),text, fill="#fff", font=font)
                #text = "RGB Predcit : " + str(prd_rgb)
                #draw.text((0,h-50),text, fill="#fff", font=font)
                #font = ImageFont.truetype("light_classification/yolo-config/font/FiraMono-Medium.otf", 16)
                #text = "0: Red, 1:Yellow, 2: Green"
                #draw.text((w-300,h-20),text, fill="#fff", font=font)
                #if self.count > 30:
                    #self.imgs_for_gif.append(image)
                    #print(len(self.imgs_for_gif))
                #elif self.count == 30:
                    #print(self.imgs_for_gif)
                    #image.save('out.gif', save_all=True, append_images=self.imgs_for_gif, duration=100, loop=0)
                    #print("Done GIF making.")
                #name = 'imgs/traffic-light-' + str(self.count) + '.jpg'
                #image.save(name, quality=80)
                #print("saved image.")
                #self.count = self.count+1
                #print(self.count)
                
        return state_predicted
