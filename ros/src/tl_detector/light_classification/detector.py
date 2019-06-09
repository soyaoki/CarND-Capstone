import cv2
import numpy as np

class detector(object):
    def __init__(self):
        print("created classic classifier)
        
    def detect_by_hsv_feature(rgb_image):

        # Resize input image
        rgb_image = rgb_image.resize((32,32))

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
        print(feature)
        label = np.argmax(feat)
        print(label)

        return label

    def detect_by_rgb_feature(rgb_image):        
        # Resize input image
        rgb_image = rgb_image.resize((32,32))
        
        ## Create and return a feature value and/or vector
        feature = []

        r_hist = np.histogram(rgb_image[5:10,14:19,0], bins=32, range=(0, 255))
        g_hist = np.histogram(rgb_image[15:18,12:17,1], bins=32, range=(0, 255))
        b_hist = np.histogram(rgb_image[25:28,12:17,2], bins=32, range=(0, 255))
    #    print(r_hist[0])

        bin_edges = r_hist[1]
        bin_centers = (bin_edges[1:]  + bin_edges[0:len(bin_edges)-1])/2

    #    fig = plt.figure(figsize=(12,3))
    #    plt.subplot(131)
    #    plt.bar(bin_centers, r_hist[0])
    #    plt.xlim(0, 255)
    #    plt.title('R Histogram')
    #    plt.subplot(132)
    #    plt.bar(bin_centers, g_hist[0])
    #    plt.xlim(0, 255)
    #    plt.title('G Histogram')
    #    plt.subplot(133)
    #    plt.bar(bin_centers, b_hist[0])
    #    plt.xlim(0, 255)
    #    plt.title('B Histogram')

        fullest_r = np.argmax(r_hist[0])
        fullest_g = np.argmax(g_hist[0])
        fullest_b = np.argmax(b_hist[0])
        feature = [fullest_r, fullest_g ,fullest_b]
        print(feature)
        label = np.argmax(feat)
        print(label)

        return label