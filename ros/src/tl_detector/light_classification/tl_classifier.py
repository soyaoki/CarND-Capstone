from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # Traffic light Detector -> YOLO
        
        # Light state classifier - > pretrained CNN
        
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
        
        # (2) Input CNN
        
        return TrafficLight.RED
