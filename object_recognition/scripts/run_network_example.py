import numpy as np
import cv2
import os
from tensorflow.keras.models import load_model
import rospy  ## The ROS library for python
from image_converter import ImageConverter  ## import a class that can transform ros image messages to opencv images, and back
from sensor_msgs.msg import Image
import datetime


# Image input size
IMAGE_SIZE = 32

# Load the model
path = os.path.join(os.environ["HOME"], "network_model")
model_path = os.path.join(path, "model_classifier.h5")

model = load_model(model_path)

image_converter = ImageConverter()
rospy.init_node("run_network_example")


# Extract the bounding boxes, using a filtered HSV image
def extract_bounding_boxes(hsv_image):
    bounding_boxes = []
    contours, _ = cv2.findContours(hsv_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        [x, y, w, h] = cv2.boundingRect(contour)

        if w > 4 and h > 4: # Already filtering out very small rectangles
            bounding_boxes.append([x, y, x + w, y + h])

    return bounding_boxes

## This function receives the image as obtained from the camera topic.
## Do your image preprocessing here, such that both your line-tracker and the hsv-calibrator tool can use the same preprocessing function
## As preprocessing step, this function should implement a gaussian blur.    <----------------
def preprocess_image(image):
    return cv2.GaussianBlur(image, (11, 11), 4)  ## Return the preprocessed image here


## This function receives the hsv-filtered image
## Do your image postprocessing here, by first applying a few erosion steps to remove noise pixels, and then applying a few dilation steps. <------------
def postprocess_image(image):
    kernel = np.ones((2,2), np.uint8)
    erosion = cv2.erode(image, kernel, iterations = 2)
    dilation = cv2.dilate(erosion, kernel,iterations = 2)
    return dilation  ## Return the preprocessed image here

def update(image):
    original_image = image_converter.convert_to_opencv(image)

    # Use your preprocessing, you might need to create a separate function for it   
    preprocessed_image = preprocess_image(original_image) 

    # Use the calibrator to get the values for the filter
    low_filter = np.array([105, 50, 40])
    high_filter = np.array([160, 255, 255])

    # Convert preprocessed_image to hsv
    hsv_image = cv2.cvtColor(preprocessed_image, cv2.COLOR_BGR2HSV)
    hsv_image = cv2.inRange(hsv_image, low_filter, high_filter)

    # Perform the postprocessing, you might need to create a separate function for it
    postprocessed_image = postprocess_image(hsv_image)

    # Create bounding boxes on postprocessed image
    bounding_boxes = extract_bounding_boxes(postprocessed_image)

    input_data = []
    for bb in bounding_boxes:
        img = original_image[bb[1]: bb[3], bb[0] : bb[2]]
        img = cv2.resize(img, (32, 32))
        data = np.asarray(img, dtype=np.float32) / 255.0
        input_data.append(data)

    if len(input_data) > 0:
        input_data = np.array(input_data)

        predictions = model.predict(input_data)
        for p in predictions:
            index = np.where(p == np.amax(p))[0][0] + 1
            if index != 3:
                print("[", datetime.datetime.now(), "] ", index)

image_subscriber = rospy.Subscriber("/camera/uncompressed", Image, update, queue_size = 1)

rospy.spin()