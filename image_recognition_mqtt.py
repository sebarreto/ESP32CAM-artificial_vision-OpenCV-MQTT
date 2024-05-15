import cv2
import numpy as np
import paho.mqtt.client as mqtt
import time

# Adafruit MQTT Config
adafruit_username = "AAAAAAAAAAAAA"
adafruit_key = "XXXXXXXXXXXXXXXXXXXXXXXXXXX"
mqtt_topic = "AAAAAAAAAAAAA/feeds/YYYYYYYYYYYYYYYYY"

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

# Load the YOLOv3 model and its configuration and weights files
net = cv2.dnn.readNetFromDarknet('yolov3.cfg', 'yolov3.weights')

# Load the class labels file
classes = []
with open('coco.names', 'r') as f:
    classes = [line.strip() for line in f.readlines()]

# Set the input and output layers of the network
output_layers = net.getUnconnectedOutLayersNames()
input_layer = net.getLayerNames()[0]

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
# Initialize MQTT client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.username_pw_set(adafruit_username, adafruit_key)
client.connect("io.adafruit.com", 1883, 60)

# Initialize variables for person count
person_count = 0
previous_count = 0

# Initialize time variables for MQTT publishing interval
start_time = time.time()
publish_interval = 30  # Publish every 30 seconds

# Initialize the video capture object
cap = cv2.VideoCapture(0)  # Use 0 for webcam, or introduce IP address coming from ESP32CAM executed in first script from Arduino IDE.

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        break  # If video capture fails, exit the loop

    # Resize the frame to the input size of the network
    blob = cv2.dnn.blobFromImage(frame, scalefactor=1/255.0, size=(416, 416), swapRB=True, crop=False)

    # Set the input to the network
    net.setInput(blob)

    # Forward pass through the network to get the output
    outputs = net.forward(output_layers)

    # Process the output to get the bounding boxes, class IDs, and confidence scores
    boxes = []
    class_ids = []
    confidences = []
    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                if class_id == 0:  # Check if the detected object is a person
                    current_time = time.time()
                    if current_time - start_time >= publish_interval:
                        person_count += 1  # Increment person count
                        # Publish person count to MQTT topic if it has changed
                        # Check if it's time to publish the person count
                        # Publish person count to MQTT topic if it has changed
                        if person_count != previous_count:
                            client.publish(mqtt_topic, payload=str(person_count))
                            previous_count = person_count
                        start_time = current_time  # Reset start time for next interval
                center_x = int(detection[0] * frame.shape[1])
                center_y = int(detection[1] * frame.shape[0])
                width = int(detection[2] * frame.shape[1])
                height = int(detection[3] * frame.shape[0])
                left = int(center_x - width/2)
                top = int(center_y - height/2)
                boxes.append([left, top, width, height])
                class_ids.append(class_id)
                confidences.append(float(confidence))

    # Apply non-max suppression to remove overlapping bounding boxes
    indices = cv2.dnn.NMSBoxes(boxes, confidences, score_threshold=0.5, nms_threshold=0.4)

    # Draw the bounding boxes and class labels on the frame
    colors = np.random.uniform(0, 255, size=(len(classes), 3))
    if len(indices) > 0:
        for i in indices.flatten():
            x, y, w, h = boxes[i]
            label = classes[class_ids[i]]

            # Draw the bounding box
            color = colors[class_ids[i]]
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, thickness=2)

            # Draw the class label and confidence score
            label = f"{label}: {confidences[i]:.2f}"
            cv2.putText(frame, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness=2)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture object and close all windows
cap.release()
cv2.destroyAllWindows()