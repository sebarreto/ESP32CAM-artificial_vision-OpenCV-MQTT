# ESP32CAM-artificial_vision-OpenCV-MQTT
Small project to practice image detection. The idea here is to use esp32cam to detect people and objects. When a person is found, a counter is sent to the Adafruit feed using the MQTT protocol.

![General Architecture](https://i.ibb.co/ckP2xLC/general-arq.png)

## Hardware requirements

**ESP32-CAM**

I used ESP32-S2-WROOM board. Model FNK0085. Brand Freenove.

![ESP32-CAM](https://i0.wp.com/randomnerdtutorials.com/wp-content/uploads/2023/01/Freenove-ESP32.png?w=300&quality=100&strip=all&ssl=1)

Documentation: http://freenove.com/fnk0085
official store: https://store.freenove.com/

**Wires**

Wires male female connector. It'll depend if you used protoboard or not.

![Arduino wires](https://upload.wikimedia.org/wikipedia/commons/thumb/5/5c/A_few_Jumper_Wires.jpg/800px-A_few_Jumper_Wires.jpg)

**Protoboard**

It's not mandatory. It's recommended.

![Protoboard](https://upload.wikimedia.org/wikipedia/commons/thumb/b/be/Protoboard_Unitec.jpg/800px-Protoboard_Unitec.jpg?20141110214556)

## Software solution

For this project I decided to practice with Arduino IDE (C++). I took the example code provided by the ESP32CAM vendor and adapted it to include the WiFi connection and video server implementation.

First, we run the Arduino code (image_detection.ino) from the Arduino IDE. This action will start the video server and show us the IP address on the screen.

Second, we execute Python code ()

