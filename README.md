# Final Project
> Final Project in RC autonomous car
## Autonomous car
Autonomous vehicles are automobiles that can move without any intervention by detecting the road, traffic flow and surrounding objects with the help of the control system they have. These vehicles can detect objects around them by using technologies and techniques such as RADAR,  GPS, Odometry, Computer Vision. The autopilot drive of autonomous vehicles starts briefly with the ultrasonic sensors on its wheels, detecting the positions of vehicles that are braking or parked, and data from a wide range of sensors(Cameras , GPS etc..) are analysed with a central computer system and events such as steering control, braking and acceleration are performed.

## Purpose 

We are aiming to add emergency vehicle priority awareness feature to autonomous cars. In our project, we plan to use Artificial Intelligence, Machine Learning, Image Processing methods and test the results in simulation environment. The Autonomous Vehicle Drive Simulator that we will use need to provide us to simulate sensors.

## Tools to be used in this project
1) ESP32
> In this section we will cover all informations that we should know about ESP32.
3) Arduino IDE
> Code we to calibirate the RC car in Arduino we will see in this section how we used servo motors with RC speed controller.
5) Android Studio
> The code of to detect lanes and street signs with TF lite models and OpenCV
7) OpenCv Library
8) YoloV5 / RCNN / Mobile Net v2.
9) Deep Sort / Kalman filter for tracking purpose.

# ESP32
<img src="https://user-images.githubusercontent.com/58775369/185209020-ddea0242-99da-4150-aa80-dbba22ac8b0e.jpeg" width="400" height="400">
 >ESP32 is a low-cost, low-power Microcontroller with an integrated Wi-Fi and Bluetooth. It is the successor to the ESP8266 which is also a low-cost Wi-   >Fi microchip albeit with limited vastly limited functionality.

### OpenCV + Android Studio
1) Collecting the frames of the Realtime video.
2) Navigating the vehicle.
3) Compassing information.
4) Running the AI models for e.g. YoloV5 & Deep Sort.
5) Running the Computer vision's algorithms (OpenCV library).
6) Andruav for Navigating.
7) Two android phones connected to transfer data.
## Cameras:
One camera for Lane Detection and Object Detection.

