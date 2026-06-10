# RoundSumo
A sumo robot that I made for km.edu.uz

# Project description
Hello! I made this robot not only for a competition in my local country, Uzbekistan, but I also made it because of statis on hack club. The name for the robot was inspired by the shape of the robot, because it's "round". The code and the CAD files are available. I used 2 SEN0019 IR sensors on the sides, 2 HC-SR04s for the front, 4 line follower sensors for edge detection, and one L298N to power 4 360 RPM motors. I also used an external button for starting the countdown.

# Bill Of Materials:
|Part name                  |Quantity|Link                     |
|---------------------------|--------|-------------------------|
|JGA25-370 360 RPM 12V motor|4       |https://ali.click/u0bqd1a|
|HC-SR04                    |2       |https://ali.click/vs4zd1c|
|DFRobot SEN0019            |2       |https://ali.click/y4bqd1v|
|ESP-WROOM32                |1       |https://ali.click/bs4zd1g|
|L298N motor driver         |1       |https://ali.click/mu4zd1g|
|CR3050B 5V converter       |1       |https://ali.click/xebqd1w|
|KY-033 line sensor         |4       |https://ali.click/at4zd1n|
|65mm silicone wheel & tire |4       |https://ali.click/rc5zd1hбб(tire), https://ali.click/pd5zd10(wheel and couplings)|

# CAD
Here is a screenshot of what the robot in CAD form looks like:

<img width="970" height="748" alt="2026-04-23_21-16-52" src="https://github.com/user-attachments/assets/a0e59930-4a0f-4e99-91de-fa235177bcff" />

# Circuit diagram
Also, here is the circuit diagram for my ESP32(Check pinout on your specific model of esp32, because I used esp-wroom32, while you may be using another model. Connect pins according to your diagram):

<img width="1585" height="778" alt="2026-05-06_21-00-55" src="https://github.com/user-attachments/assets/0ec9be62-9386-472e-8968-40368b2431d4" />

# Pinouts
And here are the pinouts, just in case:

IR SENSORS:
Left: 34,
Right: 35


Ultrasonic sensors:
TRIG Left: 4,
ECHO Left: 16,
TRIG Right: 5,
ECHO Right: 17


LINE SENSORS:
Front Left: 32,
Front Right: 33,
Back Left: 21,
Back Right: 22


MOTORS:
ENA: 18,
IN1: 14,
IN2: 27,
IN3: 26,
IN4: 25,
ENB: 19

# The robot in action
**The progress, images of the build are at the bottom**

https://github.com/user-attachments/assets/c455ae6e-ef34-4682-b008-044727d600a6

https://github.com/user-attachments/assets/11a2001b-4e5e-49d3-a010-1698324c45ee

https://github.com/user-attachments/assets/0a8def3c-9945-4a7f-9428-d5f1029c288a

# Progress, images of the build
And, here are some photos of the progress))
<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/10a04574-0195-4605-acf1-b0d4c3f64ebd" />

<img width="1080" height="638" alt="image" src="https://github.com/user-attachments/assets/3025856b-64a5-4396-b4b8-009b5505cd46" />

<img width="1280" height="960" alt="image" src="https://github.com/user-attachments/assets/6f4ac030-68b4-4ef1-8f3a-9cf3d9ce6818" />

<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/2371168a-e58d-4bbc-9663-da62c81e4b08" />

<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/20c0f1b9-6d65-4b93-91ef-4c7d1e630f51" />
