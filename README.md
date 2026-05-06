# RoundSumo
A sumo robot that I made for km.edu.uz

Hello! I made this robot not only for a competition in my local country, Uzbekistan, but I also made it because of statis on hack club. The name for the robot was inspired by the shape of the robot, because it's "round". The code and the CAD files are available. I used 2 SEN0019 IR sensors on the sides, 2 HC-SR04s for the front, 4 line follower sensors for edge detection, and one L298N to power 4 360 RPM motors. I also used an external button for starting the countdown.

Here is the circuit diagram for my ESP32(Check pinout on your specific model of esp32, because I used esp-wroom32, while you may be using another model. Connect pins according to your diagram):

<img width="1585" height="778" alt="2026-05-06_21-00-55" src="https://github.com/user-attachments/assets/0ec9be62-9386-472e-8968-40368b2431d4" />

And here are the pinouts, just in case:

IR SENSORS:
Left: 34
Right: 35

Ultrasonic sensors:
TRIG Left: 4
ECHO Left: 16
TRIG Right: 5
ECHO Right: 17

LINE SENSORS
Front Left: 32
Front Right: 33
Back Left: 21
Back Right: 22

MOTORS
ENA: 18
IN1: 14
IN2: 27
IN3: 26
IN4: 25
ENB: 19

And, here are some photos of the progress))
<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/10a04574-0195-4605-acf1-b0d4c3f64ebd" />

<img width="1080" height="638" alt="image" src="https://github.com/user-attachments/assets/3025856b-64a5-4396-b4b8-009b5505cd46" />

<img width="1280" height="960" alt="image" src="https://github.com/user-attachments/assets/6f4ac030-68b4-4ef1-8f3a-9cf3d9ce6818" />

<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/2371168a-e58d-4bbc-9663-da62c81e4b08" />

<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/20c0f1b9-6d65-4b93-91ef-4c7d1e630f51" />
