# Mobile Robot Control (PID + Wall Following)

This project was completed as part of my Mobile Robots course. It implements two core behaviors in a maze environment using Python and LIDAR data:

## Task 1: PID Forward Wall Stop
- Robot moves forward using PID control, tuned with Kp, Ki, Kd gains.
- Stops 1m from the end wall without collision.
- Tested with 6 different gain constants (0.2 → 8.0).
- Robot also backs up if placed closer than 1m.

## Task 2: Wall Following
- Robot performs consistent left- or right-wall following.
- Handles 90° turns and sharp corners.
- Tested on `maze3.xml` and `maze4.xml`.
- Certain maze/mode combos are impossible (documented in report).

## Demo
- Video demos available here:
Task 1 [Demo Video](https://usfedu-my.sharepoint.com/:v:/g/personal/scobonavas_usf_edu/EcByNt12y_hBs5AOqGP4960B4_5WWQX4qlUd2nlqRbjppA?e=GKqXz2&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZy1MaW5rIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXcifX0%3D)
Task 2 [Demo Video](https://usfedu-my.sharepoint.com/:v:/g/personal/scobonavas_usf_edu/EbDhFdqhsIdGm_pK6oLro2sBbd_LSW9Of5gHCbQJNWZ93A?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=k6JUjx)

## Tech
- Language: Python
- Sensors: LIDAR
- Control: PID, wall-following logic
- Simulator: [Webots](https://cyberbotics.com/) (maze2.xml, maze3.xml, maze4.xml)

## Results
- Successfully tuned PID constants to achieve smooth stopping at 1m.
- Wall following works for most maze/mode combos.
- Identified limitations when maze geometry prevents full navigation.

