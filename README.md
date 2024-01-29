# virtual-cane
#### University project on a virtual white can for visually impaired people

The project is realised by Evan BOKOBZA, Thibault JEANNOT and Laurent LIN for a master degree project. The aim of the project is to develop a tool to help navigate with obstacles. To do so, a significant isolated part is distributed, in order that everyone can progress smoothly and give regards to the other parts. The creation of the device is divided in three parts: 
 - Electronic (arduino and electronic circuit)
 - Conception (design and 3d printer)
 - Obstacle detection code (python)

Experiment:
  Based on the locomotion cane, we created a device which vibrate when it detects an obstacle adjusted by the potentiometer. Depend on the location of the obstacle the vibrators would vibrate each side or both sides. The computer play the role of a battery for the camera and the arduino, which why the subject needs to carry a laptop to use the device.
  
The materials used to create the device is:
- Camera OAK-D PRO WIDE (RGBD)
- Arduino MKR 1010
- Transistor
- Linear potentiometer
- Vibrators
- 3D printer
- Button
- Computer

Electronic section:
Before realisation of the set-up, a prototype  is made on tinkercad which we ran the arduino to test the arduino code. 
![electronic](https://github.com/Miightx/virtual-cane-M2-SMR/assets/117952621/ecce0642-844c-484a-a8a0-788cd35e087b)


Conception part:
After taking the measurements, we created a casing to hold the device and to stabilize it:  
![Boitier 3D](https://github.com/Miightx/virtual-cane-M2-SMR/assets/117952621/0fbcb637-5a95-4df6-9bf7-7b43a7eabb77)


![Couvercle 3D](https://github.com/Miightx/virtual-cane-M2-SMR/assets/117952621/330c678b-454c-401f-ac92-efc6735935dd)

Obstacle detection code:
Based on the documentation (https://github.com/luxonis/depthai/blob/main/depthai_sdk/docs/source/samples/mixed/sdk_collision_avoidance.rst), it provide a guidance to implement the navigation system using the OAK-D camera. Morevoer, in the center of the image, a circle segmentation may reduce the noise. An identification of objects by the disparity of neighbor colors allow to detect objects. From the object detected, some information is sent to the arduino system in order to give the command to vibrate on a side.

![Detection](https://github.com/Miightx/virtual-cane-M2-SMR/assets/117952621/1efaa287-512c-4a54-8718-a77de1aec3b6)

