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
![montage](https://private-user-images.githubusercontent.com/117952621/300277181-2f004eae-69a0-4ba7-ad5c-6eff9b3a1a1c.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MDY0NTkxMzYsIm5iZiI6MTcwNjQ1ODgzNiwicGF0aCI6Ii8xMTc5NTI2MjEvMzAwMjc3MTgxLTJmMDA0ZWFlLTY5YTAtNGJhNy1hZDVjLTZlZmY5YjNhMWExYy5wbmc_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjQwMTI4JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI0MDEyOFQxNjIwMzZaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT02YjcwMDZmMjhhZGI4ZWM5ZTA1Nzc0ODRiYjkyNTg1NjkxODNiYjU0MzI1ZDE3YTQyZDhlMWM0NmY2ZDg5NzQyJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZhY3Rvcl9pZD0wJmtleV9pZD0wJnJlcG9faWQ9MCJ9.k5dNS5u1QDYeOU_8o5cBOvIdBsWiiOVWq9axW97id5Y)

Conception part:
After taking the measurements, we created a casing to hold the device and to stabilize it:  
![Boitier](https://private-user-images.githubusercontent.com/117952621/300277928-3dedf6ea-c3f6-47e9-b8f4-a7b57d07b71e.jpg?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MDY0NTk4NDIsIm5iZiI6MTcwNjQ1OTU0MiwicGF0aCI6Ii8xMTc5NTI2MjEvMzAwMjc3OTI4LTNkZWRmNmVhLWMzZjYtNDdlOS1iOGY0LWE3YjU3ZDA3YjcxZS5qcGc_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjQwMTI4JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI0MDEyOFQxNjMyMjJaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT02MTg3YThlYjZlYTAzM2FlYmIyZjA1ZDAyMmNmM2VjNzZhNzliNDkwMjJhMDkzNTllMzljMWE1Nzg2YjNhZjk2JlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZhY3Rvcl9pZD0wJmtleV9pZD0wJnJlcG9faWQ9MCJ9.vWmN9_9CrdQHaUXZAgXeAjoXcO59Qz21cC_gA3qW_4E)

![Couvercle](https://private-user-images.githubusercontent.com/117952621/300277940-1c95257e-cdee-4778-b4ce-df5328779fc8.jpg?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MDY0NTk4NDIsIm5iZiI6MTcwNjQ1OTU0MiwicGF0aCI6Ii8xMTc5NTI2MjEvMzAwMjc3OTQwLTFjOTUyNTdlLWNkZWUtNDc3OC1iNGNlLWRmNTMyODc3OWZjOC5qcGc_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjQwMTI4JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI0MDEyOFQxNjMyMjJaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT01Y2E1M2RkMmEyNDkzMmYwNmFkY2MxOTljYzNkZDdkMmUxOTBiOWUxMjU2ODliM2RiMmJhODI2YTFhZWU4Y2NiJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZhY3Rvcl9pZD0wJmtleV9pZD0wJnJlcG9faWQ9MCJ9.s7JnJJaqGVYip4urEYkg9XLbl9giflEAlq2eH3Sf5DM)

Obstacle detection code:
Based on the documentation (https://github.com/luxonis/depthai/blob/main/depthai_sdk/docs/source/samples/mixed/sdk_collision_avoidance.rst), it provide a guidance to implement the navigation system using the OAK-D camera. Morevoer, in the center of the image, a circle segmentation may reduce the noise. An identification of objects by the disparity of neighbor colors allow to detect objects. From the object detected, some information is sent to the arduino system in order to give the command to vibrate on a side.

![Detection](https://private-user-images.githubusercontent.com/117952621/300278978-46fa77a7-e993-438b-a586-d5406dd294d3.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MDY1NTgzOTAsIm5iZiI6MTcwNjU1ODA5MCwicGF0aCI6Ii8xMTc5NTI2MjEvMzAwMjc4OTc4LTQ2ZmE3N2E3LWU5OTMtNDM4Yi1hNTg2LWQ1NDA2ZGQyOTRkMy5wbmc_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjQwMTI5JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI0MDEyOVQxOTU0NTBaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT01ZjE5YmU0OTk5NzJkZjFkZDllMzU0NGMwYTgwODIyNzVhYTI0YjA2ZDI2YjQwMzc4YTA1YzEzMjFhZjRmZGJlJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZhY3Rvcl9pZD0wJmtleV9pZD0wJnJlcG9faWQ9MCJ9.HoaLDFF7WJnY3pLjVK8bHN5ZZXJuH8Jl45CkYeLBa1c).
