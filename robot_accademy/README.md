# Robot Academy Classes
[Robot Academy](https://github.com/Learning-and-Intelligent-Systems/lis_pr2_pkg/wiki/Robot-Academy)

### Class 1
Make the robot dance ([code](https://github.com/jingxixu/lis-work/blob/master/robot_accademy/dance_jingxi.py), [video](https://youtu.be/N1F5F4eFNtI))

### Class 2
- create a custom rviz profile under `~/.rviz/jingxi.rviz`
- record a video while driving via teleop and save it under `~/jingxi/jingxi.avi`

### Class 3
Create a closed loop motion ([code](https://github.com/jingxixu/lis-work/blob/master/robot_accademy/reactive_jingxi.py), [video](https://youtu.be/5Gty81elUDA))
- the robot keeps saying "show me something"
- if a red block is shown to its head kinect camera, it says "I do not like red"
- if a green block is shown to its head kinect camera, it says "yeah, baby" and then waves arms

### Class 5
This is the same closed loop demo as class 3 but directly uses a simple action client to control the movement of arms
instead of using the functions from uber controller to do so ([code](https://github.com/jingxixu/lis-work/blob/master/robot_accademy/sac_reactive_jingxi.py))


