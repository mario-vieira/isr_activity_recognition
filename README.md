# isr_activity_recognition

The isr_activity_recognition stack was developed for real time human activity recognition using a mobile robot. It uses an RGB-D sensor (Microsoft Kinect) to track the human skeleton and extract features. The classification is done using a Dynamic Bayesian Mixture Model (DBMM) which combines two or more single classifiers to improve the overall classification performance.  
In addition to recognizing activities, reaction can be triggered according to the activity being performed. If a person says "follow me", the robot will follow the person, keeping the monitoring. If the activity detected is "running" or "jumping", a sonorous warning will be triggered. Finally, if the activity detected is "falling", the robot will ask if the person needs help and in affirmative case, it will call a doctor or relative (not implemented).  

**ACTIVITIES:** 

1. Walking
2. Standing still
3. Working on computer
4. Talking on the phone
5. Running
6. Jumping
7. Falling
8. Sitting down

isr_activity_recognition is available at

## Necessary third party libraries

* The NITE library must be manually installed for `openni_tracker` to function. See https://github.com/ros-drivers/openni_tracker

* The scikit-learn must be installed for `classifica_w.py` to work. See http://scikit-learn.org/stable/install.html

* The `sound_play` node must be installed for text-to-speech. See http://wiki.ros.org/sound_play

* The `pocketsphinx` package must be installed for speech recognition. See http://www.pirobot.org/blog/0022/

## Description of packages and other files

* **openni_launch:** This package contains launch files for using OpenNI-compliant devices such as the Microsoft Kinect in ROS.

* **openni_tracker:** The OpenNI tracker broadcasts the OpenNI skeleton frames using `tf`.

* **learning_image_geometry:** This package projects the `tf` frames of the skeleton acquired by the `openni_tracker` onto an image.

* **learninf_tf:** This package uses the `tf_listener` node to get the coordinates of the skeleton being tracked relative to the torso and camera, and the `classifica_w.py` node to recognize the activity being performed.

* **pi_speech_tutorial:** This package contains lauch files for speech recognition.

* **random_navigation_goals:** This package is responsible for robot navigation. The `simple_navigation_goals` node makes the robot randomly navigate the environment. The `follower_speed` node makes the robot follow a person, using velocity commands. The `follower` node makes the robot follow a person if it hear "follow me", avoiding collision with the human. Finally, the `follower_speed` node does the same as the `follower` node, sending velocity commands instead.  
## Usage

```
roslaunch isr_activity_recognition activity_recognition.launch
```


## Additional Information

If you find isr_activity_recognition stack helpful, please cite it as

1. Mario Vieira, Diego R. Faria, Urbano Nunes, Real-time Aplication for Monitoring Human Daily Activities and Risk Situations in Robot-assisted Living, 2015
2. Diego R. Faria, Mário Vieira, Cristiano Premebida, Urbano Nunes, Probabilistic Human Daily Activity Recognition towards Robot-assisted Living, 2015


Software available at http://www.

For any questions and comments, please send your email to
mvieira@isr.uc.pt or diego@isr.uc.pt

**Acknowledgments:**                                                                                                        
This work has been supported by the Portuguese Foundation for Science
and  Technology,  COMPETE  and  QREN  programs  under  Grant  AMS-
HMI12  RECI/EEI-AUT/0181/2012.  The  authors  are  with  Institute  of  Sys-
tems and Robotics, Dept. of Electrical and Computer Engineering, Univer-
sity of Coimbra, Polo II, 3030-290 Coimbra, Portugal. 

