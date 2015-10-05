# isr_activity_recognition

The isr_activity_recognition stack was developed for real time human activity recognition. It uses an 
RGB-D sensor (Microsoft Kinect) to track the human skeleton and extract features. The classification is
done using a Dynamic Bayesian Mixture Model (DBMM) which combines two or more single classifiers  
In addition to recognizing activities, reaction can be triggered according to the activity being performed. 

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

Necessary third party libraries
===============================
* The NITE library must be manually installed for openni_tracker to function. See https://github.com/ros-drivers/openni_tracker

* The scikit-learn must be intalled for classifica.py to work. See http://scikit-learn.org/stable/install.html

Usage
=====
```
roslaunch isr_activity_recognition activity_recognition.launch
```


Additional Information
======================

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

