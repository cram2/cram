# lisp_ease

This package provides the functionality to import data which has been recorded in Virtual Reality and stored in OpenEase (MongoDB) into CRAM, so that it can be used within plans. 
This currently means, that from a recorded episode, 'GraspingSomething' Events are extracted and for each event, the start and end time stamps are given, the positions of the object, the head of the user and the hand which has performed the grasping at the start and end of the event are extracted. 
The package also contains tools for adjusting the poses from the Virtual Reality environment and map to the one used within CRAM and the bullet world. 


## Contents
The following gives an overview over the files and their contents.

```
    lisp_ease
    ├─ rescource
    │  ├─...stl
    │  ├─...dae
    │  └─...blend
    ├─ src
    │  ├─lisp-ease.asd
    │  └─queries
    │    ├─init
    │    │ 
    │
    │
    │
    
```






## Initialisation of the Environment
In order to be able to use this package, several other programs need to be running. This paragraph will explain which files need to be launched in which order, for the system to be usable.

### MongoDB
Make sure the MongoDB is not running, since otherwise Openease might not launch properly. 

    mongo 				;; starts the mongo console
    use admin 			;; gives you admin rights
    db.shutdownServer()	        ;; shuts down the db server
    
### OpenEase
Once the dockerized version of OpenEase is installed, go to the *docker/scripts* folder and execute:
    
    ./start-webrob
    
OpenEase should launch normally. The first time it is launched, a lot of files might need to be downloaded, so please be patient. It will also check some GitHub repositories for changes and updates, so don't be suprised if occasionally things are getting downloaded during startup.

### ROS
A rosbridge is needed in order to allow OpenEase to communicate with the local system, since otherwise it is contained within docker.
First, start a roscore in a separate terminal, so in case of needing to restart any of the components, they can be restarted separately.

    roscore
    
After that, the launch file can be launched:

    roslaunch knowrob_roslog_launch knowrob_ease.launch
    
The CRAM-bullet-world environment needs to be setup also, which can be done with the following:

    roslaunch cram_bullet_world_tutorial world.launch
    
### Emacs
Startup emacs with whichever command you prefer, load the lisp_ease package and call the setup functions. It should be something among the lines of the following: 

    emacs
    M-x slime                ;; starts the slime environment, aka. repl 
    
    , r-s-l TAB ENTER        ;; ros-load-system
    lisp_ease ENTER ENTER    ;; package name and confirm
    
    , in-package ENTER       ;; in-package command
    le ENTER                 ;; alias for the lisp_ease package and confirm
    
Now one is in the package and can initialize what is needed there. 
