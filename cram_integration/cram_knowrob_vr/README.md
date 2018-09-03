(README.md for lisp_ease repository)
# cram_knowrob_vr

This package provides the functionality to import data which has been recorded in Virtual Reality and stored in a MongoDB Database (or OpenEase) into CRAM, so that it can be used within robot plans. 
This currently means, that from a recorded episode, 'GraspingSomething' Events are extracted and for each event, the start and end time stamps are given, the positions of the object, the head of the user and the hand which has performed the grasping action at the start and end of the event are extracted. 
The package also contains tools for adapting the poses from the Virtual Reality environment and map to the one used within CRAM and the CRAM bullet world simulation. 


## Initialisation of the Environment
In order to be able to use this package, several other programs need to be running. This paragraph will explain which files need to be launched in which order, for the system to be usable.

There are two ways to use this. Either with just a local MongoDB or with OpenEase instead. The only Mongo way is more leightweight, while the OpenEase option allows for visualization of the data within OpenEase. From the CRAM perspective, nothing within the usability changes. 

### Only MongoDB (leightweight option)
Have a Mongo container set up with the Episode Data and run it in the backgroud. It can be set to autostart on boot so one does not have to worry abot it at all later on. 


#### ROS
Once mongo is up and running, launch a roscore and the json_prolog ros node:

    roscore
    
    roslaunch json_prolog json_prolog.launch 
    
### OpenEase 
Using OpenEase instead of just the MongoDB databse, would allow  for visualization of the Data within  OpenEase.

In order to run this with local OpenEase instead of just the MongoDB database, one needs to make sure Mongo is not running beforehand, since otherwise OpenEase might not launch properly and this might lead to connection issues.

    mongo 				;; starts the mongo console
    use admin 			;; gives you admin rights
    db.shutdownServer()	        ;; shuts down the db server

Once the dockerized version of OpenEase is installed, go to the *docker/scripts* folder and execute:
    
    ./start-webrob
    
OpenEase should launch normally. The first time it is launched, a lot of files might need to be downloaded, so please be patient. It will also check some GitHub repositories for changes and updates, so don't be suprised if occasionally things are getting downloaded during startup.

#### ROS
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


## Using the code
The following will describe how to access the VR data and use it within CRAM in order to make the robot perform plans within the bullet world environment.

### Initialization (init.lisp)
In the init.lisp file, in the very first function (init-episde) a connection to the episode data and the MongoDB is established. For the episode data to be loaded correctly, the path of the episode data has to be adjusted accordingly to where the data lies on the disk. Aka the general Episode data and it's semantic map.


The connection to the MongoDB Database has to be established from CRAM. This can be done by calling:

    (init-full-simulation)
    
Or, if preferred, step by step by calling the following:

    (init-episode) ;initializes the episode data and connects to the database
    (init-bullet-world) ;launches the bullet world with kitchen and PR2 robot
    (init-items) ;spawns the in VR used items in the bullet world and 3 axes objects for debugging
    
The (init-full-simulation) function does the above 3 steps in one swoop.

This can take a while to load, please be patient.

### Preparing the experiment (demo-preparation.lisp)
In order for the robot to be able to perform a picking up action, the objects have to be first moved to the locations where the human within the VR interacted with them. To accomplish this, call: 

    (demo-spawn-all-obj-in-place)
    
Will move all the objects that have been used in the current VR episode, to the location at which they were in the beginning of the episode.    

### Running a pick-and-place-action (plan-execution.lisp)
In order to have the simulated robot pick an object up and bring it to the kitchen island, one can call:
    
    (execute-pick-and-place 'cup)
    
The parameter of this function is the name of the object that should be manipulated. Currently six objects are supported, which are: muesli, cup, milk, bowl, fork and spoon.

If anything goes wrong, try a different object or check the API for more debugging options.



## Function descriptions    
### init.lisp
#### (init-episode())
Creates a ROS node and connects to the database (MongoDB), which contains the episode data. Note that the path to where the episode data and the semantic map are located, needs to be adjusted to your system. If other episode data is to be loaded, it has to be adjusted here accordingly. 

#### (init-bullet-world())
Initializes the bullet-world, spawning the kitchen and the robot. It also adds the objects which were used in the recording process of the VR data and which are not part of the typical kitchen set, to the mesh list, so that they can later be spawned in the bullet world. Also contains costmap settings. (based on the bullet world tutorial)

#### (init-items())
Spawns all the kitchen items that were used in the VR data in the bullet world. Includes also three axes objects for debugging purposes.

#### (init-full-simulation())
Calls the three fucntions above in one swoop.


### items.lisp
Contains all the item spawning functions.

#### (add-bowl (&optional (?name 'edeka-red-bowl)))
Adds a new bowl object to the scene, at a hardcoded pose somewhere above the robot. (Since the bowl will be moved to where it was in the scenario, the initial pose does not matter.)
A new name can be given to the bowl, which is important if one needs more then one. If the ?name variable is not set, the default name will be: edeka-red-bowl.

#### (add-cup (&optional (?name :cup-eco-orange))) 
same as the above

#### (add-muesli (&optional (?name :koelln-muesli-knusper-honig-nuss)))
same as the above

#### (add-fork (&optional (?name :fork-blue-plastic)))
same as the above

#### (add-spoon (&optional (?name :spoon-blue-plastic)))
same as the above

#### (add-milk (&optional (?name :weide-milch-small)))
same as the above

#### (add-axes (?optional (?name :axes)))
Adds an axes object which can be used for pose debugging at a random pose in the world. (Initial pose does not matter.)
A new name can be given to the axes object, which is important if one needs more then one. If the ?name variable is not set, the default name will be: axes.


### queries.lisp
Contains all the queries that can be used to access information stored in the MongoDB or OpenEase. These are used within the plans to determine where and object is, where the human was standing, etc.
Each query gets the name of an object as a parameter, and will output the pose or other information about this object, based on when a 'GraspingSomething' event occured.

#### (base-query (object-type))
Contains the query on which all the other queries are based, and which is used within the other queries. It asks for an Event based on the given type of the object. Also, the event has to contain an "objectActedOn", meaning it is a "GraspingSomething" event. It also asks for the Start and End time of the event.

#### (get-event-by-object-type (object-type))
Returns the event in which an "objectActedOn" event has been performed on the object of the given type.

#### (get-object-location-at-start-by-object-type (object-type))
Returns the location of an object of given type at the beginning of the event in the form of a cl-tf:transform.
Meaning, the pose which the object had before it was manipulated by the human. (pick up pose)

#### (get-object-location-at-end-by-object-type (object-type))
Returns the location of an object of given type at the end of the event in the form of a cl-tf:transform.
(placing pose)

#### (get-hand (object-type))
Returns which hand has been used to manipulate the given object. The result can be :left or :right.

#### (get-hand-location-at-start-by-object-type (object-type))
Returns a cl-tf:transform which describes the pose the humans hand had at the start of the "GraspingSomething" Event. The grasping pose the robot uses for his gripper is based on this pose.

#### (get-hand-location-at-end-by-object-type (object-type))
Returns a cl-tf:transform which describes the pose the humans hand had at the end of the "GraspingSomething" Event. 

#### (get-camera-location-at-start-by-object-type (object-type))
Returns a cl-tf:transform which describes the pose of the humans head during the pick up action. The robots location during the pick up action within the environment is based on this pose.

#### (get-camera-location-at-end-by-object-type (object-type))
Returns a cl-tf:transform which describes the pose of the humans head at the placing action. The robots location during the placing action within the environment is based on this pose.

#### (get-table-location)
Returns a cl-tf:transform of the pose of the kitchen-island-table within the environment, which is used for the calculation of the placing pose of the object.
