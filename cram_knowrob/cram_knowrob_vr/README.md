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

#### (add-bowl (&optional (?name :edeka-red-bowl)))
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

#### (get-table-location ())
Returns a cl-tf:transform of the pose of the kitchen-island-table within the environment, which is used for the calculation of the placing pose of the object.

### openease-to-bullet.lisp
Contains all functions which manipulate the data that is obtained from the VR environment, so that it can be used in the plans. Some further manipulation is sometimes necessary depending on the use case of the data, some are more generic and therefore in a different file.

#### (apply-bullet-transform (transform))
Applies a transform to the given transform so that the position of the object within the bullet world is correct. Without this offset, the object would appear in a different spot within the bullet world as in the VR world, even if both environments are the same. There is an offset between the two worlds which is corrected by applying this function to the transform of an object or the robot.

#### (apply-bullet-rotation (transform))
Same as the above, just that it fixes the rotation only and not the 3d-vector offset between the two environments (VR and bullet world kitchen.)

#### (quaternion-w-flip (pose))
Flips the quaternion of the recived pose from wxyz (OpenEase standard) into xyzw (CRAM standard). Is not needed when using the data from the mongoBD directly, but when using OpenEase, it might need to be commented in again (in the make-pose function within the data-manipulation) file, depending on the used OpenEase version.

#### (remove-z (pose))
Removes the z component of a transform. Is used for deducing the position within the kitchen of the human/robot, since the only data one can deduce the positon from is the position of the camera/head of the human. 

#### (human-ro-tobot-hand-transform ())
Defines the offset between the human hand from the virtual reality to the
robot standart gripper, which has been calculated manually.


### data-manipulation.lisp
Contains the remaining, more general data manipulating functions, which don't necessarily have anything to do with correcting the offset between the VR and bullet world.

#### (make-pose (pose))
Makes a proper cl-tf:transform out of the data received from the mongoDB or OpenEase, since in both cases the received result would have been a list of seven values. These are separated into the 3d vector and the quaternion here, which form the resulting transform. Also the correcting steps which fix the offsets between the two worlds and which were described in the previous file, are applied here. 

#### (apply-rotation (transform))
Rotates the given transform around the z axis for pi/2.

#### (add-pos-offset-to-transform (transform x y z))
Adds a position offset to a given transform. Can be used to add an offset to the positon of an object, in order to test if the robot would still be able to interact with it, even if the position expected does not match.

### designators.lisp
Contains the designator definitions for grasping the objects relevant for the current scenario, which are all the objects described and added in the items.lisp file.

### movement.lisp
All the functions which move either the robot, parts of the robot or objects within the bullet world are collected here.

#### (move-obj-with-offset (x-offset y-offset object))
Moves the object to the given x and y offset. It does not change the z aspect of the object, since the usecase of this function is to just move an object on the surface of the table to test if the robot would still be able to interact with it, even after a change in position.

#### (move-head (pose))
Moves the head of the robot to the given pose.

#### (move-object (transform object))
Moves the given object to the given transform.

#### (move-robot (transform))
Moves the robot to the given transform.

#### (move-torso-up (?angle))
Moves the torso up by a given angle.

### utils.lisp
Contains many of the utility functions. Some of them are just handy and make the usage a bit easier and basically don't have to be used and are therefore convenience functions. 

#### (object-type-filter-prolog (object-type))
Maps the simple name of an object, e.g. 'cup to the proper one used within the mondoDB or OpenEase database, e.g. "CupEcoOrange".

#### (object-type-filter-bullet (object-type))
Maps the simple name of an object, e.g. 'cup to the proper one used within the bullet world, e.g. :cup-eco-orange

#### (object-type-fixer (object-type))
Since some names of the objects differ between the known names in bullet world and the names known to the database, this function makes sure that if they are mismatched, it will get fixed to the proper one so that the plans can still be executed.
This function can be removed once new data is recorded and the object names will be the same for the bullet world and the Virtual Reality.

#### (move-object-to-starting-pose (object))
Moves the object to it's starting position. Sarting position meaning where the object was located when the human started picking it up in the Virtual Reality.

#### (place-offset-transform ())
Contains the transform which describes the offset for placing an object.

#### (parse-str (str))
A string parser. Parses the output from knowrob into a proper string which can be used within prolog. 

#### (transform-to-pose-stamped (transform))
Converts the given transform into a stamped one within the "map" frame and a time stamp of 0.0.


### robot-positions-calculations.lisp
Functions which calculate the positions the robot has to navigate to or to move his head or hand to, in order to interact with the objects. 

#### param: \*human-feet-offset* 0.05
Sets the human-feet-offset, which corrects the robots positon. This is necessary in cases and episodes, where the human was standing too close to a table in the VR, and therefore if the robot would try to move to the same position, he would hit the base of the table, since the robots base (or foot) is a lot larger then the human one. 
In some VR episodes this parameter is not needed at all and can be set to 0.0, but at the moment it is safer to keep it at 0.05.

#### (set-grasp-base-pose (transform))
Calculates the transform of where the robot should stand in order to interact
with an object. Based on data from Virtual Reality. This function removes the z
component of the Camera pose of the human and also fixes the quaternion so that
the robot won't tilt into the pane of the floor.

#### (set-grasp-look-pose (transform))
Transforms the given transform of the position of an object, into a pose
stamped, at which the robot will look for an object.

#### (set-place-pose (transform))
Takes the given transform of the placing the object pose, sets the
quaternion to an identity one and transforms the transform into a pose stamped.

#### (place-pose-btr-island (type))
Calculates the placing pose for an object, relative to the bullet world
kitchen island. This is needed, since the OpenEase kitchen island and the
bullet world kitchen island, are slightly offset to one another, and the offset
fixing the general semantic map offset, is not enough to fix it.

### grasping.lisp
Implements several of the functions needed in order for the robot to perform a succesfull grasp, based on the one performed by the human in the Virtual Reality. Aka. Calculates the grasping pose, pre-pose and the lifting poses. 

#### (get-object-type-to-gripper-transform (object-type object-name arm grasp (eql :human-grasp)))
Calculates the object-type to gripper transform based on the transform the human has used in the Virtual Reality. 

#### (get-object-type-to-gripper-pregrasp-transform ((object-type object-name arm(grasp (eql :human-grasp)) grasp-transform))
Calculates the object-type to gripper pre-transform based on the transform the human has used in the Virtual Reality. 

#### (get-object-type-to-gripper-2nd-pregrasp-transform (object-type object-name arm(grasp (eql :human-grasp)) grasp-transform))
Calculates the object-type to gripper 2nd-pre-transform based on the transform the human has used in the Virtual Reality. 

#### (get-object-type-to-gripper-lift-transform ((object-type object-name arm(grasp (eql :human-grasp)) grasp-transform)))
Calculates the lift transform for the gripper.


#### (get-object-type-to-gripper-2nd-lift-transform ((object-type object-name arm(grasp (eql :human-grasp)) grasp-transform)))
Calculates the 2nd lift transform for the gripper.

### plans.lisp
Contains all the plans for pick and place manipulation with designators.

#### (move-to-object (?grasping-base-pose ?grasping-look-pose))
Moves the robot into the position which the human had when interacting
with an object. The robot is placed at the spot where the human was standing and
is looking at the spot where the object was in Virtual Reality.

#### (pick-up-obj (?type))
Picks up an object of the given type.

#### (pick-up-object (?grasping-base-pose ?grasping-look-pose ?type))
A plan to pick up an object of the given time. This time the grasping base position and looking positon cam be passed to this function.

#### (place-object (?placing-base-pose ?placing-look-pose ?place-pose ?obj-desig))
A plan to place an object which is currently in one of the robots hands.

#### (pick-and-place (?grasping-base-pose ?grasping-look-pose ?placing-base-pose ?placing-look-pose ?place-pose ?type))
Picks up and object and places it down based on Virtual Reality data.

### demo-preparation.lisp
Functions which prepare the environment for the pick and place execution.

#### (demo-spawn-all-obj-in-place ())
Spawns all the objects which are present in the current episode in the respective locations.

### plan-execution.lisp
Executes some of the plans that are specified in the plan.lisp file.

#### (execute-pick-and-place (type))
Executes the pick and place plan on an object of the given type.
The positions of where the robot looks for the object and where he is placing
it down are the ones extracted from Virtual Reality.

#### (execute-pick-up-object (type))
Executes only the picking up action on an object given the type of the object.

#### (execute-place-object (?obj-desig type))
Executes the placing action given the object designator of the picked up and
held in hand object. The placing pose is the one used in VR for that kind of 
object.
#### execute-move-to-object (type)
Moves the robot to the position where the human was standing in order to
grasp the object.

### demo-plans.lisp
Contains some of the plan-executions used for demo puposes.

#### (demo-all-pick-place ())
Picks and places all objects of an episode one by one. Meaning the robot will always hold on to just one object and finish placing it before going back to picking up another one.

#### (demo-all-obj ())
For the entire episode, first place the object at the location where it was
for the robot to pick up, and then pick it up and place it.

#### (execution-adjustment-test (type))
Function to test if the newly done adjustments to the plans are still working. Calls one pick-up-object plan.

### gaussian.lisp
(In progress) will contain an implementation auf gaussian functions for the base-pose of the robot and maybe more.

### debugging-utils.lisp
Contains a lot of debuggin convenience functions, which the average user probably (hopefully) will not need.

#### (reset-simulation ())
Resets the simulation and respawns the objects. 

### utility-queries.lisp
Contains the lisp implementations of many of the prolog queries, so that they can be called easier, within lisp, with just a string as a parameter, instead of having to use the prolg-simple function. One can use them, but one can also just call prolog directly. Preference choice. 



