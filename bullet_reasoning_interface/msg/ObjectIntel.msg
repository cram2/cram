# Object types
uint32 BOWL = 0
uint32 NESQUIK = 1
uint32 MONDAMIN = 2
uint32 FRUIT_APPLE = 3
uint32 FRUIT_ORANGE = 4
uint32 SUGAR = 5


# Object's id
string id

# Indicates if the object is visible. 
bool isStable

# Indicates if the object is stable.
bool isVisible

# Object's stamped pose.
geometry_msgs/PoseStamped poseStamped

# Which operation had to be executed?
uint32 operation

# Did the operation succeed?
bool operationSucceeded
