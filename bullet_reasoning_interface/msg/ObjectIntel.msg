# Object types
uint32 BOWL = 0
uint32 NESQUIK = 1
uint32 MONDAMIN = 2
uint32 FRUIT_APPLE = 3
uint32 FRUIT_ORANGE = 4
uint32 SUGAR = 5


# Object's id
uint64 id

# Indicates if the object is stable.
bool isStable

# Indicates if the object is visible.
bool isVisible

# Indicates if the object had contact with kitchen before simulation.
bool contactWithKitchenBeforeSimulation

# Indicates if the object had contact with kitchen after simulation.
bool contactWithKitchenAfterSimulation

# Has collision with this objects.
uint64[] collisionWith

# Object's stamped pose.
geometry_msgs/PoseStamped poseStamped

# Object's bounding box in world's fixed frame.
geometry_msgs/Vector3 boundingBox

# Which operation had to be executed?
uint32 operation

# Did the operation succeed?
bool operationSucceeded
