# Valid operations
uint32 SPAWN_OBJECT = 0
uint32 MOVE_OBJECT = 1
uint32 REMOVE_OBJECT = 2
uint32 QUERY_STATUS = 3


# Object's id
string id

# Operation to execute
uint32 operation

# Object's type. Only necessary when spawning an object.
uint32 type

# Object's stamped pose. Only necessary when spawning or moving an object.
geometry_msgs/PoseStamped poseStamped

# Object's color. Only necessary when spawning an object.
std_msgs/ColorRGBA color

# Object's bounding box in world's fixed frame.
geometry_msgs/Vector3 boundingBox
