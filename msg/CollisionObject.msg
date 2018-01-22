# a header, used for interpreting the poses
Header header

# the name of the object 
string object_name

# solid geometric primitives
#shape_msgs/SolidPrimitive[] primitives
geometry_msgs/PoseStamped[] primitive_poses
visualization_msgs/Marker visualize_marker_array

# Adds the object to the planning scene. If the object previously existed, it is replaced.
#byte ADD=0

# Removes the object from the environment entirely (everything that matches the specified id)
#byte REMOVE=1

# Operation to be performed
#byte operation
