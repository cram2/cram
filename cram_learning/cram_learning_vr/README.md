# ROS Interface for cram/costmap_learning

This CRAM package connects learned object placements from the ROS python package [costmap_learning](../../../costmap_learning) and the referencing 
of location designators by implementing methods from the CRAM package cram_manipulation_interfaces. This package depends on the current used 
CRAM kitchen, since the learned object placements were recorded in the global map frame.

## Using

First install the ROS python package costmap_learning and run it with roslaunch. After that load your CRAM demo together with this CRAM package.
This package will automatically create a CRAM ros node, if none was created.

## Documentation

### [./src/manipulation-interfaces.lisp](manipulation-interfaces.lisp)

In [./src/manipulation-interfaces.lisp](manipulation-interfaces.lisp) the methods man-int:get-object-destination and man-int:get-object-likely-location
are implemented, which get the symbolic location of the given object type and context from the package costmap_learning. The result is a valid location
designator, which can be used for the target navigation to the symbolic location. This designator will then be later dereferenced in the implemented
man-int:get-location-poses method, which returns a cram:location-costmap object saving the learned object placements from costmap_learning.

### [./src/costmap_client.lisp](costmap_client.lisp)

In [./src/costmap_client.lisp](costmap_client.lisp) the ROS interfaces of the services GetCosmtap and GetSymbolicLocation are implemented. 
GetSymbolicLocation allows to return the symbolic storage or destiantion location of an object type. For this information the service
needs more information, which is shown in the srv file in costmap_learning. 
GetCosmtap returns the parameters of the learned GMMs distributions. These are unwrapped and inputted in a cram:location-costmap object
by using cram:gmm objects.
