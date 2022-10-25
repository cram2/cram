# cram_knowrob

Libraries for using knowrob in Lisp.

## cram_cloud_logger

Runs with Knowrob on commit 59baa3f6c856bd977b54dd51d35e028520aea77d

Create a directory where the neems are saved and set the KNOWROB_MEMORY_DIR to that location:
```
mkdir ~/neems
export KNOWROB_MEMORY_DIR=${HOME}/neems # put that in ~/.bashrc 
```

If the kitchen.urdf is missing from iai_maps, build it from the xacro like so:
```
roscd iai_maps/../iai_kitchen/urdf_obj
xacro iai_kitchen_python.urdf.xacro > kitchen.urdf
```
