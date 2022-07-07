# cram_knowrob

Libraries for using knowrob in Lisp.

## cram_cloud_logger

Runs with Knowrob on commit 59baa3f6c856bd977b54dd51d35e028520aea77d

If the kitchen.urdf is missing from iai_maps, build it from the xacro like so:
```
roscd iai_maps/../iai_kitchen/urdf_obj
xacro iai_kitchen_python.urdf.xacro > kitchen.urdf
```
