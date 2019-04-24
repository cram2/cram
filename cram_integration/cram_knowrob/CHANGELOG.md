
ChangeLog
=========

### [0.3.0] - 2017-03-15

2016-12-23 [sem_map_desigs] before asking for CURRENT-DESIG check if an argument is of type desig

2016-12-23 [sem_map_desigs] resolve (a location (of item123)) using semantic map

2017-01-03 [sem_map_utils] bugfix: when using alist hash table stuff don't forget about :TEST

2017-01-19 [sem_map_utils] fixed bug in pose

2017-03-08 [sem_map_cm] bugfix: calculate bounding box for rotated objects correctly

2017-03-15 renamed CRAM_SEMANTIC_MAPS -> CRAM_KNOWROB


### [0.2.2] - 2016-11-15

2016-05-19 [sem_map_utils] support for new way of storing poses in Knowrob: quaternion + translation

2016-06-06 [sem_map_utils] added some error checking in updating semantic map cache

2016-06-08 [sem_map_utils] added debugging output and map_object_dimensions -> object_dimensions

2016-06-15 [sem_map_utils] two changes to support the new semantic map of kitchen:
* children of links are called subComponent and not properPhysicalParts,
* dimensions in the old map were doubles, in the new one they are strings.
Backwards compatibility is assured.

2016-06-15 [sem_map_utils] initialize urdf-name when creating the object: instead of only initializing it on demand.

2016-07-12 [sem-map-costmap] only use objects of type sem-map-geom for costmaps: some semantic map parts are only of type sem-map-part and don't have a pose or dimensions as they are virtual. There is no point in using those for costmaps

2016-11-02 [sem_map_utils] replaced map_object_type with other queries

2016-11-15 cleaned up package.xmls


### [0.2.1] - 2016-01-28

