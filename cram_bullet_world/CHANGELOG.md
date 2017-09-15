
ChangeLog
=========

### [0.3.0] - 2017-03-15

2016-11-16 renamed cram_spatial_relations_cm into cram_btr_cm

2016-11-17 [btr] package.xml to version 2 format

2016-11-17 [btr] commented out spawning ros_household_objects: nobody's using it, just adds an unnecessary dependency

2016-11-17 [btr] visibility-facts also depend on robot-model-facts

2016-11-22 [btr] commented out make-seed-states as it's not used anywhere

2016-11-22 [btr] moved GET-LINK-CHAIN and GET-JOINT-CHAIN to cl_urdf package

2016-11-24 [btr] got rid of IK calls: it's really not Bullet world's responsibility to calculate IKs. Should now use cram_pr2_arm_kinematics package for an implementation of compute-ik

2016-11-24 [btr] adopted a slightly different new compute-ik signature

2016-12-13 [btr] added colored-box primitive object for bt world

2017-02-03 [cl_bullet_vis] increased distance to z plane to 1000 for visualizing big maps

2017-03-13 [btr_utils] took over 2 utility functions from sherpa: TRANSLATE-OBJECT and SPAWN-OBJECT-AABB-BOX.

2017-03-15 [btr] better way of applying colors to objects. Solves #5


### [0.2.2] - 2016-11-15

2016-01-21 [spatial_rels_cm] cleaned up

2016-01-28 [btr] added missing dependency on household_objects_database_msgs

2016-02-22 CRAM_TRANSFORMS_STAMPED -> CRAM_TF

2016-02-22 [belief_state] moved TF initalization to CRAM_TF

2016-03-04 [btr] moved pose-related facts to CRAM_TF

2015-11-12 added new stls for manmade objects

2016-03-04 renamed household objects into items: for applications outside of human household

2016-03-09 [btr] moved implementation of AVAILABLE-ARMS here from PR2_DESCRIPTION

2016-07-12 [btr] default colors for urdf objects

2016-07-15 Perceive-objects: Use keywords rather than symbols

2016-08-02 Besides symbols, also allow strings as object names

2016-11-15 [btr-utils] return perceived-object after having perceived an object

2016-11-15 cleaned up package.xmls


### [0.2.1] - 2016-01-28
