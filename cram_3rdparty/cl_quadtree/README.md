cl_quadtree
===========

This library implements Quadtrees and brings following methods:

  * Creating of Quadtrees represented as struct with object, subtrees, boundary and depth parameter
  * Querying of Quadtrees with a possibility to return the quried path
  * Inserting of objects with excluding middlepoints since these increment the depth of a Quadtree
  * Set and get object values (represented as the parameter z in a 3d-vector) in Quadtrees
  * Map functions over Quadtrees
  * Import a matrix into an empty Quadtree
  * Basic minimizing: meaning if the leaves have all the same value or are all empty these leaves can be removed and their parent gets updated
  * Rotation: boundaries of quadtree will be rotated

This library is not unit-tested and may therefore contain bugs. Moreover, the rotation implementation should be changed: the rotation of boundaries is way to heavy since these have to be properly aligned when merging Quadtrees. Therefore, these should be static.
