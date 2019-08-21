# Quadtree

[![Build Status](https://travis-ci.org/takagi/quadtree.svg?branch=master)](https://travis-ci.org/takagi/quadtree)
[![Coverage Status](https://coveralls.io/repos/takagi/quadtree/badge.svg?branch=master)](https://coveralls.io/r/takagi/quadtree?branch=master)

Quadtree is quadtree data structure in Common Lisp.

## Usage

Here shows how to store points into a quadtree.

First, define point structure.

    (defstruct (point (:constructor make-point (x y)))
      (x 0d0 :read-only t)
      (y 0d0 :read-only t))

Then, implement an intersection test method for the point structure, which is called when inserting points to a quadtree.

    (defmethod quadtree:intersect-p (quadtree (point point))
      (destructuring-bind (x0 y0 x1 y1) (quadtree:boundary quadtree)
        (let ((x (point-x point))
              (y (point-y point)))
          (and (<= x0 x x1) (<= y0 y y1)))))

Now, make and bulid a quadtree with its boundary, query it to get points stored in the leaf that contains specified `x` and `y`.

    (let ((points (list (make-point 0.0d0 0.0d0)
                        (make-point 1.0d0 1.0d0)
                        (make-point 0.1d0 0.1d0)
                        (make-point 0.1d0 0.2d0)
                        (make-point 0.2d0 0.1d0)
                        (make-point 0.2d0 0.2d0)
                        (make-point 0.2d0 0.3d0)
                        (make-point 0.2d0 0.4d0)
                        (make-point 0.3d0 0.2d0)
                        (make-point 0.3d0 0.3d0))))
      ;; make a quadtree with its boundary
      (let ((qt (quadtree:make 0.0d0 0.0d0 1.0d0 1.0d0)))
        ;; build the quadtree
        (loop for point in points
           do (quadtree:insert qt point))
        ;; query the quadtree
        (is (length (quadtree:query qt 0.1 0.1)) 5)))


## Installation

Since quadtree is just requesting to Quicklisp, plese use its local-projects feature for now.

    $ cd quicklisp/local-projects
    $ git clone git://github.com/takagi/quadtree.git

After approved, you can install via Quicklisp.

    (ql:quickload :quadtree)

## API

### [Special Variable] \*max-depth\*

Specifies the maximum depth of a newly created quadtree.

### [Special Variable] \*max-capacity\*

Specifies the maximum capacity of leaves of a newly created quadtree.

### [Function] make

    MAKE x0 y0 x1 y1 &key max-depth max-capacity => quadtree

Creates an empty quadtree with the boundary of `x0`, `y0`, `x1` and `y1`. `max-depth` specifies the maxinum depth of a quadtree and its default value is provided by `*max-depth*` special variable. `max-capacity` is the maxinum capacity of a quadtree leaf, subdevided if the value is exceeded, and its default value is provided by `*max-capacity*` special variable. However, leaves at the maximum depth contain all inserted objects regardless of the capacity.

### [Function] insert

    INSERT quadtree object => boolean

Inserts `object` to `quadtree`. If suceeded, returns `t`. Otherwise, returns `nil`. The case it fails to inserted an object to a quadtree is that it is out of boundary of the quadtree. If the number of objects in a leaf is exceeded, the leaf is subdevided into four subtrees and its objects are distributed to them.

### [Function] query

    QUERY quadtree x y &optional neighbor-p => object-list

Queries objects at the point `(x, y)` to `quadtree`. If `neighbor-p` is `nil`, returns objects stored in the leaf that contains the point. Otherwise, additionally returns objects stored in leaves neighboring top, bottom, left and right to the leaf that contains the point. The default value of `neighbor-p` is `nil`.

### [Function] clear

    CLEAR quadtree => boolean

Clears the contents of `quadtree`. The return value is always `t`.

### [Function] boundary

    BOUNDARY quadtree => boundary

Returns the boundary of `quadtree`, represented by a list of `x0`, `y0`, `x1` and `y1`. It is intended to be used in following `intersect-p` function to determine if an object intersects a quadtree.

### [Generic Function] intersect-p

    INTERSECT-P quadtree object => boolean

Called when `object` is being inserted to `quadtree` which stores objects and returns if `object` intersects `quadtree`. Library users should implement this generic function as the object's structure or class they are storing into a quadtree.

## Author

* Masayuki Takagi (kamonama@gmail.com)

## Copyright

Copyright (c) 2015 Masayuki Takagi (kamonama@gmail.com)

## License

Licensed under the MIT License.

