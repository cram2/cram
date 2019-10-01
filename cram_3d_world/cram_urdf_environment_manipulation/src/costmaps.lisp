;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :env-man)

(defun get-handle-min-max-pose (container-name btr-environment)
  "Return a list of two poses representing the container's handle position when
at its min and max joint-limits.
CONTAINER-NAME and BTR-ENVIRONMENT are the names of the container and the
environment, in which it can be found, respectively."
  (let* ((handle-link
           (get-handle-link
            container-name
            btr-environment))
         (neutral-handle-pose
           (get-manipulated-pose
            (cl-urdf:name handle-link)
            0
            btr-environment
            :relative T))
         (manipulated-handle-pose
           (get-manipulated-pose
            (cl-urdf:name handle-link)
            1
            btr-environment
            :relative T)))
    (list neutral-handle-pose manipulated-handle-pose)))

(defun get-aabb (container-name btr-environment)
  "Return the aabb of the container.
CONTAINER-NAME and BTR-ENVIRONMENT are the names of the container and the
environment, in which it can be found, respectively."
  (btr:aabb
   (btr:rigid-body
    (btr:object btr:*current-bullet-world* btr-environment)
    (btr::make-rigid-body-name
     (string-upcase btr-environment)
     (roslisp-utilities:rosify-underscores-lisp-name container-name)))))

(defun get-width (container-name direction btr-environment)
  "Return the width of the container appropriate for the joint DIRECTION
3d-vector given.
CONTAINER-NAME and BTR-ENVIRONMENT are the names of the container and the
environment, in which it can be found, respectively."
  (let ((dimensions (cl-bullet:bounding-box-dimensions
                     (get-aabb container-name btr-environment)))
        (norm-direction (cl-transforms:normalize-vector direction)))
    (if (> (abs (cl-transforms:x norm-direction))
           (abs (cl-transforms:y norm-direction)))
        (cl-transforms:y dimensions)
        (cl-transforms:x dimensions))))

;; NOTE(cpo): It might be useful to pass this the desired position
;; and calculate the current one to make the costmap more precise.
(defun make-opened-drawer-cost-function (container-name btr-environment
                                         &optional (padding 0.2))
  "Resolve the relation according to the poses of the handle of a container
in neutral and manipulated form.
CONTAINER-NAME and BTR-ENVIRONMENT are the names of the container and the
environment, in which it can be found, respectively."
  (let* ((handle-link
           (get-handle-link container-name
                            btr-environment))
         (handle-pose
           (get-manipulated-pose
            (cl-urdf:name handle-link)
            0
            btr-environment
            :relative T))
         (manipulated-handle-pose
           (get-manipulated-pose
            (cl-urdf:name handle-link)
            1
            btr-environment
            :relative T))
         (neutral-point
           (cl-transforms:make-3d-vector
            (cl-transforms:x (cl-transforms:origin handle-pose))
            (cl-transforms:y (cl-transforms:origin handle-pose))
            0))
         (manipulated-point
           (cl-transforms:make-3d-vector
            (cl-transforms:x (cl-transforms:origin manipulated-handle-pose))
            (cl-transforms:y (cl-transforms:origin manipulated-handle-pose))
            0))
         (V
           (cl-transforms:v- manipulated-point neutral-point))
         (width
           (get-width container-name V btr-environment)))
    (lambda (x y)
      (multiple-value-bind (a b c)
          (line-equation-in-xy neutral-point manipulated-point)
        (let* ((P (cl-transforms:make-3d-vector x y 0))
               (dist (line-p-dist a b c P))
               ;;(dist-p (line-p-dist-point a b c P))
               )
          (if (and
               (< dist (+ (/ width 2) padding))
               ;; Commenting this out for now, so there won't be poses in front of the drawer.
               ;;(< (cl-transforms:v-norm (cl-transforms:v- dist-p neutral-point))
               ;;   (+ (cl-transforms:v-norm V) padding))
               )
              0
              1))))))

(defun make-opened-drawer-side-cost-function (container-name arm
                                              btr-environment)
  "Resolve the side relation according to the poses of the handle of a container
in neutral and manipulated form. Disregard any samples where the robot would
stand on the same side as the arm it wants to use.
CONTAINER-NAME and BTR-ENVIRONMENT are the names of the container and the
environment, in which it can be found, respectively.
ARM has to be either :left or :right."
  (let* ((handle-link
           (get-handle-link container-name
                            btr-environment))
         (handle-pose
           (get-manipulated-pose
            (cl-urdf:name handle-link)
            0
            btr-environment
            :relative T))
         (manipulated-handle-pose
           (get-manipulated-pose
            (cl-urdf:name handle-link)
            1
            btr-environment
            :relative T))
         (neutral-point
           (cl-transforms:make-3d-vector
            (cl-transforms:x (cl-transforms:origin handle-pose))
            (cl-transforms:y (cl-transforms:origin handle-pose))
            0))
         (manipulated-point
           (cl-transforms:make-3d-vector
            (cl-transforms:x (cl-transforms:origin manipulated-handle-pose))
            (cl-transforms:y (cl-transforms:origin manipulated-handle-pose))
            0))
         (AB
           (cl-transforms:v- manipulated-point neutral-point)))
    (lambda (x y)
      (let ((d (cl-transforms:z
                (cl-transforms:cross-product
                 AB
                 (cl-transforms:v- (cl-transforms:make-3d-vector x y 0)
                                   neutral-point)))))
        (if (and (< d 0) (eql arm :right))
            1.0
            (if (and (> d 0) (eql arm :left))
                1.0
                0.0))))))

(defun make-opened-door-cost-function (container-name btr-environment
                                       &optional (padding 0.2))
  "Resolve the relation according to the pose of the door hinge joint and the
handles neutral and manipulated poses. Diregard any samples inside the door's
opening arc.
CONTAINER-NAME and BTR-ENVIRONMENT are the names of the container and the
environment, in which it can be found, respectively."
  (let* ((handle-link
           (get-handle-link container-name btr-environment))
         (manipulated-handle-pose
           (get-manipulated-pose
            (cl-urdf:name handle-link)
            1
            btr-environment
            :relative T))
         (man-handle-pos-2d
           (cl-transforms:make-3d-vector
            (cl-transforms:x
             (cl-transforms:origin manipulated-handle-pose))
            (cl-transforms:y
             (cl-transforms:origin manipulated-handle-pose))
            0))
         (joint-name
           (cl-urdf:name
            (cl-urdf:child (get-connecting-joint handle-link))))
         (joint-pose
           (get-urdf-link-pose joint-name btr-environment))
         (joint-pos-2d
           (cl-transforms:make-3d-vector
            (cl-transforms:x
             (cl-transforms:origin joint-pose))
            (cl-transforms:y
             (cl-transforms:origin joint-pose))
            0))
         (v2
           (cl-transforms:v- man-handle-pos-2d joint-pos-2d))
         (v2-length
           (sqrt (cl-transforms:dot-product v2 v2))))

    (lambda (x y)
      (let* ((vP (cl-transforms:v- (cl-transforms:make-3d-vector x y 0)
                                   joint-pos-2d))
             (vP-length (sqrt (cl-transforms:dot-product vP vP))))
        (if (and (< vP-length (+ v2-length padding))
                 T)
            0
            1)))))

(defun make-opened-door-for-opposite-arm-cost-function (container-name
                                                        btr-environment
                                                        arm)
  "Resolve the relation according to the pose of the door hinge joint and the
handles neutral and manipulated poses. Disregard any samples in tha angle between
the closed and half open state of the door.
CONTAINER-NAME and BTR-ENVIRONMENT are the names of the container and the
environment, in which it can be found, respectively."
  (let* ((handle-link
           (get-handle-link container-name btr-environment))
         (neutral-handle-pose
           (get-manipulated-pose
            (cl-urdf:name handle-link)
            0
            btr-environment
            :relative T))
         (manipulated-handle-pose
           (get-manipulated-pose
            (cl-urdf:name handle-link)
            0.8
            btr-environment
            :relative T))
         (neutral-handle-pos-2d
           (cl-transforms:make-3d-vector
            (cl-transforms:x
             (cl-transforms:origin neutral-handle-pose))
            (cl-transforms:y
             (cl-transforms:origin neutral-handle-pose))
            0))
         (man-handle-pos-2d
           (cl-transforms:make-3d-vector
            (cl-transforms:x
             (cl-transforms:origin manipulated-handle-pose))
            (cl-transforms:y
             (cl-transforms:origin manipulated-handle-pose))
            0))
         (joint-name
           (cl-urdf:name
            (cl-urdf:child (get-connecting-joint handle-link))))
         (joint-pose
           (get-urdf-link-pose joint-name btr-environment))
         (joint-pos-2d
           (cl-transforms:make-3d-vector
            (cl-transforms:x
             (cl-transforms:origin joint-pose))
            (cl-transforms:y
             (cl-transforms:origin joint-pose))
            0))
         (v1 (cl-transforms:v- neutral-handle-pos-2d joint-pos-2d))
         (v2 (cl-transforms:v- man-handle-pos-2d joint-pos-2d))
         (angle-full (acos (angle-between-vectors v1 v2))))

    (lambda (x y)
      (let* ((vP (cl-transforms:v- (cl-transforms:make-3d-vector x y 0)
                                    joint-pos-2d))
             (angle-to-v1 (acos (angle-between-vectors v1 vP)))
             (angle-to-v2 (acos (angle-between-vectors v2 vP)))
             (v-PtoJ (cl-transforms:v-
                      joint-pos-2d
                      (cl-transforms:make-3d-vector x y 0)))
             (v-PtoH (cl-transforms:v-
                      neutral-handle-pos-2d
                      (cl-transforms:make-3d-vector x y 0)))
             (opens-from (v-which-side v-PtoJ v-PtoH)))
        (if (and
             (equal arm opens-from)
             (< angle-to-v1 angle-full)
             (< angle-to-v2 angle-full))
            0
            1)))))

(defun point-to-point-direction (x y pos1 pos2)
  "Takes an X and Y coordinate, but ignores them, and returns a quaternion
to face from `pos1' towards `pos2'."
  (declare (ignore x y))
  (let* ((point1 (etypecase pos1
                   (cl-transforms:pose (cl-transforms:origin pos1))
                   (cl-transforms:3d-vector pos1)))
         (point2 (etypecase pos2
                   (cl-transforms:pose (cl-transforms:origin pos2))
                   (cl-transforms:3d-vector pos2)))
         (p-rel (cl-transforms:v- point2 point1)))
    (atan (cl-transforms:y p-rel) (cl-transforms:x p-rel))))

(defun make-point-to-point-generator (pos1 pos2 &key (samples 1) sample-step)
  "Returns a function that takes an X and Y coordinate and returns a lazy-list of
quaternions to face from `pos1' to `pos2'."
  (location-costmap:make-orientation-generator
   (alexandria:rcurry
    #'point-to-point-direction
    pos1
    pos2)
   :samples samples
   :sample-step sample-step))

(defun angle-halfway-to-point-direction (x y pos1 pos2 target-pos)
  "Takes an X and Y coordinate and returns a quaternion between the one facing
from pos1 to pos2 and the one facing from (X,Y) to target-pos."
  (let ((pos-direction (point-to-point-direction 0 0 pos1 pos2))
        (target-direction (costmap::angle-to-point-direction x y target-pos)))
    (/ (+ pos-direction target-direction) 2)))

(defun make-angle-halfway-to-point-generator (pos1 pos2 target-pos &key (samples 1) sample-step)
  "Returns a function that takes an X and Y coordinate and returns a lazy-list of
quaternions facing from (X,Y) halfway to target-pos. Meaning pos1 and pos2
describe a direction and the quaternion is the angle between one facing directly
from (X,Y) to target-pos and the direction between pos1 and pos2."
  (location-costmap:make-orientation-generator
   (alexandria:rcurry #'angle-halfway-to-point-direction
                      pos1 pos2 target-pos)
   :samples samples
   :sample-step sample-step))

(defun middle-pose (pose1 pose2)
  "Take two poses and return a pose in the middle of them.
Disregarding the orientation (using the pose2's)."
  (let ((translation-to-middle (cl-transforms:v*
                                (cl-transforms:translation
                                 (cl-transforms:transform-diff
                                  (cl-transforms:pose->transform pose1)
                                  (cl-transforms:pose->transform pose2)))
                                0.5)))
    (cram-tf:translate-pose
     pose2
     :x-offset (cl-transforms:x translation-to-middle)
     :y-offset (cl-transforms:y translation-to-middle)
     :z-offset (cl-transforms:z translation-to-middle))))

(defmethod costmap:costmap-generator-name->score
    ((name (eql 'poses-reachable-cost-function))) 10)

(defmethod costmap:costmap-generator-name->score
    ((name (eql 'container-handle-reachable-cost-function))) 10)

(defmethod costmap:costmap-generator-name->score
    ((name (eql 'opened-drawer-cost-function))) 10)

(defmethod costmap:costmap-generator-name->score
    ((name (eql 'opened-drawer-side-cost-function))) 10)

(defmethod costmap:costmap-generator-name->score
    ((name (eql 'opened-door-cost-function))) 10)

(defmethod costmap:costmap-generator-name->score
    ((name (eql 'opened-door-for-opposite-arm-cost-function))) 10)


(def-fact-group environment-manipulation-costmap (costmap:desig-costmap)
  (<- (costmap:desig-costmap ?designator ?costmap)
    (cram-robot-interfaces:reachability-designator ?designator)
    (spec:property ?designator (:object ?container-designator))
    (spec:property ?container-designator (:type ?container-type))
    (man-int:object-type-subtype :container-prismatic ?container-type)
    (spec:property ?container-designator (:urdf-name ?container-name))
    (spec:property ?container-designator (:part-of ?btr-environment))
    (spec:property ?designator (:arm ?arm))
    (costmap:costmap ?costmap)
    ;; reachability gaussian costmap
    (lisp-fun get-handle-min-max-pose ?container-name ?btr-environment ?poses)
    (lisp-fun costmap:2d-pose-covariance ?poses 0.05 (?mean ?covariance))
    (costmap:costmap-add-function
     container-handle-reachable-cost-function
     (costmap:make-gauss-cost-function ?mean ?covariance)
     ?costmap)
    ;; cutting out drawer costmap
    (costmap:costmap-manipulation-padding ?padding)
    (costmap:costmap-add-function
     opened-drawer-cost-function
     (make-opened-drawer-cost-function ?container-name ?btr-environment ?padding)
     ?costmap)
    ;; cutting out for specific arm costmap
    (costmap:costmap-add-function
     opened-drawer-side-cost-function
     (make-opened-drawer-side-cost-function ?container-name ?arm ?btr-environment)
     ?costmap)
    ;; orientation generator
    ;; generate an orientation opposite to the axis of the drawer
    (equal ?poses (?neutral-pose ?manipulated-pose))
    (costmap:orientation-samples ?samples)
    (costmap:orientation-sample-step ?sample-step)
    (costmap:costmap-add-orientation-generator
     (make-angle-halfway-to-point-generator
      ?manipulated-pose
      ?neutral-pose
      ?neutral-pose
      :samples ?samples
      :sample-step ?sample-step)
     ?costmap))

  (<- (costmap:desig-costmap ?designator ?costmap)
    (cram-robot-interfaces:reachability-designator ?designator)
    (spec:property ?designator (:object ?container-designator))
    (spec:property ?container-designator (:type ?container-type))
    (man-int:object-type-subtype :container-revolute ?container-type)
    (spec:property ?container-designator (:urdf-name ?container-name))
    (spec:property ?container-designator (:part-of ?btr-environment))
    (spec:property ?designator (:arm ?arm))
    (costmap:costmap ?costmap)

    ;; reachability gaussian costmap
    (lisp-fun get-handle-min-max-pose ?container-name ?btr-environment (?min-pose ?max-pose))
    (lisp-fun middle-pose ?min-pose ?max-pose ?middle-pose)
    ;; TODO(cpo): If you can, please beautifiy this.
    (equal (?middle-pose) ?poses)
    (lisp-fun costmap:2d-pose-covariance ?poses 0.05 (?mean ?covariance))
    (costmap:costmap-add-function
     container-handle-reachable-cost-function
     (costmap:make-gauss-cost-function ?mean ?covariance)
     ?costmap)

    ;; cutting out door costmap
    (costmap:costmap-manipulation-padding ?padding)
    (costmap:costmap-add-function
     opened-door-cost-function
     (make-opened-door-cost-function ?container-name ?btr-environment ?padding)
     ?costmap)

    ;; cutting out for specific arm
    (costmap:costmap-add-function
     opened-door-for-opposite-arm-cost-function
     (make-opened-door-for-opposite-arm-cost-function ?container-name ?btr-environment ?arm)
     ?costmap)

    ;; orientate towards the door
    (lisp-fun get-container-link ?container-name ?btr-environment ?link)
    (lisp-fun get-connecting-joint ?link ?joint)
    (lisp-fun cl-urdf:child ?joint ?joint-link)
    (lisp-fun cl-urdf:name ?joint-link ?joint-name)
    (lisp-fun get-urdf-link-pose ?joint-name ?btr-environment ?joint-pose)
    (costmap:orientation-samples ?samples)
    (costmap:orientation-sample-step ?sample-step)
    (costmap:costmap-add-orientation-generator
     (costmap:make-angle-to-point-generator
      ?joint-pose
      :samples ?samples
      :sample-step ?sample-step)
     ?costmap)
    )
  )
