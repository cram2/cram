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

(in-package :pr2-em)

(defun get-handle-min-max-pose (container-name btr-environment)
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
  (btr:aabb
   (btr:rigid-body
    (btr:object btr:*current-bullet-world* btr-environment)
    (btr::make-rigid-body-name
     (string-upcase btr-environment)
     (roslisp-utilities:rosify-underscores-lisp-name container-name)))))

(defun get-width (container-name direction btr-environment)
  "Return the width of the container with name `container-name',
appropriate for the joint `direction' given."
  (let ((dimensions (cl-bullet:bounding-box-dimensions (get-aabb container-name btr-environment)))
        (norm-direction (cl-transforms:normalize-vector direction)))
    (if (> (abs (cl-transforms:x norm-direction))
           (abs (cl-transforms:y norm-direction)))
        (cl-transforms:y dimensions)
        (cl-transforms:x dimensions))))

;; NOTE(cpo): It might be useful to pass this the desired position
;; and calculate the current one to make the costmap more precise.
(defun make-opened-drawer-cost-function (container-name btr-environment &optional (padding 0.2))
  "Resolve the relation according to the poses of the handle of `container-name'
in neutral and manipulated form."
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

(defun make-opened-drawer-side-cost-function (container-name arm btr-environment)
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

(defun make-opened-door-cost-function (container-name btr-environment &optional (padding 0.2))
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
         ;; TODO(cpo): Has to be the door joint, because above it theres the fridge main
         (joint-name (cl-urdf:name
                      (cl-urdf:child (get-connecting-joint handle-link))))
         (joint-pose (get-urdf-link-pose joint-name
                                         btr-environment))
         (joint-pos-2d (cl-transforms:make-3d-vector
                            (cl-transforms:x
                             (cl-transforms:origin joint-pose))
                            (cl-transforms:y
                             (cl-transforms:origin joint-pose))
                            0))
         (handle-pos-2d (cl-transforms:make-3d-vector
                         (cl-transforms:x
                          (cl-transforms:origin handle-pose))
                         (cl-transforms:y
                          (cl-transforms:origin handle-pose))
                         0))
         (man-handle-pos-2d (cl-transforms:make-3d-vector
                             (cl-transforms:x
                              (cl-transforms:origin manipulated-handle-pose))
                             (cl-transforms:y
                              (cl-transforms:origin manipulated-handle-pose))
                             0))
         (v1 (cl-transforms:v- handle-pos-2d
                               joint-pos-2d))
         (v2 (cl-transforms:v- man-handle-pos-2d
                               joint-pos-2d))
         (v1-length (sqrt (cl-transforms:dot-product v1 v1)))
         (v2-length (sqrt (cl-transforms:dot-product v2 v2))))
    
    (lambda (x y)
      (let* ((vP (cl-transforms:v- (cl-transforms:make-3d-vector x y 0)
                                   joint-pos-2d))
             (vP-length (sqrt (cl-transforms:dot-product vP vP))))
        (if (and (< vP-length (+ v2-length 0))
                 T)
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
    (equal ?poses (?pose1 ?pose2))
    (costmap:orientation-samples ?samples)
    (costmap:orientation-sample-step ?sample-step)
    (costmap:costmap-add-orientation-generator
     (make-point-to-point-generator
      ?pose2
      ?pose1
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
    
    (lisp-fun get-handle-min-max-pose ?container-name ?btr-environment ?poses)
    (lisp-fun costmap:2d-pose-covariance ?poses 0.12 (?mean ?covariance))
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
    )
  )
