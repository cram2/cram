;;; Copyright (c) 2012, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :pr2-manipulation-process-module)

(defstruct assignable-entity-list
  entities min-assignments max-assignments)

(defun list-rotate-left (list &key (rotations 1))
  "Rotates a list `list' left `rotations' times."
  (cond ((= rotations 1)
         (append (rest list) (list (first list))))
        (t
         (list-rotate-left
          (append (rest list) (list (first list)))
          :rotations (1- rotations)))))

(defun rotatable-lists-equal-p (list1 list2)
  "Checks if two double nested lists are equal. The contents of the
second list `list2' are rotated in each manner possible to see if the
contents are in any way equal to what `list1' is."
  (let* ((rotations
           (loop for set in list2
                 for list2-rotated = (loop for x
                                             from 1 to (length set)
                                           collect (list-rotate-left
                                                    set :rotations x))
                 collect list2-rotated))
         (rotated-sets (loop for x from 0 below (length (first rotations))
                             collect (loop for set in rotations
                                           collect (nth x set)))))
    (not (equal (position T (mapcar (lambda (x)
                                      (equal x list1))
                                    rotated-sets)) nil))))

(defun combine-lists-grouped (list)
  "Creates a list of results similar to a cross-product, except that
each block of combinations is grouped by the current element of the
respective outermost list. `list' is a list of lists to be combined."
  (let ((list-first (first list))
        (list-rest (rest list)))
    (loop for x in list-first
          when (= (length list) 1)
            collect (list x)
          when (> (length list) 1)
            collect (loop for y in (combine-lists-grouped list-rest)
                          collect (cons x y)))))

(defun permutation (list)
  "Creates all permutations of the items in list `list'."
  (let ((result nil))
    (alexandria:map-permutations (lambda (permutation)
                                   (push permutation result))
                                 list)
    result))

(defun entity-assignment (assignment-entity-lists)
  "Creates all valid assignments from the `assignment-entity-lists'
lists. Minimum and maximum assignments of each entity type can be
given in the list (which consists of structs of type
`assignable-entity-list'). A list of all possible combinations that
satisfy these constraints is returned."
  (let* ((assignment-sizes
           (loop for assignment-entity-list in assignment-entity-lists
                 for entity-list = (assignable-entity-list-entities
                                    assignment-entity-list)
                 maximizing (or (assignable-entity-list-min-assignments
                                 assignment-entity-list)
                                0) into min-assignments
                 minimizing (or (assignable-entity-list-max-assignments
                                 assignment-entity-list)
                                (length entity-list)) into max-assignments
                 finally (return (list min-assignments max-assignments))))
         (smallest-min-assignments (first assignment-sizes))
         (biggest-max-assignments (second assignment-sizes))
         (all-lists-in-range T))
    (loop for assignment-entity-list in assignment-entity-lists
          for entity-list = (assignable-entity-list-entities
                             assignment-entity-list)
          for list-length = (length entity-list)
          when (< list-length smallest-min-assignments)
            do (setf all-lists-in-range nil)
          when (< list-length biggest-max-assignments)
            do (setf biggest-max-assignments list-length))
    (when all-lists-in-range
      (let* ((permutated-lists
               (loop for assignment-entity-list in assignment-entity-lists
                     for entity-list = (assignable-entity-list-entities
                                        assignment-entity-list)
                     for permutated-list = (permutation entity-list)
                     for staged-lists = (loop for x in permutated-list
                                              collect (loop for n on x
                                                            collect n))
                     collect (reduce #'append (loop for x in staged-lists
                                                    collect (reduce #'append x)))))
             (unnested-lists (loop for x in permutated-lists
                                   collect (reduce #'append x)))
             (unified-lists (loop for x in unnested-lists
                                  collect (remove-duplicates
                                           x
                                           :test #'equal)))
             (fitted-lists
               (loop for i from smallest-min-assignments
                       to biggest-max-assignments
                     for lists-fitting-length = (loop for list
                                                      in unified-lists
                                                      collect
                                                      (loop for entity in list
                                                            when
                                                            (=
                                                             (length entity) i)
                                                            collect entity))
                     collect lists-fitting-length)))
        (remove-duplicates (reduce #'append
                            (reduce #'append
                             (loop for fitted-list in fitted-lists
                                   collect (combine-lists-grouped fitted-list))))
                           :test #'rotatable-lists-equal-p)))))

(defun publish-pose (pose topic)
  (let* ((pose-stamped
           (case (class-name (class-of pose))
             (cl-transforms:pose (cl-tf-datatypes:pose->pose-stamped "/map" (ros-time) pose))
             (cl-tf-datatypes:pose-stamped
              (cond ((or (string= (cl-tf-datatypes:frame-id pose) "map")
                         (string= (cl-tf-datatypes:frame-id pose) "/map"))
                     pose)
                    (t (cl-tf2:transform-pose
                        *tf2-buffer*
                        :pose pose :target-frame "/map"
                        :timeout cram-roslisp-common:*tf-default-timeout*
                        :use-current-ros-time t))))))
         (pose-stamped-msg (cl-tf2:to-msg pose-stamped)))
    (roslisp:publish
     (roslisp:advertise topic "geometry_msgs/PoseStamped")
     pose-stamped-msg)))
