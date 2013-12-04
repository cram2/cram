;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
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

(in-package :visibility-costmap)

(defun make-semantic-visibility-costmap (objects &key pose robot
                                                   occluding-object invert)
  (declare (ignore objects))
  (let ((min-dist 0.8)
        (max-dist 1.3)
        (from-pose (cond (robot
                          (cut:var-value
                           '?pose
                           (first
                            (cut:force-ll
                             (crs:prolog
                              `(and (btr:bullet-world ?w)
                                    (btr:robot ?robot)
                                    (btr:pose ?w ?robot ?pose)))))))
                         (t pose))))
    (flet ((point-visible-from-pose (point pose-from)
             (crs:prolog
              `(and (btr:bullet-world ?w)
                    (btr:robot ?robot)
                    (btr:assert
                     (btr:object-pose ?w test-obj
                                      ((,(tf:x point)
                                         ,(tf:y point)
                                         ,(tf:z point))
                                       (0 0 0 1))))
                    (btr::visible-from ?w ,pose-from test-obj))))
           (point-visible-from-robot (point)
             (crs:prolog
              `(and (btr:bullet-world ?w)
                    (btr:robot ?robot)
                    (btr:assert
                     (btr:object-pose ?w test-obj
                                      ((,(tf:x point)
                                         ,(tf:y point)
                                         ,(tf:z point))
                                       (0 0 0 1))))
                    (btr:visible ?w ?robot test-obj))))
           (point-occluded-by-object (point object-name)
             (crs:prolog
              `(and (btr:bullet-world ?w)
                    (btr:robot ?robot)
                    (btr:assert
                     (btr:object-pose ?w test-obj
                                      ((,(tf:x point)
                                         ,(tf:y point)
                                         ,(tf:z point))
                                       (0 0 0 1))))
                    (btr:occluding-objects ?w ?robot test-obj ?occ-obj-names)
                    (member ,object-name ?occ-obj-names )))))
      (flet ((generator (costmap-metadata matrix)
               (declare (type cma:double-matrix matrix))
               (let ((res-mult 1))
                 (with-slots (origin-x origin-y width height resolution)
                     costmap-metadata
                   (crs:prolog
                    `(and (btr:bullet-world ?w)
                          (btr:assert
                           (btr:object
                            ?w
                            btr:box test-obj
                            ((0 0 0)
                             (0 0 0 1))
                            :size (,resolution ,resolution ,resolution)
                            :mass 1.0))))
                   (loop for y from origin-y below (- width 1)
                           by (* res-mult resolution)
                         do (loop for x from origin-x below (- height 1)
                                    by (* res-mult resolution)
                                  for x-index = (map-coordinate->array-index
                                                 x resolution origin-x)
                                  for y-index = (map-coordinate->array-index
                                                 y resolution origin-y)
                                  when (and (< x-index 500)
                                            (< y-index 500))
                                    do (when (and (<= (tf:v-dist
                                                       (tf:origin from-pose)
                                                       (tf:make-3d-vector
                                                        x y (tf:z
                                                             (tf:origin
                                                              from-pose))))
                                                      max-dist)
                                                  (>= (tf:v-dist
                                                       (tf:origin from-pose)
                                                       (tf:make-3d-vector
                                                        x y (tf:z
                                                             (tf:origin
                                                              from-pose))))
                                                      min-dist))
                                         (let* ((pt (tf:make-3d-vector
                                                     x y 1.0d0))
                                                (visible
                                                  (cond
                                                    (pose
                                                     ;(format t "asd1~%")
                                                     (point-visible-from-pose
                                                      pt from-pose))
                                                    (occluding-object
                                                     ;(format t "asd2~%")
                                                     (point-occluded-by-object
                                                      pt occluding-object))
                                                    (t
                                                     ;(format t "asd3~%")
                                                     (point-visible-from-robot
                                                      pt))))
                                                (mark (or (and (not invert)
                                                               visible)
                                                          (and invert
                                                               (not visible)))))
                                           (when mark
                                             (setf
                                              (aref matrix y-index x-index)
                                              1.0d0))))))
                   (crs:prolog
                    `(and (btr:bullet-world ?w)
                          (btr:retract
                           (btr:object
                            ?w test-obj))))))
               matrix))
        (make-instance
         'map-costmap-generator
         :generator-function #'generator)))))
