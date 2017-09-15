;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :pr2-reachability-tests)

;; Tests if all poses that are set as valid in the reachability map
;; are actually valid.
(define-test reachability-valid
  (loop for side in '(:left :right)
        for reachability-map = (get-reachability-map side)
        for service-namespace = (cdr (assoc side pr2-reachability-costmap::*arm-namespaces*)) do
          (format t "side: ~a~%" side)
          (loop for z from (cl-transforms:z (minimum reachability-map))
                  below (cl-transforms:z (maximum reachability-map))
                    by (cl-transforms:z (resolution reachability-map))
                do (format t "z: ~a~%" z)
                   (loop for y from (cl-transforms:y (minimum reachability-map))
                           below (cl-transforms:y (maximum reachability-map))
                             by (cl-transforms:y (resolution reachability-map))
                         do (loop for x from (cl-transforms:x (minimum reachability-map))
                                    below (cl-transforms:x (maximum reachability-map))
                                      by (cl-transforms:x (resolution reachability-map))
                                  do (loop for orientation in (orientations reachability-map)
                                           for pose = (cl-transforms:make-pose
                                                       (cl-transforms:make-3d-vector x y z)
                                                       orientation)
                                           when (pose-reachable-p reachability-map pose) do
                                             (assert-true
                                              (find-ik-solution
                                               :pose pose :service-namespace service-namespace))))))))

(defun origin-reachable-p (reachability-map robot-point goal-orientation)
  (some (lambda (orientation)
          (let ((pose (cl-transforms:transform-pose
                       (cl-transforms:transform-inv
                        (cl-transforms:make-transform
                         robot-point orientation))
                       (cl-transforms:make-pose
                        (cl-transforms:make-identity-vector)
                        goal-orientation))))
            (pose-reachable-p reachability-map pose :use-closest-orientation t)))
        (orientations reachability-map)))

(define-test inverse-reachability-reachable
  (loop for side in '(:left :right)
        for reachability-map = (get-reachability-map side)
        for origin = (inverse-map-origin reachability-map)
        for resolution = (resolution reachability-map)
        for matrix = (inverse-reachability-map reachability-map) do
          (format t "side: ~a:~%" side)
          (loop for z-index from 0 below (array-dimension matrix 0)
                for z from (cl-transforms:z (inverse-map-origin reachability-map))
                  by (cl-transforms:z (resolution reachability-map))
                do (format t "z: ~a~%" z)
                   (loop for y-index from 0 below (array-dimension matrix 1)
                         for y from (cl-transforms:y (inverse-map-origin reachability-map))
                           by (cl-transforms:y resolution)
                         do (loop for x-index from 0 below (array-dimension matrix 2)
                                  for x from (cl-transforms:x (inverse-map-origin reachability-map))
                                    by (cl-transforms:z resolution)
                                  do (loop for orientation in (orientations reachability-map)
                                           for orientation-index from 0 do
                                             (if (eql (aref matrix z-index y-index x-index
                                                            orientation-index) 1)
                                                 (assert-true (origin-reachable-p
                                                               reachability-map
                                                               (cl-transforms:make-3d-vector x y z)
                                                               orientation))
                                                 (assert-false (origin-reachable-p
                                                                reachability-map
                                                                (cl-transforms:make-3d-vector x y z)
                                                                orientation)))))))))
