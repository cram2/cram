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

(in-package :pr2-reachability-costmap)

(defparameter *arm-namespaces* `((:left . "/reasoning/pr2_left_arm_kinematics")
                                 (:right . "/reasoning/pr2_right_arm_kinematics")))

(defclass reachability-map ()
  ((side :reader side :initarg :side :type (or :left :right))
   (minimum :reader minimum :initarg :minimum :type cl-transforms:3d-vector)
   (maximum :reader maximum :initarg :maximum :type cl-transforms:3d-vector)
   (resolution :reader resolution :initarg :resolution ::type cl-transforms:3d-vector)
   (angles :reader angles :initarg :angles :type list)
   (reachability-map :reader reachability-map)))

(defmethod initialize-instance :after ((map reachability-map) &key filename)
  (when filename
    (restore-reachability-map map filename)))

(defmethod reachability-map :before ((map reachability-map))
  (unless (slot-boundp map 'reachability-map)
    (with-slots (reachability-map side) map
      (setf (slot-value map 'reachability-map)
            (generate-reachability-map
             map :namespace (cdr (assoc side *arm-namespaces*)))))))

(defun generate-reachability-map (map &key namespace)
  (with-slots (minimum maximum resolution angles) map
    (loop with map = (make-array
                      (list
                       (truncate (- (cl-transforms:z maximum)
                                    (cl-transforms:z minimum))
                                 (cl-transforms:y resolution))
                       (truncate (- (cl-transforms:y maximum)
                                    (cl-transforms:y minimum))
                                 (cl-transforms:y resolution))
                       (truncate (- (cl-transforms:x maximum)
                                    (cl-transforms:x minimum))
                                 (cl-transforms:y resolution))
                       (length angles))
                      :element-type 'bit)
          for z from (cl-transforms:z minimum)
            by (cl-transforms:z resolution)
          for z-index from 0 below (array-dimension map 0)
          do (loop for y from (cl-transforms:y minimum)
                     by (cl-transforms:y resolution)
                   for y-index from 0 below (array-dimension map 1)
                   do (format t "y: ~a z: ~a~%" y z)
                      (loop for x from (cl-transforms:x minimum)
                              by (cl-transforms:x resolution)
                            for x-index from 0 below (array-dimension map 2)
                            do (loop for angle in angles
                                     for angle-index from 0
                                     do (cond ((find-ik-solution
                                                :pose (cl-transforms:make-pose
                                                       (cl-transforms:make-3d-vector x y z)
                                                       angle)
                                                :service-namespace namespace)
                                               (format t ".")
                                               (setf (aref map z-index y-index x-index angle-index) 1))
                                              (t (format t "x")
                                                 (setf (aref map z-index y-index x-index angle-index) 0))))
                            finally (format t "~%")))
          finally (return map))))

(defun store-reachability-map (map filename)
  (let ((reachability-map (reachability-map map)))
    (with-slots (side minimum maximum resolution angles) map
      (with-open-file (file filename
                            :direction :output
                            :element-type 'unsigned-byte
                            :if-exists :supersede)
        (store file side)
        (store file minimum)
        (store file maximum)
        (store file resolution)
        (store file angles)
        (store file reachability-map)))))

(defun restore-reachability-map (map filename)
  (with-slots (side minimum maximum resolution angles reachability-map)
      map
    (with-open-file (file filename :direction :input :element-type 'unsigned-byte)
      (setf side (restore file 'symbol))
      (setf minimum (restore file 'cl-transforms:3d-vector))
      (setf maximum (restore file 'cl-transforms:3d-vector))
      (setf resolution (restore file 'cl-transforms:3d-vector))
      (setf angles (restore file 'list))
      (setf reachability-map (restore file 'array)))))
