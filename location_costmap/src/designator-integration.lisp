;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :location-costmap)

(defparameter *costmap-valid-solution-threshold* 0.20)
(defconstant +costmap-n-samples+ 5)

(defvar *costmap-cache* (tg:make-weak-hash-table :test 'eq :weakness :key))
(defvar *costmap-max-values (tg:make-weak-hash-table :test 'eq :weakness :key))

(defun get-cached-costmap (desig)
  (or (gethash desig *costmap-cache*)
      (setf (gethash desig *costmap-cache*)
            (with-vars-bound (?cm)
                (lazy-car
                 (prolog `(merged-desig-costmap ,desig ?cm)))
              (unless (is-var ?cm)
                ?cm)))))

(defun get-cached-costmap-maxvalue (costmap)
  (or (gethash costmap *costmap-max-values)
      (setf (gethash costmap *costmap-max-values)
            (let ((cm (get-cost-map costmap))
                  (max 0))
              (declare (type cma:double-matrix cm))
              (dotimes (row (cma:height cm) max)
                (dotimes (col (cma:width cm))
                  (when (> (aref cm row col) max)
                    (setf max (aref cm row col)))))))))

(defun robot-current-pose-generator (desig)
  (declare (ignore desig))
  (when (and *tf* (cl-tf:can-transform *tf* :target-frame "/map" :source-frame "/base_footprint"))
    (let* ((robot (cl-tf:lookup-transform
                   *tf* :target-frame "/map" :source-frame "/base_footprint")))
      (list
       (tf:make-pose-stamped
        "/map" (roslisp:ros-time)
        (cl-transforms:translation robot)
        (cl-transforms:rotation robot))))))

(defun location-costmap-generator (desig)
  (flet ((take-closest-pose (poses)
           ;; If we don't have tf available, just return the first of
           ;; the points since we don't have any reference for
           ;; distance measurement.
           (cond ((and *tf*
                       (cl-tf:can-transform
                        *tf* :target-frame "/map" :source-frame "/base_footprint"))
                  (let ((closest (car poses))
                        (dist (cl-transforms:v-dist (cl-transforms:translation
                                                     (cl-tf:lookup-transform
                                                      *tf*
                                                      :target-frame "/map"
                                                      :source-frame "/base_footprint"))
                                                    (cl-transforms:origin (car poses)))))
                    (dolist (p (cdr poses) closest)
                      (let ((new-dist (cl-transforms:v-dist (cl-transforms:translation
                                                             (cl-tf:lookup-transform
                                                              *tf*
                                                              :target-frame "/map"
                                                              :source-frame "/base_footprint"))
                                                            (cl-transforms:origin p))))
                        (when (< new-dist dist)
                          (setf dist new-dist)
                          (setf closest p))))))
                 (t (car poses)))))
    (let ((cm (get-cached-costmap desig)))
      (unless cm
        (return-from location-costmap-generator nil))
      (let ((solutions (costmap-samples cm)))
        (publish-location-costmap cm)
        (lazy-list ((solutions solutions)
                    (generated-poses nil))
          (cond (generated-poses
                 (let ((pose (take-closest-pose generated-poses)))
                   (publish-pose pose)
                   (cont pose solutions (remove pose generated-poses))))
                (t
                 (next (lazy-skip +costmap-n-samples+ solutions)
                       (force-ll (lazy-take +costmap-n-samples+ solutions))))))))))

(defun location-costmap-pose-validator (desig pose)
  (when (typep pose 'cl-transforms:pose)
    (let* ((cm (get-cached-costmap desig))
           (p (cl-transforms:origin pose)))
      (if cm
          (let ((costmap-value (/ (get-map-value
                                   cm
                                   (cl-transforms:x p)
                                   (cl-transforms:y p))
                                  (get-cached-costmap-maxvalue cm))))
            (> costmap-value *costmap-valid-solution-threshold*))
          t))))

(register-location-generator
 15 robot-current-pose-generator
 "We should move the robot only if we really need to move. Try the
 current robot pose as a first solution.")

(register-location-generator
 20 location-costmap-generator
 "This generator uses a location costmap generated by the predicate
\(DESIG-COSTMAP ?desig ?costmap\) to generate an infinite number of
candidate solutions.")

(register-location-validation-function
 20 location-costmap-pose-validator
 "Generates a costmap from the designator and only allows poses that
 have a costmap value above the threshold
 *COSTMAP-VALID-SOLUTION-THRESHOLD*. Please note that this threshold
 is specifying the fraction of the maximal value in the costmap, not
 the direct value returned by GET-MAP-VALUE.")

