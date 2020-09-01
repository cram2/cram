;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :giskard)

(defun make-grasp-bar-goal (arm
                            tip-grasp-axis bar-axis
                            tip-finger-axis bar-perpendicular-axis
                            bar-center
                            bar-length
                            root-link)
  (declare (type keyword arm)
           (type cl-transforms-stamped:vector-stamped
                 tip-grasp-axis bar-axis tip-finger-axis bar-perpendicular-axis)
           (type cl-transforms-stamped:point-stamped bar-center)
           (type string root-link)
           (type number bar-length))
  (make-giskard-goal
   :constraints (make-constraints-vector
                 (make-prefer-base-constraint)
                 (make-align-planes-tool-frame-constraint
                  arm bar-perpendicular-axis tip-finger-axis)
                 (make-grasp-bar-constraint
                  arm root-link tip-grasp-axis bar-axis bar-center bar-length))))

(defun call-grasp-bar-action (&key
                                arm root-link
                                tip-grasp-axis tip-finger-axis
                                bar-axis bar-perpendicular-axis
                                bar-center bar-length
                                action-timeout)
  (declare (type (or keyword list) arm)
           (type (or cl-transforms-stamped:vector-stamped null)
                 tip-grasp-axis bar-axis)
           (type (or cl-transforms-stamped:point-stamped null) bar-center)
           (type (or string null) root-link)
           (type (or number null) bar-length action-timeout))

  (call-action
   :action-goal (print
                 (make-grasp-bar-goal
                  arm
                  tip-grasp-axis bar-axis
                  tip-finger-axis bar-perpendicular-axis
                  bar-center
                  bar-length root-link))
   :action-timeout action-timeout))



#+test-for-grasp-bar-action
(
 (defun call-grasp-fridge-handle-action ()
   (call-grasp-bar-action
    :arm :right
    :tip-grasp-axis (cl-transforms-stamped:make-vector-stamped
                     (ecase arm
                       (:left cram-tf:*robot-left-tool-frame*)
                       (:right cram-tf:*robot-right-tool-frame*))
                     0.0
                     (cl-transforms:make-3d-vector 0 0 1))
    :tip-finger-axis (cl-transforms-stamped:make-vector-stamped
                      (ecase arm
                        (:left cram-tf:*robot-left-tool-frame*)
                        (:right cram-tf:*robot-right-tool-frame*))
                      0.0
                      (cl-transforms:make-3d-vector 0 1 0))
    :bar-axis (cl-transforms-stamped:make-vector-stamped
               "iai_kitchen/iai_fridge_door_handle" 0.0
               (cl-transforms:make-3d-vector 0 0 -1))
    :bar-perpendicular-axis (cl-transforms-stamped:make-vector-stamped
                             "iai_kitchen/iai_fridge_door_handle" 0.0
                             (cl-transforms:make-3d-vector 0 1 0))
    :bar-center (cl-transforms-stamped:make-point-stamped
                 "iai_kitchen/iai_fridge_door_handle" 0.0
                 (cl-transforms:make-3d-vector 0 0 0))
    :bar-length 0.8
    :root-link cram-tf:*odom-frame*))

 (defun call-grasp-dishwasher-handle-action ()
   (call-grasp-bar-action
    :arm :right
    :tip-grasp-axis (cl-transforms-stamped:make-vector-stamped
                     (ecase arm
                       (:left cram-tf:*robot-left-tool-frame*)
                       (:right cram-tf:*robot-right-tool-frame*))
                     0.0
                     (cl-transforms:make-3d-vector 0 0 1))
    :tip-finger-axis (cl-transforms-stamped:make-vector-stamped
                      (ecase arm
                        (:left cram-tf:*robot-left-tool-frame*)
                        (:right cram-tf:*robot-right-tool-frame*))
                      0.0
                      (cl-transforms:make-3d-vector 0 1 0))
    :bar-axis (cl-transforms-stamped:make-vector-stamped
               "iai_kitchen/sink_area_dish_washer_door_handle" 0.0
               (cl-transforms:make-3d-vector 0 1 0))
    :bar-perpendicular-axis (cl-transforms-stamped:make-vector-stamped
                             "iai_kitchen/sink_area_dish_washer_door_handle" 0.0
                             (cl-transforms:make-3d-vector 0 0 1))
    :bar-center (cl-transforms-stamped:make-point-stamped
                 "iai_kitchen/sink_area_dish_washer_door_handle" 0.0
                 (cl-transforms:make-3d-vector 0.0 0 0))
    :bar-length 0.2
    :root-link cram-tf:*odom-frame*))

 (defun call-grasp-dishwasher-tray-handle-action ()
   (call-grasp-bar-action
    :arm :right
    :tip-grasp-axis (cl-transforms-stamped:make-vector-stamped
                     (ecase arm
                       (:left cram-tf:*robot-left-tool-frame*)
                       (:right cram-tf:*robot-right-tool-frame*))
                     0.0
                     (cl-transforms:make-3d-vector 0 0 1))
    :tip-finger-axis (cl-transforms-stamped:make-vector-stamped
                      (ecase arm
                        (:left cram-tf:*robot-left-tool-frame*)
                        (:right cram-tf:*robot-right-tool-frame*))
                      0.0
                      (cl-transforms:make-3d-vector 0 1 0))
    :bar-axis (cl-transforms-stamped:make-vector-stamped
               "iai_kitchen/sink_area_dish_washer_tray_handle_front_side" 0.0
               (cl-transforms:make-3d-vector 0 -1 0))
    :bar-perpendicular-axis (cl-transforms-stamped:make-vector-stamped
                             "iai_kitchen/sink_area_dish_washer_tray_handle_front_side" 0.0
                             (cl-transforms:make-3d-vector 0 0 -1))
    :bar-center (cl-transforms-stamped:make-point-stamped
                 "iai_kitchen/sink_area_dish_washer_tray_handle_front_side" 0.0
                 (cl-transforms:make-3d-vector 0.06 0 -0))
    :bar-length 0.20
    :root-link cram-tf:*odom-frame*))
 )
