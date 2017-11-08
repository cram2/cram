;;;
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

(in-package :cram-object-interfaces)

(defgeneric get-object-type-gripping-effort (object-type)
  (:documentation "Returns effort in Nm, e.g. 50."))

(defgeneric get-object-type-gripper-opening (object-type)
  (:documentation "How wide to open the gripper before grasping, in m."))

(defgeneric get-object-type-to-gripper-transform (object-type object-name arm grasp)
  (:documentation "Returns a pose stamped.
Gripper is defined by a convention where Z is pointing towards the object."))

(defgeneric get-object-type-pregrasp-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped"))

(defgeneric get-object-type-2nd-pregrasp-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped. Default value is NIL.")
  (:method (object-type grasp-pose arm grasp) nil))

(defgeneric get-object-type-lift-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped"))

(defgeneric get-object-type-2nd-lift-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped")
  (:method (object-type grasp-pose arm grasp) nil))

(defun get-object-grasping-poses (object-name object-type arm grasp object-transform)
  (declare (type symbol object-name object-type arm grasp)
           (type cl-transforms-stamped:transform-stamped object-transform))
  "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"

  ;; First correct the object transform such that rotationally-symmetric objects
  ;; would not be grasped in an awkward way with weird orientations
  (when (prolog `(object-rotationally-symmetric ,object-type))
    (setf object-transform
          (cram-tf:copy-transform-stamped
           object-transform
           :rotation (cl-transforms:make-identity-rotation))))

  (let* ((gripper-tool-frame
           (ecase arm
             (:left cram-tf:*robot-left-tool-frame*)
             (:right cram-tf:*robot-right-tool-frame*)))
         (object-to-standard-gripper-transform ; oTg'
           (get-object-type-to-gripper-transform object-type object-name arm grasp))
         (standard-to-particular-gripper-transform ; g'Tg
           (cl-tf:transform->transform-stamped
            gripper-tool-frame
            gripper-tool-frame
            0.0
            (cut:var-value
             '?transform
             (car (prolog:prolog
                   `(and (cram-robot-interfaces:robot ?robot)
                         (cram-robot-interfaces:standard-to-particular-gripper-transform
                          ?robot ?transform))))))))
    (when (and object-to-standard-gripper-transform standard-to-particular-gripper-transform)
      (let* ((base-to-standard-gripper-transform
               (cram-tf:multiply-transform-stampeds
                cram-tf:*robot-base-frame* gripper-tool-frame
                object-transform                    ; bTo
                object-to-standard-gripper-transform ; oTg'
                :result-as-pose-or-transform :transform)) ; bTo * oTg' = bTg'
             (grasp-pose ; for particular gripper of particular robot
               (cram-tf:multiply-transform-stampeds ; bTg' * g'Tg = bTg
                cram-tf:*robot-base-frame* gripper-tool-frame
                base-to-standard-gripper-transform      ; bTg'
                standard-to-particular-gripper-transform ; g'Tg
                :result-as-pose-or-transform :pose)))
        (list (get-object-type-pregrasp-pose object-type arm grasp grasp-pose)
              (get-object-type-2nd-pregrasp-pose object-type arm grasp grasp-pose)
              grasp-pose
              (get-object-type-lift-pose object-type arm grasp grasp-pose))))))


(def-fact-group object-knowledge (object-rotationally-symmetric orientation-matters object-type-grasp)

  (<- (object-rotationally-symmetric ?object-type)
    (fail))

  ;; The predicate ORIENTATION-MATTERS holds for all objects where the
  ;; orientation really matters when putting down the object. E.g. for
  ;; knives, forks, etc, the orientation is important while for plates
  ;; the orientation doesn't matter at all.
  (<- (orientation-matters ?object-type-symbol)
    (fail))

  (<- (object-type-grasp ?object-type ?grasp)
    (fail)))
