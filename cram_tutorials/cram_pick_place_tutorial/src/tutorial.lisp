;;;
;;; Copyright (c) 2019, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Amar Fayaz <amar@uni-bremen.de>
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

(in-package :pp-tut)

(defun get-kitchen-urdf ()
  (slot-value (btr:get-environment-object) 'btr:urdf))

(defun move-kitchen-joint (&key (joint-name "iai_fridge_door_joint")
                             (joint-angle 0.2d0))
  (btr:set-robot-state-from-joints
   `((,joint-name  ,joint-angle))
   (btr:get-environment-object)))

(defun spawn-object (spawn-pose &optional (obj-type :bottle) (obj-name 'bottle-1) (obj-color '(1 0 0)))
  (unless (assoc obj-type btr::*mesh-files*)
    (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo"))
  (btr-utils:spawn-object obj-name obj-type :color obj-color :pose spawn-pose)
  (btr:simulate btr:*current-bullet-world* 10))

(defun list-available-objects ()
  (mapcar #'car btr::*mesh-files*))


(defun make-pose (reference-frame pose)
  (cl-transforms-stamped:make-pose-stamped
   reference-frame 0.0
   (apply #'cl-transforms:make-3d-vector (first pose))
   (apply #'cl-transforms:make-quaternion (second pose))))

(defun park-arms ()
  (pp-plans::park-arms))

(defun park-arm (arm)
  (pp-plans::park-arms :arm arm))

(defmacro handle-failure (errors program-body &body error-handler-body)
  `(progn
    (cpl:with-failure-handling
        ((,errors (e)
           (print e)
           ,@error-handler-body))
    ,@program-body)))


(defun list-defined-grasps (object-type)
  (cut:force-ll (cram-object-interfaces::get-object-type-grasps object-type nil nil nil nil)))


(defun visualize-coordinates (object-or-pose &optional (size 0.3))
  (flet ((find-object-of-type (type)
           (find type
                 (remove-if-not (lambda (obj) (typep obj 'btr:item))
                                (btr:objects btr:*current-bullet-world*))
                 :test (lambda (type item) (eql type (car (slot-value item 'btr::types)))))))
    (etypecase object-or-pose
      (symbol
       (if (btr:object btr:*current-bullet-world* object-or-pose)
           (btr:add-vis-axis-object object-or-pose size)
           (if (find-object-of-type object-or-pose)
               (btr:add-vis-axis-object (btr:name (find-object-of-type object-or-pose)) size)
               (warn "Unknown object, please either give an object name or object type."))))
      (cl-transforms-stamped:pose-stamped
       (when (equalp (cl-transforms-stamped:frame-id object-or-pose) "base_footprint")
         (warn "Pose is not in MAP frame. It is visualized with respect to the MAP though."))
       (btr:add-vis-axis-object object-or-pose size))
      (cl-transforms:pose
       (btr:add-vis-axis-object object-or-pose size)))))


(defmethod print-object ((pose cl-transforms-stamped:pose-stamped) stream)
  (print-unreadable-object (pose stream :type nil :identity nil)
    (let ((origin (cl-transforms:origin pose))
          (orientation (cl-transforms:orientation pose)))
     (format stream
             "~s ((~,5f ~,5f ~,5f) (~,5f ~,5f ~,5f ~,5f))"
             (cl-transforms-stamped:frame-id pose)
             (cl-transforms:x origin) (cl-transforms:y origin) (cl-transforms:z origin)
             (cl-transforms:x orientation)
             (cl-transforms:y orientation)
             (cl-transforms:z orientation)
             (cl-transforms:w orientation)))))

(defmethod print-object ((pose cl-transforms-stamped:transform-stamped) stream)
  (print-unreadable-object (pose stream :type nil :identity nil)
    (let ((origin (cl-transforms:translation pose))
          (orientation (cl-transforms:rotation pose)))
     (format stream
             "~s ~s ((~,5f ~,5f ~,5f) (~,5f ~,5f ~,5f ~,5f))"
             (cl-transforms-stamped:frame-id pose)
             (cl-transforms-stamped:child-frame-id pose)
             (cl-transforms:x origin) (cl-transforms:y origin) (cl-transforms:z origin)
             (cl-transforms:x orientation)
             (cl-transforms:y orientation)
             (cl-transforms:z orientation)
             (cl-transforms:w orientation)))))


(defun inverse-transformation (transformation-matrix)
  (cl-transforms-stamped:transform-inv transformation-matrix))

(defun get-current-pose-of-object (?obj-name)
  (let ((?obj-pose (btr:pose
                    (btr:object btr:*current-bullet-world* ?obj-name))))
    (cl-transforms-stamped:make-pose-stamped
     "map" 0.0
     (cl-transforms:origin ?obj-pose)
     (cl-transforms:orientation ?obj-pose))))

(defun get-robot-transformation ()
  (let ((?robot-pose (btr:pose (btr:get-robot-object))))
    (cl-transforms-stamped:make-transform-stamped
     "map" "base_footprint" 0.0
     (cl-transforms:origin ?robot-pose)
     (cl-transforms:orientation ?robot-pose))))

(defun get-obj-name (perceived-object)
  (desig:desig-prop-value perceived-object :name))

(defun apply-transformation (transformation pose)
  (cl-transforms-stamped:transform transformation pose))

(defun get-x-of-pose (pose-stamped)
  (cl-transforms:x (cl-transforms:origin pose-stamped)))

(defun get-y-of-pose (pose-stamped)
  (cl-transforms:y (cl-transforms:origin pose-stamped)))


(defun look-up-transformation (parent child)
  (cl-transforms-stamped:lookup-transform cram-tf:*transformer* parent child :timeout 2.0))
