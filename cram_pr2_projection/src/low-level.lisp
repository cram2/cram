;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;               2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-proj)

;;;;;;;;;;;;;;;;; NAVIGATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun drive (target)
  (declare (type cl-transforms-stamped:pose-stamped target))
  (assert
   (prolog:prolog
    `(and (cram-robot-interfaces:robot ?robot)
          (btr:bullet-world ?w)
          (btr:assert ?w (btr:object-pose ?robot ,target))))))

;;;;;;;;;;;;;;;;; PTU ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun look-at-pose-stamped (pose-stamped)
  (declare (type cl-transforms-stamped:pose-stamped pose-stamped))
  (assert
   (prolog:prolog
    `(and (btr:bullet-world ?world)
          (cram-robot-interfaces:robot ?robot)
          (btr:head-pointing-at ?world ?robot ,pose-stamped)))))

(defgeneric look-at (pose-or-frame-or-direction)
  (:method ((pose cl-transforms-stamped:pose-stamped))
    (look-at-pose-stamped pose))
  (:method ((frame string))
    (look-at-pose-stamped
     (cl-transforms-stamped:make-pose-stamped
      frame
      0.0
      (cl-transforms:make-identity-vector)
      (cl-transforms:make-identity-rotation))))
  (:method ((direction symbol))
    (look-at-pose-stamped
     (case direction
       (:forward (cl-transforms-stamped:make-pose-stamped
                  cram-tf:*robot-base-frame*
                  0.0
                  (cl-transforms:make-3d-vector 3.0 0.0 1.5)
                  (cl-transforms:make-quaternion 0.0 0.0 0.0 1.0)))
       (t (error 'simple-error
                 :format-control "~a direction is unknown for PR2 projection PTU"
                 :format-arguments direction))))))

;;;;;;;;;;;;;;;;; PERCEPTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun detect (input-designator quantifier)
  (declare (type desig:object-designator input-designator)
           (type keyword quantifier))

  (let* ((object-name (desig:desig-prop-value input-designator :name))
         (object-type (desig:desig-prop-value input-designator :type))

         ;; find all visible objects with name `object-name' and of type `object-type'
         (name-pose-type-lists ; e.g.: ((mondamin-1 :mondamin <pose-1>) (mug-2 :mug <pose-2>))
           (cut:force-ll
            (cut:lazy-mapcar
             (lambda (solution-bindings)
               (cut:with-vars-strictly-bound (?object-name ?object-pose ?object-type)
                   solution-bindings
                 (list ?object-name ?object-pose ?object-type)))
             (prolog:prolog `(and (cram-robot-interfaces:robot ?robot)
                                  (btr:bullet-world ?world)
                                  ,@(when object-name
                                      `((prolog:== ?object-name ,object-name)))
                                  (btr:object ?world ?object-name)
                                  ,@(when object-type
                                      `((prolog:== ?object-type ,object-type)))
                                  (btr:item-type ?world ?object-name ?object-type)
                                  (btr:visible ?world ?robot ?object-name)
                                  (btr:pose ?world ?object-name ?object-pose))))))

         ;; if multiple objects are visible and need to find :an object (not :all objects)
         ;; only take the first result
         (quantified-name-pose-type-lists
           (case quantifier
             (:all name-pose-type-lists)
             ((:a :an) (list (first name-pose-type-lists)))
             (t (error "[PROJECTION DETECT]: Quantifier can only be a/an or all.")))))

    ;; extend the input-designator with the information found through visibility check:
    ;; name & pose & type of the object,
    ;; equate the input-designator to the new output-designator
    (mapcar (lambda (name-pose-type-list)
              (destructuring-bind (name pose type)
                  name-pose-type-list
                (let ((output-designator
                        (desig:extend-designator-properties
                         input-designator
                         `((:type ,type)
                           (:location ,(desig:make-designator :location `((:pose ,pose))))
                           (:name name)))))
                  (setf (slot-value output-designator 'desig:data)
                        (make-instance 'desig:object-designator-data
                          :object-identifier name
                          :pose pose))
                  (desig:equate input-designator output-designator))))
            quantified-name-pose-type-lists)))

;;;;;;;;;;;;;;;;; GRIPPERS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun gripper-action (action-type arm &optional maximum-effort)
  (declare (ignore maximum-effort))
  "Opens or closes the specific gripper."
  (cut:force-ll
   (cut:lazy-mapcar

    (lambda (solution-bindings)
      (prolog:prolog
       `(assert ?world (btr:joint-state ?robot ((?joint ,(case action-type
                                                          (:open '?max-limit)
                                                          ((:close :grip) '?min-limit)
                                                          (t (error "[PROJ GRIP] failed")))))))
       solution-bindings))

    (prolog:prolog
     `(and (cram-robot-interfaces:robot ?robot)
           (cram-robot-interfaces:gripper-joint ?robot ,arm ?joint)
           (cram-robot-interfaces:joint-lower-limit ?robot ?joint ?min-limit)
           (cram-robot-interfaces:joint-upper-limit ?robot ?joint ?max-limit)
           (btr:bullet-world ?world))))))

;;;;;;;;;;;;;;;;; ARMS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun move-joints (left-configuration right-configuration)
  (declare (type list left-configuration right-configuration))
  (flet ((set-configuration (arm joint-values)
           (when joint-values
             (let ((joint-names
                     (cut:var-value
                      '?joints
                      (car (prolog:prolog
                            `(and (cram-robot-interfaces:robot ?robot)
                                  (cram-robot-interfaces:arm-joints ?robot ,arm ?joints)))))))
               (unless (= (length joint-values) (length joint-names))
                 (error "[PROJECTION MOVE-JOINTS] length of joints list is incorrect."))
               (let ((joint-name-value-list (mapcar (lambda (name value)
                                                      (list name (* value 1.0d0)))
                                                    joint-names joint-values)))
                 (prolog:prolog
                  `(and
                    (btr:bullet-world ?world)
                    (cram-robot-interfaces:robot ?robot)
                    (assert ?world (btr:joint-state ?robot ,joint-name-value-list)))))))))
    (set-configuration :left left-configuration)
    (set-configuration :right right-configuration)))

(defun move-tcp (left-tcp-pose right-tcp-pose)
  (declare (type (or cl-transforms-stamped:pose-stamped null) left-tcp-pose right-tcp-pose))
  (flet ((get-ik-joint-positions (arm tcp-pose)
           (when tcp-pose
             (let ((ik-solution-msg (call-ik-service arm tcp-pose ; seed-state ; is todo
                                                     )))
               (unless ik-solution-msg
                 (cpl:fail 'pr2-fail:manipulation-pose-unreachable))
               (map 'list #'identity
                    (roslisp:msg-slot-value ik-solution-msg 'sensor_msgs-msg:position))))))
    (move-joints (get-ik-joint-positions :left left-tcp-pose)
                 (get-ik-joint-positions :right right-tcp-pose))))

(defun move-with-constraints (constraints-string)
  (declare (ignore constraints-string))
  (warn "Moving with constraints is not supported in projection! Ignoring."))




