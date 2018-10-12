;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;               2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :boxy-proj)

(defun robot-transform-in-map ()
  (let ((pose-in-map
          (cut:var-value
           '?pose
           (car (prolog:prolog
                 `(and (cram-robot-interfaces:robot ?robot)
                       (btr:bullet-world ?w)
                       (btr:object-pose ?w ?robot ?pose)))))))
    (cram-tf:pose->transform-stamped
     cram-tf:*fixed-frame*
     cram-tf:*robot-base-frame*
     (cut:current-timestamp)
     pose-in-map)))

;;;;;;;;;;;;;;;;; NAVIGATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun drive (target)
  (declare (type cl-transforms-stamped:pose-stamped target))
  (assert
   (prolog:prolog
    `(and (cram-robot-interfaces:robot ?robot)
          (btr:bullet-world ?w)
          (btr:assert ?w (btr:object-pose ?robot ,target)))))
  ;; (cram-occasions-events:on-event
  ;;  (make-instance 'cram-plan-occasions-events:robot-state-changed))
  )

;;;;;;;;;;;;;;;;; TORSO ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun move-torso (joint-angle)
  (declare (type number joint-angle))
  (assert
   (prolog:prolog
    `(and (cram-robot-interfaces:robot ?robot)
          (btr:bullet-world ?w)
          (cram-robot-interfaces:robot-torso-link-joint ?robot ?_ ?joint)
          (btr:assert (btr:joint-state ?w ?robot ((?joint ,joint-angle)))))))
  ;; (cram-occasions-events:on-event
  ;;  (make-instance 'cram-plan-occasions-events:robot-state-changed))
  )

;;;;;;;;;;;;;;;;; PTU ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun look-at-joint-angles (joint-angles)
  (declare (type list joint-angles))
  (assert
   (prolog:prolog
    `(and (cram-robot-interfaces:robot ?robot)
          (btr:bullet-world ?w)
          (cram-robot-interfaces:robot-pan-tilt-joints ?robot . ?joint-names)
          (prolog:lisp-fun mapcar list ?joint-names ,joint-angles ?joint-states)
          (btr:assert ?w (btr:joint-state ?robot ?joint-states))))))

(defun look-at-joint-states (joint-states)
  (declare (type list joint-states))
  (assert
   (prolog:prolog
    `(and (cram-robot-interfaces:robot ?robot)
          (btr:bullet-world ?w)
          (btr:assert ?w (btr:joint-state ?robot ,joint-states))))))

(defgeneric look-at (joint-angle-or-state-or-direction)
  (:method ((joint-angles-or-states list))
    (if (typep (car joint-angles-or-states) 'list)
        (look-at-joint-states joint-angles-or-states)
        (look-at-joint-angles joint-angles-or-states)))
  (:method ((direction symbol))
    (look-at-joint-states
     (case direction
       (:away
        (cut:var-value
         '?joints
         (car (prolog:prolog
               `(and (cram-robot-interfaces:robot ?robot)
                     (cram-robot-interfaces:robot-neck-parking-joint-states ?robot ?joints))))))
       (:down
        (cut:var-value
         '?joints
         (car (prolog:prolog
               `(and (cram-robot-interfaces:robot ?robot)
                     (cram-robot-interfaces:robot-neck-looking-joint-states ?robot ?joints))))))
       (t (error 'simple-error
                 :format-control "~a direction is unknown for Boxy projection PTU"
                 :format-arguments direction))))))

;;;;;;;;;;;;;;;;; PERCEPTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; todo: test btr:visible with boxy camera stuff
(defun extend-perceived-object-designator (input-designator name-pose-type-list)
  (destructuring-bind (name pose type) name-pose-type-list
    (let* ((transform-stamped-in-fixed-frame
             (cl-transforms-stamped:make-transform-stamped
              cram-tf:*fixed-frame*
              (roslisp-utilities:rosify-underscores-lisp-name name)
              (cut:current-timestamp)
              (cl-transforms:origin pose)
              (cl-transforms:orientation pose)))
           (pose-stamped-in-base-frame
             (cram-tf:multiply-transform-stampeds
              cram-tf:*robot-base-frame*
              (roslisp-utilities:rosify-underscores-lisp-name name)
              (cram-tf:transform-stamped-inv (robot-transform-in-map))
              transform-stamped-in-fixed-frame
              :result-as-pose-or-transform :pose))
           (transform-stamped-in-base-frame
             (cram-tf:multiply-transform-stampeds
              cram-tf:*robot-base-frame*
              (roslisp-utilities:rosify-underscores-lisp-name name)
              (cram-tf:transform-stamped-inv (robot-transform-in-map))
              transform-stamped-in-fixed-frame
              :result-as-pose-or-transform :transform)))
      (let ((output-designator
              (desig:copy-designator
               input-designator
               :new-description
               `((:type ,type)
                 (:name ,name)
                 (:pose ((:pose ,pose-stamped-in-base-frame)
                         (:transform ,transform-stamped-in-base-frame)))))))
        (setf (slot-value output-designator 'desig:data)
              (make-instance 'desig:object-designator-data
                :object-identifier name
                :pose pose-stamped-in-base-frame))
        ;; (desig:equate input-designator output-designator)

        ;; before returning a freshly made output designator of perceived object
        ;; emit an object perceived event to update the belief state
          ;; (cram-occasions-events:on-event
          ;;  (make-instance 'cram-plan-occasions-events:object-perceived-event
          ;;    :object-designator output-designator
          ;;    :perception-source :projection))

        output-designator))))

(defun detect (input-designator)
  (declare (type desig:object-designator input-designator))

  (let* ((object-name (desig:desig-prop-value input-designator :name))
         (object-type (desig:desig-prop-value input-designator :type))
         (quantifier (desig:quantifier input-designator))

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
                                  (btr:pose ?world ?object-name ?object-pose)))))))

    ;; check if objects were found
    (unless name-pose-type-lists
      (cpl:fail 'common-fail:perception-object-not-found :object input-designator
                :description (format nil "Could not find object ~a." input-designator)))

    ;; Extend the input-designator with the information found through visibility check:
    ;; name & pose & type of the object,
    ;; equate the input-designator to the new output-designator.
    ;; If multiple objects are visible, return multiple equated objects,
    ;; otherwise only take first found object. I.e. need to find :an object (not :all objects)
    (case quantifier
      (:all (mapcar (alexandria:curry #'extend-perceived-object-designator input-designator)
                    name-pose-type-lists))
      ((:a :an) (extend-perceived-object-designator
                 input-designator
                 (first name-pose-type-lists)))
      (t (error "[PROJECTION DETECT]: Quantifier can only be a/an or all.")))))

;;;;;;;;;;;;;;;;; GRIPPERS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun one-gripper-action (action-type arm &optional maximum-effort)
  (declare (ignore maximum-effort))
  "Opens or closes the specific gripper."
  (mapc

   (lambda (solution-bindings)
     (prolog:prolog
      `(and
        (btr:bullet-world ?world)
        (assert ?world (btr:joint-state ?robot
                                        ((?joint ,(case action-type
                                                    (:open '?max-limit)
                                                    ((:close :grip) '?min-limit)
                                                    (t (if (numberp action-type)
                                                           action-type
                                                           (error "[PROJ GRIP] failed")))))))))
      solution-bindings))

   (cut:force-ll
    (prolog:prolog
     `(and (cram-robot-interfaces:robot ?robot)
           (cram-robot-interfaces:gripper-joint ?robot ,arm ?joint)
           (cram-robot-interfaces:joint-lower-limit ?robot ?joint ?min-limit)
           (cram-robot-interfaces:joint-upper-limit ?robot ?joint ?max-limit)))))

  ;; robot-state-changed event
  ;; (cram-occasions-events:on-event
  ;;  (make-instance 'cram-plan-occasions-events:robot-state-changed))

  ;; check if there is an object to grip
  (when (eql action-type :grip) ; if action was gripping check if gripper collided with an item
    (unless (prolog:prolog
             `(and (btr:bullet-world ?world)
                   (cram-robot-interfaces:robot ?robot)
                   (btr:contact ?world ?robot ?object-name ?link)
                   (cram-robot-interfaces:gripper-link ?robot ,arm ?link)
                   (btr:%object ?world ?object-name ?object-instance)
                   (prolog:lisp-type ?object-instance btr:item)))
      (cpl:fail 'common-fail:gripper-closed-completely
                :description "There was no object to grip"))))

(defun gripper-action (action-type arm &optional maximum-effort)
  (if (and arm (listp arm))
      (cpl:par
        (one-gripper-action action-type (first arm) maximum-effort)
        (one-gripper-action action-type (second arm) maximum-effort))
      (one-gripper-action action-type arm maximum-effort)))

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
                 (assert
                  (prolog:prolog
                   `(and
                     (btr:bullet-world ?world)
                     (cram-robot-interfaces:robot ?robot)
                     (assert ?world (btr:joint-state ?robot ,joint-name-value-list))))))))))
    (set-configuration :left left-configuration)
    (set-configuration :right right-configuration)
    ;; (cram-occasions-events:on-event
    ;;  (make-instance 'cram-plan-occasions-events:robot-state-changed))
    ))

(defparameter *tcp-pose-in-ee* (cl-transforms:make-pose
                                (cl-transforms:make-3d-vector 0 0 0.3191d0)
                                (cl-transforms:make-identity-rotation))
  "In meters, Teetcp, EE -> TCP but a Pose")

(defun move-tcp (left-tcp-pose right-tcp-pose)
  (declare (type (or cl-transforms-stamped:pose-stamped null) left-tcp-pose right-tcp-pose))
  (flet ((tcp-pose->ee-pose (tcp-pose tool-frame end-effector-frame)
           (when tcp-pose
             (cram-tf:strip-transform-stamped
              (cram-tf:multiply-transform-stampeds
               cram-tf:*robot-base-frame*
               end-effector-frame
               (cram-tf:pose->transform-stamped
                cram-tf:*robot-base-frame*
                tool-frame
                0.0
                tcp-pose)
               (cram-tf:transform-stamped-inv
                (cram-tf:pose->transform-stamped
                 end-effector-frame
                 tool-frame
                 0.0
                 *tcp-pose-in-ee*))))))
         (ee-pose-in-base->ee-pose-in-torso (ee-pose-in-base)
           (when ee-pose-in-base
             (if (string-equal
                  (cl-transforms-stamped:frame-id ee-pose-in-base)
                  cram-tf:*robot-base-frame*)
                 ;; tPe: tTe = tTb * bTe = tTm * mTb * bTe = (mTt)-1 * mTb * bTe
                 (let* ((map-torso-transform
                          (cram-tf:pose->transform-stamped
                           cram-tf:*fixed-frame*
                           cram-tf:*robot-torso-frame*
                           0.0
                           (btr:link-pose
                            (btr:get-robot-object)
                            cram-tf:*robot-torso-frame*)))
                        (torso-map-transform
                          (cram-tf:transform-stamped-inv map-torso-transform))
                        (map-base-transform
                          (cram-tf:pose->transform-stamped
                           cram-tf:*fixed-frame*
                           cram-tf:*robot-base-frame*
                           0.0
                           (btr:pose (btr:get-robot-object))))
                        (torso-base-transform
                          (cram-tf:multiply-transform-stampeds
                           cram-tf:*robot-torso-frame*
                           cram-tf:*robot-base-frame*
                           torso-map-transform
                           map-base-transform))
                        (base-ee-transform
                          (cram-tf:pose-stamped->transform-stamped
                           ee-pose-in-base
                           ;; dummy link name for T x T to work
                           "end_effector_link")))
                   (cram-tf:multiply-transform-stampeds
                    cram-tf:*robot-torso-frame*
                    "end_effector_link"
                    torso-base-transform
                    base-ee-transform
                    :result-as-pose-or-transform :pose))
                 (error "Arm movement goals should be given in robot base frame"))))
         (get-ik-joint-positions (arm ee-pose)
           (when ee-pose
             (multiple-value-bind (ik-solution-msg torso-angle)
                 (cut:with-vars-bound (?torso-angle ?lower-limit ?upper-limit)
                     (car (prolog:prolog
                           `(and
                             (cram-robot-interfaces:robot ?robot)
                             (cram-robot-interfaces:robot-torso-link-joint ?robot
                                                                           ?_ ?torso-joint)
                             (cram-robot-interfaces:joint-lower-limit ?robot ?torso-joint
                                                                      ?lower-limit)
                             (cram-robot-interfaces:joint-upper-limit ?robot ?torso-joint
                                                                      ?upper-limit)
                             (btr:bullet-world ?world)
                             (btr:joint-state ?world ?robot ?torso-joint ?torso-angle))))
                   (call-ik-service-with-torso-resampling
                    arm ee-pose
                    :torso-angle ?torso-angle
                    :torso-lower-limit ?lower-limit
                    :torso-upper-limit ?upper-limit))
               (unless ik-solution-msg
                 (cpl:fail 'common-fail:manipulation-pose-unreachable
                           :description (format nil "~a is unreachable for EE." ee-pose)))
               (values
                (map 'list #'identity
                     (roslisp:msg-slot-value ik-solution-msg 'sensor_msgs-msg:position))
                torso-angle)))))

    (let* ((frame-bindings
             (cut:lazy-car
              (prolog:prolog
               `(and (cram-robot-interfaces:robot ?robot)
                     (cram-robot-interfaces:robot-tool-frame ?robot :left ?left-tool-frame)
                     (cram-robot-interfaces:robot-tool-frame ?robot :right ?right-tool-frame)
                     (cram-robot-interfaces:end-effector-link ?robot :left ?left-ee-frame)
                     (cram-robot-interfaces:end-effector-link ?robot :right ?right-ee-frame)))))
           (left-tcp-frame
             (cut:var-value '?left-tool-frame frame-bindings))
           (right-tcp-frame
             (cut:var-value '?right-tool-frame frame-bindings))
           (left-ee-frame
             (cut:var-value '?left-ee-frame frame-bindings))
           (right-ee-frame
             (cut:var-value '?right-ee-frame frame-bindings)))

      (multiple-value-bind (left-ik left-torso-angle)
          (get-ik-joint-positions :left
                                  (ee-pose-in-base->ee-pose-in-torso
                                   (tcp-pose->ee-pose left-tcp-pose
                                                      left-tcp-frame left-ee-frame)))
        (multiple-value-bind (right-ik right-torso-angle)
            (get-ik-joint-positions :right
                                    (ee-pose-in-base->ee-pose-in-torso
                                     (tcp-pose->ee-pose right-tcp-pose
                                                        right-tcp-frame right-ee-frame)))
          (cond
            ((and left-torso-angle right-torso-angle)
             (when (not (eq left-torso-angle right-torso-angle))
               (cpl:fail 'common-fail:manipulation-pose-unreachable
                         :description (format nil "In MOVE-TCP goals for the two arms ~
                                                 require different torso angles).")))
             (move-torso left-torso-angle))
            (left-torso-angle (move-torso left-torso-angle))
            (right-torso-angle (move-torso right-torso-angle)))
          (move-joints left-ik right-ik))))))
