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

(defun look-at-pose-stamped (pose-stamped)
  (declare (type cl-transforms-stamped:pose-stamped pose-stamped))
  (let* ((bindings
           (car
            (prolog:prolog
             '(and (cram-robot-interfaces:robot ?robot)
               (cram-robot-interfaces:robot-pan-tilt-links ?robot ?pan-link ?tilt-link)
               (cram-robot-interfaces:robot-pan-tilt-joints ?robot ?pan-joint ?tilt-joint)
               (cram-robot-interfaces:joint-lower-limit ?robot ?pan-joint ?pan-lower)
               (cram-robot-interfaces:joint-upper-limit ?robot ?pan-joint ?pan-upper)
               (cram-robot-interfaces:joint-lower-limit ?robot ?tilt-joint ?tilt-lower)
               (cram-robot-interfaces:joint-upper-limit ?robot ?tilt-joint ?tilt-upper)))))
         (pan-link
           (cut:var-value '?pan-link bindings))
         (tilt-link
           (cut:var-value '?tilt-link bindings))
         (pan-joint
           (cut:var-value '?pan-joint bindings))
         (tilt-joint
           (cut:var-value '?tilt-joint bindings))
         (pan-lower-limit
           (cut:var-value '?pan-lower bindings))
         (pan-upper-limit
           (cut:var-value '?pan-upper bindings))
         (tilt-lower-limit
           (cut:var-value '?tilt-lower bindings))
         (tilt-upper-limit
           (cut:var-value '?tilt-upper bindings))
         (pose-in-world
           (cram-tf:ensure-pose-in-frame
            pose-stamped
            cram-tf:*fixed-frame*
            :use-zero-time t))
         (pan-tilt-angles
           (btr:calculate-pan-tilt (btr:get-robot-object) pan-link tilt-link pose-in-world))
         (pan-angle
           (first pan-tilt-angles))
         (tilt-angle
           (second pan-tilt-angles))
         (cropped-pan-angle
           (if (< pan-angle pan-lower-limit)
               pan-lower-limit
               (if (> pan-angle pan-upper-limit)
                   pan-upper-limit
                   pan-angle)))
         (cropped-tilt-angle
           (if (< tilt-angle tilt-lower-limit)
               tilt-lower-limit
               (if (> tilt-angle tilt-upper-limit)
                   tilt-upper-limit
                   tilt-angle))))
    (prolog:prolog
     `(and (btr:bullet-world ?w)
           (cram-robot-interfaces:robot ?robot)
           (btr:%object ?w ?robot ?robot-object)
           (assert ?world
                   (btr:joint-state
                    ?robot ((,pan-joint ,cropped-pan-angle)
                            (,tilt-joint ,cropped-tilt-angle))))))
    (unless (and (= pan-angle cropped-pan-angle)
                 (= tilt-angle cropped-tilt-angle))
        (cpl:fail 'common-fail:ptu-goal-unreachable
                  :description "Look action wanted to twist the neck"))
    ;; (cram-occasions-events:on-event
    ;;  (make-instance 'cram-plan-occasions-events:robot-state-changed))
    ))

(defgeneric look-at (goal-type pose-or-frame-or-direction)
  (:method (goal-type (pose cl-transforms-stamped:pose-stamped))
    (look-at-pose-stamped pose))
  (:method (goal-type (frame string))
    (look-at-pose-stamped
     (cl-transforms-stamped:make-pose-stamped
      frame
      0.0
      (cl-transforms:make-identity-vector)
      (cl-transforms:make-identity-rotation))))
  (:method (goal-type (direction symbol))
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
        (assert ?world (btr:joint-state ?robot ((?joint ,(case action-type
                                                           (:open '?max-limit)
                                                           ((:close :grip) '?min-limit)
                                                           (t (if (numberp action-type)
                                                                  (* action-type 5.0)
                                                                  ;; commanded with meters
                                                                  ;; but asserted with rads
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
      (cpl:fail 'common-fail:gripping-failed :description "There was no object to grip"))))

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

(defparameter *gripper-length* 0.2 "PR2's gripper length in meters, for calculating TCP -> EE")

(defun arm-pose-hash-code (arm-pose-list)
  (let* ((pose (second arm-pose-list))
         (pose-list (cram-tf:pose->flat-list pose))
         (sum (abs (apply #'+ pose-list)))
         (sum-big-precise-num (* sum 100000000))
         (pose-hash-code (floor sum-big-precise-num))
         (arm (first arm-pose-list))
         (overall-hash-code (ecase arm
                              (:left (+ pose-hash-code 10000))
                              (:right (+ pose-hash-code 20000)))))
    overall-hash-code))
(defun arm-poses-equal-accurate (arm-pose-list-1 arm-pose-list-2)
  (let* ((pose-1 (second arm-pose-list-1))
         (pose-2 (second arm-pose-list-2))
         (arm-1 (first arm-pose-list-1))
         (arm-2 (first arm-pose-list-2)))
    (if (eql arm-1 arm-2)
        (cram-tf:poses-equal-p pose-1 pose-2 0.000001d0 0.000010d0)
        nil)))
(sb-ext:define-hash-table-test arm-poses-equal-accurate arm-pose-hash-code)
(defvar *ik-solution-cache* (make-hash-table :test 'arm-poses-equal-accurate))

(defun move-tcp (left-tcp-pose right-tcp-pose)
  (declare (type (or cl-transforms-stamped:pose-stamped null) left-tcp-pose right-tcp-pose))
  (flet ((tcp-pose->ee-pose (tcp-pose)
           (when tcp-pose
             (cl-transforms-stamped:pose->pose-stamped
              (cl-transforms-stamped:frame-id tcp-pose)
              (cl-transforms-stamped:stamp tcp-pose)
              (cl-transforms:transform-pose
               (cl-transforms:pose->transform tcp-pose)
               (cl-transforms:make-pose
                (cl-transforms:make-3d-vector (- *gripper-length*) 0 0)
                (cl-transforms:make-identity-rotation))))))
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
                   (let ((hashed-result
                           (gethash (list arm ee-pose) *ik-solution-cache*)))
                     (if hashed-result
                         (values (first hashed-result) (second hashed-result))
                         (call-ik-service-with-torso-resampling
                          arm ee-pose
                          :torso-angle ?torso-angle
                          :torso-lower-limit ?lower-limit
                          :torso-upper-limit ?upper-limit
                          ;; seed-state ; is todo
                          ))))
               (unless ik-solution-msg
                 (cpl:fail 'common-fail:manipulation-pose-unreachable
                           :description (format nil "~a is unreachable for EE." ee-pose)))
               (setf (gethash (list arm ee-pose) *ik-solution-cache*)
                     (list ik-solution-msg torso-angle))
               (values
                (map 'list #'identity
                     (roslisp:msg-slot-value ik-solution-msg 'sensor_msgs-msg:position))
                torso-angle)))))
    (multiple-value-bind (left-ik left-torso-angle)
        (get-ik-joint-positions :left
                                (ee-pose-in-base->ee-pose-in-torso
                                 (tcp-pose->ee-pose left-tcp-pose)))
      (multiple-value-bind (right-ik right-torso-angle)
          (get-ik-joint-positions :right
                                  (ee-pose-in-base->ee-pose-in-torso
                                   (tcp-pose->ee-pose right-tcp-pose)))
        (cond
          ((and left-torso-angle right-torso-angle)
           (when (not (eq left-torso-angle right-torso-angle))
             (cpl:fail 'common-fail:manipulation-pose-unreachable
                       :description (format nil "In MOVE-TCP goals for the two arms ~
                                                 require different torso angles).")))
           (move-torso left-torso-angle))
          (left-torso-angle (move-torso left-torso-angle))
          (right-torso-angle (move-torso right-torso-angle)))
        (move-joints left-ik right-ik)))))

(defun move-with-constraints (constraints-string)
  (declare (ignore constraints-string))
  (warn "Moving with constraints is not supported in projection! Ignoring."))




