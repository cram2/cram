;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;               2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2019, Vanessa Hassouna <hassouna@uni-bremen.de>
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

(in-package :urdf-proj)

(defparameter *debug-short-sleep-duration* 0.0
  "in seconds, sleeps after each movement during reasoning")
(defparameter *debug-long-sleep-duration* 0.0
  "in seconds, sleeps to show colliding configurations")

(defparameter *be-strict-with-collisions* nil
  "when grasping a spoon from table, fingers can collide with kitchen, so we might allow this")

(defun robot-transform-in-map ()
  (let ((pose-in-map
          (cut:var-value
           '?pose
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
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
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))
    (unwind-protect
         (assert
          (prolog:prolog
           `(and (rob-int:robot ?robot)
                 (btr:bullet-world ?w)
                 (btr:assert ?w (btr:object-pose ?robot ,target)))))
      (when (btr:robot-colliding-objects-without-attached '(:floor))
        (unless (< (abs *debug-short-sleep-duration*) 0.0001)
          (cpl:sleep *debug-short-sleep-duration*))
        (btr::restore-world-state world-state world)
        (cpl:fail 'common-fail:navigation-pose-unreachable :pose-stamped target)))))

;;;;;;;;;;;;;;;;; TORSO ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun move-torso (joint-angle)
  (declare (type (or number keyword) joint-angle))
  "Joint-angle can be a number or :UPPER-LIMIT or :LOWER-LIMIT keywords."
  (let* ((bindings
           (car
            (prolog:prolog
             `(and (rob-int:robot ?robot)
                   (btr:bullet-world ?w)
                   (rob-int:robot-torso-link-joint ?robot ?_ ?joint)
                   (rob-int:joint-lower-limit ?robot ?joint ?lower)
                   (rob-int:joint-upper-limit ?robot ?joint ?upper)))))
         (lower-limit
           (cut:var-value '?lower bindings))
         (upper-limit
           (cut:var-value '?upper bindings))
         (cropped-joint-angle
           (if (numberp joint-angle)
               (if (< joint-angle lower-limit)
                      lower-limit
                      (if (> joint-angle upper-limit)
                          upper-limit
                          joint-angle))
               (ecase joint-angle
                 (:upper-limit upper-limit)
                 (:lower-limit lower-limit)
                 (:middle (/ (- upper-limit lower-limit) 2))))))
    (prolog:prolog
     `(btr:assert (btr:joint-state ?w ?robot ((?joint ,cropped-joint-angle))))
     bindings)
    (when (numberp joint-angle)
      (unless (< (abs (- joint-angle cropped-joint-angle)) 0.0001)
        (cpl:fail 'common-fail:torso-goal-not-reached
                  :description (format nil "Torso goal ~a was out of joint limits" joint-angle)
                  :torso joint-angle)))))


;;;;;;;;;;;;;;;;; NECK ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun look-at-joint-angles (joint-angles)
  (declare (type list joint-angles))
  (assert
   (prolog:prolog
    `(and (rob-int:robot ?robot)
          (btr:bullet-world ?w)
          (rob-int:robot-neck-joints ?robot . ?joint-names)
          (prolog:lisp-fun mapcar list ?joint-names ,joint-angles ?joint-states)
          (btr:assert ?w (btr:joint-state ?robot ?joint-states))))))

(defun look-at-joint-states (joint-states)
  (declare (type list joint-states))
  (assert
   (prolog:prolog
    `(and (rob-int:robot ?robot)
          (btr:bullet-world ?w)
          (btr:assert ?w (btr:joint-state ?robot ,joint-states))))))

(defun look-at-pose-stamped-two-joints (pose-stamped)
  (declare (type cl-transforms-stamped:pose-stamped pose-stamped))
  (cut:with-vars-strictly-bound (?pan-link
                                 ?tilt-link
                                 ?pan-joint ?tilt-joint
                                 ?pan-lower-limit ?pan-upper-limit
                                 ?tilt-lower-limit ?tilt-upper-limit)
      (car
       (prolog:prolog
        '(and
          (rob-int:robot ?robot)
          (rob-int:robot-neck-links ?robot ?pan-link ?tilt-link)
          (rob-int:robot-neck-joints ?robot ?pan-joint ?tilt-joint)
          (rob-int:joint-lower-limit ?robot ?pan-joint ?pan-lower-limit)
          (rob-int:joint-upper-limit ?robot ?pan-joint ?pan-upper-limit)
          (rob-int:joint-lower-limit ?robot ?tilt-joint ?tilt-lower-limit)
          (rob-int:joint-upper-limit ?robot ?tilt-joint ?tilt-upper-limit))))

    (let* ((pose-in-world
             (cram-tf:ensure-pose-in-frame
              pose-stamped
              cram-tf:*fixed-frame*
              :use-zero-time t))
           (pan-tilt-angles
             (btr:calculate-pan-tilt (btr:get-robot-object) ?pan-link ?tilt-link pose-in-world))
           (pan-angle
             (first pan-tilt-angles))
           (tilt-angle
             (second pan-tilt-angles))
           (cropped-pan-angle
             (if (< pan-angle ?pan-lower-limit)
                 ?pan-lower-limit
                 (if (> pan-angle ?pan-upper-limit)
                     ?pan-upper-limit
                     pan-angle)))
           (cropped-tilt-angle
             (if (< tilt-angle ?tilt-lower-limit)
                 ?tilt-lower-limit
                 (if (> tilt-angle ?tilt-upper-limit)
                     ?tilt-upper-limit
                     tilt-angle))))

      (prolog:prolog
       `(and (btr:bullet-world ?w)
             (rob-int:robot ?robot)
             (btr:%object ?w ?robot ?robot-object)
             (assert ?world
                     (btr:joint-state
                      ?robot ((,?pan-joint ,cropped-pan-angle)
                              (,?tilt-joint ,cropped-tilt-angle))))))
      (unless (and (< (abs (- pan-angle cropped-pan-angle)) 0.00001)
                   (< (abs (- tilt-angle cropped-tilt-angle)) 0.00001))
        (cpl:fail 'common-fail:ptu-goal-not-reached
                  :description "Look action wanted to twist the neck")))))

(defparameter *camera-pose-unit-vector-multiplyer* 0.4)
(defparameter *camera-pose-z-offset* 0.2)
(defparameter *camera-resampling-step* 0.1)
(defparameter *camera-x-axis-limit* 0.5)
(defparameter *camera-y-axis-limit* 0.5)

(defun get-neck-ik (ee-link cartesian-pose base-link joint-names)
  (let ((joint-state-msg
          (or (ik:call-ik-service-with-resampling
               (cl-tf:pose->pose-stamped
                base-link
                0.0
                cartesian-pose)
               base-link ee-link
               (btr::make-robot-joint-state-msg
                (btr:get-robot-object)
                :joint-names joint-names)
               *camera-resampling-step* :x
               0 (- *camera-x-axis-limit*) *camera-x-axis-limit*)
              (ik:call-ik-service-with-resampling
               (cl-tf:pose->pose-stamped
                base-link
                0.0
                cartesian-pose)
               base-link ee-link
               (btr::make-robot-joint-state-msg
                (btr:get-robot-object)
                :joint-names joint-names)
               *camera-resampling-step* :y
               0 (- *camera-y-axis-limit*) *camera-y-axis-limit*))))
    (when joint-state-msg
      (map 'list #'identity (roslisp:msg-slot-value joint-state-msg :position)))))

(defun calculate-camera-pose-from-object-pose (neck-base-t-object)
  "Takes the vector from neck-base to object, sets its Z to 0,
then normalizes to get a unit vector, then multiplies with a multiplier to make it shorter
 (multiplier should be comparable to maximum length between neck base and camera),
then pulls the vector up in Z a bit to avoid colliding with bottom parts of robot,
and that would be the desired camera position.
Next, to calculate desired camera rotation, looks at the object from camera position,
calculates an angle that would have to be applied around X axis to align camera's Z
with the object, calculates similar angle around Y axis and applies the rotations. "
  (let* ((neck-base-t-object-vector
           (cl-transforms:translation neck-base-t-object))
         (neck-base-t-object-vector-without-z
           (cl-transforms:copy-3d-vector neck-base-t-object-vector :z 0.0))
         (neck-base-t-object-unit-vector
           (cl-transforms:normalize-vector neck-base-t-object-vector-without-z))
         (neck-base-t-object-short-vector
           (cl-transforms:v* neck-base-t-object-unit-vector *camera-pose-unit-vector-multiplyer*))
         (neck-base-t-object-short-vector-lifted
           (cl-transforms:v+ neck-base-t-object-short-vector
                             (cl-transforms:make-3d-vector 0 0 *camera-pose-z-offset*)))

         (neck-base-t-camera-not-oriented
           (cl-transforms:make-transform
            neck-base-t-object-short-vector-lifted
            (cl-transforms:make-quaternion -1 0 0 0)))
         (camera-not-oriented-t-neck-base
           (cl-transforms:transform-inv
            neck-base-t-camera-not-oriented))
         (camera-not-oriented-t-object
           (cl-transforms:transform*
            camera-not-oriented-t-neck-base
            neck-base-t-object))
         (rotation-angle-around-x
           (- (atan
               (cl-transforms:y (cl-transforms:translation camera-not-oriented-t-object))
               (cl-transforms:z (cl-transforms:translation camera-not-oriented-t-object)))))
         (neck-base-t-camera-rotated-around-x
           (cram-tf:rotate-transform-in-own-frame
            neck-base-t-camera-not-oriented
            :x rotation-angle-around-x))
         (camera-rotated-around-x-t-neck-base
           (cl-transforms:transform-inv
            neck-base-t-camera-rotated-around-x))
         (camera-rotated-around-x-t-object
           (cl-transforms:transform*
            camera-rotated-around-x-t-neck-base
            neck-base-t-object))
         (rotation-angle-around-y
           (atan
            (cl-transforms:x (cl-transforms:translation camera-rotated-around-x-t-object))
            (cl-transforms:z (cl-transforms:translation camera-rotated-around-x-t-object))))
         (neck-base-t-camera
           (cram-tf:rotate-transform-in-own-frame
            neck-base-t-camera-rotated-around-x
            :y rotation-angle-around-y)))
    neck-base-t-camera))

(defun look-at-pose-stamped-many-joints (object-pose)
  (declare (type cl-transforms-stamped:pose-stamped object-pose))
  (let* ((map-p-object
           (cram-tf:ensure-pose-in-frame object-pose cram-tf:*fixed-frame* :use-zero-time t))
         (bindings
           (cut:lazy-car
            (prolog:prolog
             `(and (rob-int:robot ?robot)
                   (rob-int:robot-neck-links ?robot . ?neck-frames)
                   (rob-int:robot-neck-joints ?robot . ?neck-joints)
                   (rob-int:robot-neck-base-link ?robot ?neck-base-frame)
                   (rob-int:camera-in-neck-ee-pose ?robot ?neck-ee-p-cam)))))
         (neck-ee-frame
           (car (last (cut:var-value '?neck-frames bindings))))
         (neck-joints
           (cut:var-value '?neck-joints bindings))
         (neck-base-frame
           (cut:var-value '?neck-base-frame bindings))
         (neck-ee-p-camera
           (cut:var-value '?neck-ee-p-cam bindings))

         (map-p-neck-base
           (btr:link-pose (btr:get-robot-object) neck-base-frame))
         (neck-base-t-map
           (cl-transforms:transform-inv
            (cl-transforms:pose->transform map-p-neck-base)))
         (neck-base-t-object
           (cl-transforms:transform*
            neck-base-t-map
            (cl-transforms:pose->transform map-p-object)))

         (neck-base-t-camera
           (calculate-camera-pose-from-object-pose neck-base-t-object))

         (camera-t-neck-ee
           (cl-transforms:transform-inv (cl-transforms:pose->transform neck-ee-p-camera)))
         (neck-base-t-neck-ee
           (cl-transforms:transform* neck-base-t-camera camera-t-neck-ee))
         (neck-base-p-neck-ee
           (cl-transforms:transform->pose neck-base-t-neck-ee))

         (joint-state
           (get-neck-ik neck-ee-frame neck-base-p-neck-ee neck-base-frame neck-joints)))

    (if joint-state
        (look-at-joint-angles joint-state)
        (cpl:fail 'common-fail:ptu-goal-not-reached
                  :description "Look goal was not reachable"))))


(defun look-at (pose configuration)
  (let* ((neck-joints-num
           (length (cut:var-value
                    '?joint-names
                    (car
                     (prolog:prolog
                      `(and (rob-int:robot ?robot)
                            (btr:bullet-world ?w)
                            (rob-int:robot-neck-joints
                             ?robot . ?joint-names))))))))
    (if pose
        (if (= neck-joints-num 2)
            (look-at-pose-stamped-two-joints pose)
            (look-at-pose-stamped-many-joints pose))
        (if configuration
            (if (typep (car configuration) 'list)
                (look-at-joint-states configuration)
                (look-at-joint-angles configuration))
            (error "LOOK action has to have either pose or configuration given.")))))



;;;;;;;;;;;;;;;;; PERCEPTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun extend-perceived-object-designator (input-designator name-pose-type-list)
  (destructuring-bind (name pose type) name-pose-type-list
    (let* ((pose-stamped-in-fixed-frame
             (cl-transforms-stamped:make-pose-stamped
              cram-tf:*fixed-frame*
              (cut:current-timestamp)
              (cl-transforms:origin pose)
              (cl-transforms:orientation pose)))
           (transform-stamped-in-fixed-frame
             (cram-tf:pose-stamped->transform-stamped
              pose-stamped-in-fixed-frame
              (roslisp-utilities:rosify-underscores-lisp-name name)))
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
                         (:transform ,transform-stamped-in-base-frame)
                         (:pose-in-map ,pose-stamped-in-fixed-frame)
                         (:transform-in-map ,transform-stamped-in-fixed-frame)))))))
        (setf (slot-value output-designator 'desig:data)
              (make-instance 'desig:object-designator-data
                :object-identifier name
                :pose pose-stamped-in-fixed-frame))
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
             (prolog:prolog `(and (rob-int:robot ?robot)
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
      (cpl:fail 'common-fail:perception-object-not-found
                :object input-designator
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
                                                    (T (if (numberp action-type)
                                                           (* action-type
                                                              (cut:var-value
                                                               '?mult
                                                               solution-bindings))
                                                           (error "[PROJ GRIP] failed")))))))))
      solution-bindings))

   (cut:force-ll
    (prolog:prolog
     `(and (rob-int:robot ?robot)
           (rob-int:gripper-joint ?robot ,arm ?joint)
           (rob-int:joint-lower-limit ?robot ?joint ?min-limit)
           (rob-int:joint-upper-limit ?robot ?joint ?max-limit)
           (rob-int:gripper-meter-to-joint-multiplier ?robot ?mult)))))

  ;; check if there is an object to grip
  (when (eql action-type :grip) ; if action was gripping check if gripper collided with an item
    (unless (prolog:prolog
             `(and (btr:bullet-world ?world)
                   (rob-int:robot ?robot)
                   (btr:contact ?world ?robot ?object-name ?link)
                   (rob-int:gripper-link ?robot ,arm ?link)
                   (btr:%object ?world ?object-name ?object-instance)
                   ;; (or (prolog:lisp-type ?object-instance btr:item)
                   ;;     (prolog:lisp-type ?object-instance btr:robot-object))
                   ))
      (cpl:fail 'common-fail:gripper-closed-completely
                :description "There was no object to grip"))))

(defun gripper-action (action-type arm &optional maximum-effort)
  (if (and arm (listp arm))
      (cpl:par
        (one-gripper-action action-type (first arm) maximum-effort)
        (one-gripper-action action-type (second arm) maximum-effort))
      (one-gripper-action action-type arm maximum-effort)))

;;;;;;;;;;;;;;;;; ARMS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; joint movement

(defun move-joints (left-configuration right-configuration)
  (declare (type list left-configuration right-configuration))
  (flet ((set-configuration (arm joint-values)
           (when joint-values
             (let ((joint-name-value-list
                     (if (listp (car joint-values))
                         joint-values
                         (let ((joint-names
                                 (cut:var-value
                                  '?joints
                                  (car (prolog:prolog
                                        `(and (rob-int:robot ?robot)
                                              (rob-int:arm-joints ?robot ,arm ?joints)))))))
                           (unless (= (length joint-values) (length joint-names))
                             (error "[PROJECTION MOVE-JOINTS] length of joints list is incorrect"))
                           (mapcar (lambda (name value)
                                     (list name (* value 1.0d0)))
                                   joint-names joint-values)))))
               (assert
                (prolog:prolog
                 `(and
                   (btr:bullet-world ?world)
                   (rob-int:robot ?robot)
                   (assert ?world (btr:joint-state ?robot ,joint-name-value-list)))))))))
    (set-configuration :left left-configuration)
    (set-configuration :right right-configuration)))

;;; cartesian movement

(defun tcp-pose->ee-pose (tcp-pose tool-frame end-effector-frame)
  "TCP-POSE is in map frame"
  (when tcp-pose
    (let* ((ee-P-tcp
             (cut:var-value
              '?tcp-in-ee-pose
              (cut:lazy-car
               (prolog:prolog
                `(and (rob-int:robot ?robot)
                      (rob-int:tcp-in-ee-pose ?robot ?tcp-in-ee-pose))))))
           (map-T-tcp
             (cram-tf:pose->transform-stamped
              cram-tf:*fixed-frame*
              tool-frame
              0.0
              tcp-pose))
           (tcp-T-ee
             (cram-tf:transform-stamped-inv
              (cram-tf:pose->transform-stamped
               end-effector-frame
               tool-frame
               0.0
               ee-P-tcp)))
           (map-T-ee
             (cram-tf:multiply-transform-stampeds
              cram-tf:*fixed-frame*
              end-effector-frame
              map-T-tcp
              tcp-T-ee)))
      (cram-tf:strip-transform-stamped map-T-ee))))

(defun ee-pose-in-base->ee-pose-in-torso (ee-pose-in-base)
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

(defun ee-pose-in-map->ee-pose-in-torso (ee-pose-in-map)
  (when ee-pose-in-map
    (if (string-equal
         (cl-transforms-stamped:frame-id ee-pose-in-map)
         cram-tf:*fixed-frame*)
        ;; tPe: tTe = tTb * bTe = tTm * mTb * bTe = (mTt)-1 * mTb * bTe
        (let* ((map-T-base
                 (cram-tf:pose->transform-stamped
                  cram-tf:*fixed-frame*
                  cram-tf:*robot-base-frame*
                  0.0
                  (btr:pose (btr:get-robot-object))))
               (base-T-map
                 (cram-tf:transform-stamped-inv map-T-base))
               (map-T-ee
                 (cram-tf:pose-stamped->transform-stamped
                  ee-pose-in-map
                  "end_effector_link")))
          (ee-pose-in-base->ee-pose-in-torso
           (cram-tf:multiply-transform-stampeds
            cram-tf:*robot-base-frame*
            "end_effector_link"
            base-T-map
            map-T-ee
            :result-as-pose-or-transform :pose)))
        (error "Arm movement goals should be given in map frame"))))

(defparameter *torso-resampling-step* 0.1)

(defun get-ik-joint-positions (ee-pose base-link end-effector-link joint-names
                               torso-joint-name
                               torso-joint-lower-limit torso-joint-upper-limit)
  (when ee-pose
    (multiple-value-bind (ik-solution-msg torso-angle)
        (let ((torso-current-angle
                (btr:joint-state
                 (btr:get-robot-object)
                 torso-joint-name))
              (seed-state-msg
                (btr::make-robot-joint-state-msg
                 (btr:get-robot-object)
                 :joint-names joint-names)))
          (ik:call-ik-service-with-resampling
           ee-pose base-link end-effector-link seed-state-msg *torso-resampling-step* :z
           torso-current-angle torso-joint-lower-limit torso-joint-upper-limit))
      (unless ik-solution-msg
        (cpl:fail 'common-fail:manipulation-pose-unreachable
                  :description (format nil "~a is unreachable for EE." ee-pose)))
      (values (map 'list #'identity (roslisp:msg-slot-value ik-solution-msg :position))
              torso-angle))))

(defun perform-collision-check (collision-mode left-tcp-pose right-tcp-pose)
  (unless collision-mode
    (setf collision-mode :avoid-all))
  (ecase collision-mode
    (:allow-all
     nil)
    (:allow-attached
     ;; allow-attached means the robot is not allowed to hit anything
     ;; (except the object it is holding),
     ;; but the object it is holding can create collisions with environment etc.
     (when (and *be-strict-with-collisions*
                (btr:robot-colliding-objects-without-attached))
       (cpl:fail 'common-fail:manipulation-goal-not-reached
                 :description "Robot is in collision with environment.")))
    (:allow-hand
     ;; allow hand allows collisions between the hand and anything
     ;; but not the rest of the robot
     ;; therefore, we take a list of all links of the robot that are colliding
     ;; with something, remove attached object collisions from this list,
     ;; and then remove the hand links from the list.
     ;; if the list is still not empty, there is a collision between
     ;; a robot non-hand link and something else
     (when (and *be-strict-with-collisions*
                (set-difference
                 (mapcar #'cdr
                         (reduce (lambda (link-contacts attachment)
                                   (remove (btr:object
                                            btr:*current-bullet-world*
                                            (car attachment))
                                           link-contacts
                                           :key #'car))
                                 (append
                                  (list (btr:link-contacts (btr:get-robot-object)))
                                  (btr:attached-objects (btr:get-robot-object)))))
                 (append (when left-tcp-pose
                           (cut:var-value
                            '?hand-links
                            (car (prolog:prolog
                                  `(and (rob-int:robot ?robot)
                                        (rob-int:hand-links ?robot :left ?hand-links))))))
                         (when right-tcp-pose
                           (cut:var-value
                            '?hand-links
                            (car (prolog:prolog
                                  `(and (rob-int:robot ?robot)
                                        (rob-int:hand-links ?robot :right ?hand-links)))))))
                 :test #'string-equal))
       (cpl:fail 'common-fail:manipulation-goal-not-reached
                 :description "Robot is in collision with environment.")))
    (:avoid-all
     ;; avoid-all means the robot is not colliding with anything except the
     ;; objects it is holding, and the objects it is holding only collides with robot
     (when (or (btr:robot-colliding-objects-without-attached)
               (btr:robot-attached-objects-in-collision))
       (cpl:fail 'common-fail:manipulation-goal-not-reached
                 :description "Robot is in collision with environment.")))))

(defun move-tcp (left-tcp-pose right-tcp-pose
                 &optional collision-mode
                   collision-object-b collision-object-b-link collision-object-a)
  (declare (type (or cl-transforms-stamped:pose-stamped null) left-tcp-pose right-tcp-pose))
  (declare (ignore collision-object-b collision-object-b-link collision-object-a))

  (cram-tf:visualize-marker (list left-tcp-pose right-tcp-pose) :r-g-b-list '(1 0 1))

  (cut:with-vars-strictly-bound (?robot
                                 ?left-tool-frame ?right-tool-frame
                                 ?left-ee-frame ?right-ee-frame
                                 ?left-arm-joints ?right-arm-joints
                                 ?torso-link ?torso-joint ?lower-limit ?upper-limit)
      (cut:lazy-car
       (prolog:prolog
        `(and (rob-int:robot ?robot)
              (rob-int:robot-tool-frame ?robot :left ?left-tool-frame)
              (rob-int:robot-tool-frame ?robot :right ?right-tool-frame)
              (rob-int:end-effector-link ?robot :left ?left-ee-frame)
              (rob-int:end-effector-link ?robot :right ?right-ee-frame)
              (rob-int:arm-joints ?robot :left ?left-arm-joints)
              (rob-int:arm-joints ?robot :right ?right-arm-joints)
              (rob-int:robot-torso-link-joint ?robot ?torso-link ?torso-joint)
              (rob-int:joint-lower-limit ?robot ?torso-joint ?lower-limit)
              (rob-int:joint-upper-limit ?robot ?torso-joint ?upper-limit))))

    (multiple-value-bind (left-ik left-torso-angle)
        ;; TODO: the LET is a temporary hack until we get a relay running for PR2
        ;; such that both arms IKs go over the same ROS service
        (let ((ik::*ik-service-name*
                (if (string-equal (symbol-name ?robot) "PR2")
                    "pr2_left_arm_kinematics/get_ik"
                    "kdl_ik_service/get_ik")))
          (get-ik-joint-positions (ee-pose-in-map->ee-pose-in-torso
                                   (tcp-pose->ee-pose left-tcp-pose
                                                      ?left-tool-frame ?left-ee-frame))
                                  ?torso-link ?left-ee-frame ?left-arm-joints
                                  ?torso-joint ?lower-limit ?upper-limit))
      (multiple-value-bind (right-ik right-torso-angle)
          (let ((ik::*ik-service-name*
                  (if (string-equal (symbol-name ?robot) "PR2")
                      "pr2_right_arm_kinematics/get_ik"
                      "kdl_ik_service/get_ik")))
            (get-ik-joint-positions (ee-pose-in-map->ee-pose-in-torso
                                     (tcp-pose->ee-pose right-tcp-pose
                                                        ?right-tool-frame ?right-ee-frame))
                                    ?torso-link ?right-ee-frame ?right-arm-joints
                                    ?torso-joint ?lower-limit ?upper-limit))
        (cond
          ((and left-torso-angle right-torso-angle)
           (when (not (eq left-torso-angle right-torso-angle))
             (cpl:fail 'common-fail:manipulation-pose-unreachable
                       :description (format nil "In MOVE-TCP goals for the two arms ~
                                                 require different torso angles).")))
           (move-torso left-torso-angle))
          (left-torso-angle (move-torso left-torso-angle))
          (right-torso-angle (move-torso right-torso-angle)))
        (move-joints left-ik right-ik)
        (perform-collision-check collision-mode left-tcp-pose right-tcp-pose)))))
