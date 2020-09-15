;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;               2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2019, Vanessa Hassouna <hassouna@uni-bremen.de>
;;;               2020, Amar Fayaz <amar@uni-bremen.de>
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
  "when grasping a spoon from table, fingers can collide with kitchen, so allow this")

(defparameter *projection-convergence-delta-joint* 0.17
  "For moving arm joints. In radiants, about 10 degrees.")

(defparameter *torso-resampling-step* 0.1d0
  "In meters.")
(defparameter *base-resampling-step* 0.1d0
  "In meters")
(defparameter *base-resampling-x-limit* 0.2d0
  "In meters")
(defparameter *base-resampling-y-limit* 0.2d0
  "In meters.")

(defparameter *giskard-mode* nil
  "Use giskard or teleporting/IK solver for joint and cartesian motions.")


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

(defun robot-joint-states-with-odom-joints-as-hash-table ()
  (let* ((observed-joint-states
           (alexandria:copy-hash-table (btr:joint-states (btr:get-robot-object))))
         (robot-pose
           (btr:pose (btr:get-robot-object)))
         (robot-x
           (cl-transforms:x (cl-transforms:origin robot-pose)))
         (robot-y
           (cl-transforms:y (cl-transforms:origin robot-pose))))
    (multiple-value-bind (axis angle)
        (cl-transforms:quaternion->axis-angle
         (cl-transforms:orientation robot-pose))
      (when (< (cl-transforms:z axis) 0)
        (setf angle (- angle)))
      (setf (gethash "odom_x_joint" observed-joint-states) robot-x)
      (setf (gethash "odom_y_joint" observed-joint-states) robot-y)
      (setf (gethash "odom_z_joint" observed-joint-states) angle)
      observed-joint-states)))

;;;;;;;;;;;;;;;;; NAVIGATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun drive (target)
  (declare (type cl-transforms-stamped:pose-stamped target))

  (btr:add-vis-axis-object target)

  (let ((world-pose-info (btr:get-world-objects-pose-info)))
    (unwind-protect
         (progn
           ;; assert new robot pose
           (assert
            (prolog:prolog
             `(and (rob-int:robot ?robot)
                   (btr:bullet-world ?w)
                   (btr:assert ?w (btr:object-pose ?robot ,target)))))
           ;; return joint state. this will be our observation
           ;; currently only used by HPN
           (robot-joint-states-with-odom-joints-as-hash-table))
      (when (or (btr:robot-colliding-objects-without-attached '(:floor))
                (btr:robot-attached-objects-in-collision))
        (unless (< (abs *debug-short-sleep-duration*) 0.0001)
          (cpl:sleep *debug-short-sleep-duration*))
        (btr:restore-world-poses world-pose-info)
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
                  :description (format nil "Torso goal ~a was out of joint limits"
                                       joint-angle)
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

  ;; first look forward, because our IK with 2 joints is buggy...
  (look-at-joint-angles '(0 0))

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
             (btr:calculate-pan-tilt
              (btr:get-robot-object) ?pan-link ?tilt-link pose-in-world))
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

(defun get-neck-ik (ee-link cartesian-pose base-link joint-names)
  (cut:with-vars-strictly-bound (?camera-resampling-step
                                 ?camera-x-axis-limit
                                 ?camera-y-axis-limit
                                 ?camera-z-axis-limit)
      (car
       (prolog:prolog
        '(and
          (rob-int:robot ?robot)
          (rob-int:neck-camera-resampling-step ?robot ?camera-resampling-step)
          (rob-int:neck-camera-x-axis-limit ?robot ?camera-x-axis-limit)
          (rob-int:neck-camera-y-axis-limit ?robot ?camera-y-axis-limit)
          (rob-int:neck-camera-z-axis-limit ?robot ?camera-z-axis-limit))))

    (let* ((validation-function
             (lambda (ik-solution-msg torso-offsets)
               (declare (ignore torso-offsets))
               (not (perform-collision-check :avoid-all nil nil ik-solution-msg))))
           (cartesian-pose-stamped
             (cl-transforms-stamped:pose->pose-stamped
              base-link 0.0
              cartesian-pose))
           (neck-seed-state-msg
             (btr::make-robot-joint-state-msg
              (btr:get-robot-object)
              :joint-names joint-names))
           (joint-state-msg
             (ik:find-ik-for
                 (cartesian-pose-stamped
                  base-link
                  ee-link
                  neck-seed-state-msg
                  validation-function)
               (ik:with-resampling
                   (:x
                    ?camera-x-axis-limit
                    (- ?camera-x-axis-limit)
                    ?camera-resampling-step)
                 (ik:with-resampling
                     (:y
                      ?camera-y-axis-limit
                      (- ?camera-y-axis-limit)
                      ?camera-resampling-step)
                   (ik:with-resampling
                       (:z
                        ?camera-z-axis-limit
                        (- ?camera-z-axis-limit)
                        ?camera-resampling-step)))))))
      (when joint-state-msg
        (map 'list #'identity
             (roslisp:msg-slot-value joint-state-msg :position))))))

(defun calculate-camera-pose-from-object-pose (neck-base-t-object)
  "Takes the vector from neck-base to object, sets its Z to 0,
then normalizes to get a unit vector, then multiplies with a multiplier
to make it shorter (multiplier should be comparable to maximum length
between neck base and camera), then pulls the vector up in Z a bit
to avoid colliding with bottom parts of robot, and that would be
the desired camera position.
Next, to calculate desired camera rotation, looks at the object from camera position,
calculates an angle that would have to be applied around X axis to align camera's Z
with the object, calculates similar angle around Y axis and applies the rotations. "
  (let* ((neck-base-t-object-vector
           (cl-transforms:translation neck-base-t-object))
         (neck-base-t-object-vector-without-z
           (cl-transforms:copy-3d-vector neck-base-t-object-vector :z 0.0))
         (neck-base-t-object-unit-vector
           (cl-transforms:normalize-vector neck-base-t-object-vector-without-z))
         (neck-camera-pose-unit-vector-multiplier
           (cut:var-value
            '?mult
            (car
             (prolog:prolog
              `(and (rob-int:robot ?rob)
                    (rob-int:neck-camera-pose-unit-vector-multiplier ?rob ?mult))))))
         (neck-base-t-object-short-vector
           (cl-transforms:v*
            neck-base-t-object-unit-vector
            neck-camera-pose-unit-vector-multiplier))
         (neck-base-t-object-short-vector-lifted
           (cl-transforms:v+
            neck-base-t-object-short-vector
            (cl-transforms:make-3d-vector
             0 0 (cut:var-value
                  '?z-offset
                  (car (prolog:prolog
                        `(and (rob-int:robot ?robot)
                              (rob-int:neck-camera-z-offset ?robot ?z-offset))))))))
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
               (cl-transforms:y
                (cl-transforms:translation camera-not-oriented-t-object))
               (cl-transforms:z
                (cl-transforms:translation camera-not-oriented-t-object)))))
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
            (cl-transforms:x
             (cl-transforms:translation camera-rotated-around-x-t-object))
            (cl-transforms:z
             (cl-transforms:translation camera-rotated-around-x-t-object))))
         (neck-base-t-camera
           (cram-tf:rotate-transform-in-own-frame
            neck-base-t-camera-rotated-around-x
            :y rotation-angle-around-y)))
    neck-base-t-camera))

(defun look-at-pose-stamped-many-joints (object-pose)
  (declare (type cl-transforms-stamped:pose-stamped object-pose))
  (let* ((map-p-object
           (cram-tf:ensure-pose-in-frame
            object-pose
            cram-tf:*fixed-frame*
            :use-zero-time t))
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
         (map-t-neck-base
           (cl-transforms:pose->transform map-p-neck-base))
         (neck-base-t-map
           (cl-transforms:transform-inv map-t-neck-base))
         (neck-base-t-object
           (cl-transforms:transform*
            neck-base-t-map
            (cl-transforms:pose->transform map-p-object)))

         (neck-base-t-camera
           (calculate-camera-pose-from-object-pose neck-base-t-object))

         (camera-t-neck-ee
           (cl-transforms:transform-inv
            (cl-transforms:pose->transform neck-ee-p-camera)))
         (neck-base-t-neck-ee
           (cl-transforms:transform* neck-base-t-camera camera-t-neck-ee))
         (neck-base-p-neck-ee
           (cl-transforms:transform->pose neck-base-t-neck-ee))

         (joint-state
           (get-neck-ik neck-ee-frame neck-base-p-neck-ee
                        neck-base-frame neck-joints))

         (map-t-camera
            (cl-transforms:transform-pose map-t-neck-base neck-base-t-camera)))

    (btr:add-vis-axis-object map-t-camera)

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
                            (rob-int:robot-neck-joints ?robot . ?joint-names))))))))
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
         ;; name-pose-type-lists is, e.g.:
         ;; ((mondamin-1 <pose-1> :mondamin) (mug-2 <pose-2> :mug))
         (name-pose-type-lists
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
                                  (btr:item ?world ?object-name)
                                  ;; it is possible to ask RoboSherlock for
                                  ;; (all object (type kitchen-object))
                                  ;; which returns a list of all the objects RS sees
                                  ;; to support that in projection, we should allow
                                  ;; an object-type :kitchen-object.
                                  ;; for asking objects of specific type, the part
                                  ;; with :kitchen-object is irrelevant
                                  ,@(when (and object-type
                                               (not (eq object-type :kitchen-object)))
                                      `((prolog:== ?object-type ,object-type)))
                                  (btr:item-type ?world ?object-name ?object-type)
                                  (btr:visible ?world ?robot ?object-name)
                                  (btr:pose ?world ?object-name ?object-pose)))))))

    ;; check if objects were found
    (unless name-pose-type-lists
      (cpl:fail 'common-fail:perception-object-not-found
                :object input-designator
                :description (format nil "Could not find object ~a."
                                     input-designator)))

    ;; Extend the input-designator with the information found
    ;; through visibility check: name & pose & type of the object,
    ;; equate the input-designator to the new output-designator.
    ;; If multiple objects are visible, return multiple equated objects,
    ;; otherwise only take first found object.
    ;; I.e. need to find :an object (not :all objects)
    (case quantifier
      (:all (mapcar (alexandria:curry #'extend-perceived-object-designator
                                      input-designator)
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
        (assert ?world
                (btr:joint-state ?robot
                                 ((?joint ,(case action-type
                                             (:open '?max-limit)
                                             ((:close :grip) '?min-limit)
                                             (T (if (numberp action-type)
                                                    (* action-type
                                                       (cut:var-value
                                                        '?mult solution-bindings))
                                                    (error
                                                     "[PROJ GRIP] failed")))))))))
      solution-bindings))

   (cut:force-ll
    (prolog:prolog
     `(and (rob-int:robot ?robot)
           (rob-int:gripper-joint ?robot ,arm ?joint)
           (rob-int:joint-lower-limit ?robot ?joint ?min-limit)
           (rob-int:joint-upper-limit ?robot ?joint ?max-limit)
           (rob-int:gripper-meter-to-joint-multiplier ?robot ?mult)))))

  ;; check if there is an object to grip
  ;; i.e. if action was gripping check if gripper collided with an item
  (when (eql action-type :grip)
    (unless (prolog:prolog
             `(and (btr:bullet-world ?world)
                   (rob-int:robot ?robot)
                   (btr:contact ?world ?robot ?object-name ?link)
                   (rob-int:hand-link ?robot ,arm ?link)
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

;;; utilities and collision check function

(defun calculate-base-pose-with-torso-offsets (map-p-base map-p-torso
                                               x-offset y-offset)
  (declare (type cl-transforms:pose map-p-base map-p-torso)
           (type (or null number) x-offset y-offset))
  (if (or (and (not x-offset) (not y-offset))
          (and (equalp x-offset 0.0) (equalp y-offset 0.0)))
      map-p-base
      (let* ((x-offset
               (or x-offset 0.0))
             (y-offset
               (or y-offset 0.0))
             (map-t-base
               (cram-tf:pose->transform-stamped
                cram-tf:*fixed-frame* cram-tf:*robot-base-frame* 0.0
                map-p-base))
             (map-t-torso
               (cram-tf:pose->transform-stamped
                cram-tf:*fixed-frame* cram-tf:*robot-torso-frame* 0.0
                map-p-torso))
             (base-t-torso
               (cram-tf:apply-transform
                (cram-tf:transform-stamped-inv map-t-base)
                map-t-torso))
             (torso-t-base
               (cram-tf:transform-stamped-inv
                base-t-torso))
             (torso-t-torso-offsetted
               (cl-transforms-stamped:make-transform-stamped
                cram-tf:*robot-torso-frame*
                cram-tf:*robot-torso-frame*
                0.0
                (cl-transforms:make-3d-vector x-offset y-offset 0.0)
                (cl-transforms:make-identity-rotation)))
             (map-t-base-offsetted
               (reduce
                #'cram-tf:apply-transform
                `(,map-t-base ,base-t-torso ,torso-t-torso-offsetted ,torso-t-base))))
        (cram-tf:strip-transform-stamped map-t-base-offsetted))))

(defun apply-torso-offsets (torso-offsets)
  (let* ((torso-offset-x
           (cdr (assoc :x torso-offsets)))
         (torso-offset-y
           (cdr (assoc :y torso-offsets)))
         (torso-offset-z
           (cdr (assoc :z torso-offsets)))
         (current-torso-angle
           (btr:joint-state (btr:get-robot-object) cram-tf:*robot-torso-joint*))
         (new-torso-angle
           (+ current-torso-angle torso-offset-z))
         (new-base-pose-stamped
           (calculate-base-pose-with-torso-offsets
            (btr:pose (btr:get-robot-object))
            (btr:link-pose (btr:get-robot-object) cram-tf:*robot-torso-frame*)
            torso-offset-x torso-offset-y)))
    (move-torso new-torso-angle)
    (setf (btr:pose (btr:get-robot-object)) new-base-pose-stamped)))

(defun perform-collision-check (collision-mode left-hand-moves right-hand-moves
                                &optional
                                  joint-state-msg
                                  torso-offsets
                                  object-name-to-allow-collisions-with)
  (declare (type (or keyword null) collision-mode)
           (type (or sensor_msgs-msg:jointstate null) joint-state-msg)
           (type list torso-offsets)
           (type (or keyword string null) object-name-to-allow-collisions-with))
  "Returns NIL if current joint state does not result in collisions
and returns (not throws or fails but simply returns) an error instance,
if a collision occurs.
`left-hand-moves' and `right-hand-moves' are used as flags for the :allow-hand
collision mode, to see which hand is currently moving and can be allowed collisions.
If `joint-state-msg' is given, check collisions in that joint state
 (and restore the world to original state afterwards),
otherwise check collisions in current joint state.
Same goes for `torso-offsets', if they are given, move the robot torso and base
with the given offsets (the offsets are specified in the torso frame).
`torso-offsets' is a list of (:x/:y/:z offset-in-torso-frame) entries."
  (flet ((the-actual-collision-check (collision-mode
                                      left-hand-moves right-hand-moves)
           (ecase collision-mode
             (:allow-all
              nil)
             (:allow-attached
              ;; allow-attached means the robot is not allowed to hit anything
              ;; (except the object it is holding),
              ;; but the object it is holding can create collisions
              ;; with environment etc.
              (when (and *be-strict-with-collisions*
                         (btr:robot-colliding-objects-without-attached))
                (make-instance 'common-fail:manipulation-goal-not-reached
                  :description "Robot is in collision with environment.")))
             ((or :allow-hand
                  :allow-arm)
              ;; allow hand allows collisions between the hand and anything
              ;; but not the rest of the robot
              ;; therefore, we take a list of all links of the robot
              ;; that are colliding with something,
              ;; remove attached object collisions from this list,
              ;; and then remove the hand links from the list.
              ;; if the list is still not empty, there is a collision between
              ;; a robot non-hand link and something else
              (when (and
                     *be-strict-with-collisions*
                     (set-difference
                      (mapcar
                       #'cdr
                       (reduce
                        (lambda (link-contacts attachment)
                          (remove
                           (btr:object btr:*current-bullet-world* (car attachment))
                           link-contacts
                           :key #'car))
                        (append
                         (list (btr:link-contacts (btr:get-robot-object)))
                         (btr:attached-objects (btr:get-robot-object)))))
                      (append
                       (when left-hand-moves
                         (cut:var-value
                          '?links
                          (car
                           (prolog:prolog
                            `(and
                              (rob-int:robot ?robot)
                              ,(ecase collision-mode
                                 (:allow-hand
                                  '(rob-int:hand-links ?robot :left ?links))
                                 (:allow-arm
                                  '(rob-int:arm-links ?robot :left ?links))))))))
                       (when right-hand-moves
                         (cut:var-value
                          '?links
                          (car
                           (prolog:prolog
                            `(and
                              (rob-int:robot ?robot)
                              ,(ecase collision-mode
                                 (:allow-hand
                                  '(rob-int:hand-links ?robot :right ?links))
                                 (:allow-arm
                                  '(rob-int:arm-links ?robot :right ?links)))))))))
                      :test #'string-equal))
                (make-instance 'common-fail:manipulation-goal-not-reached
                  :description "Robot is in collision with environment.")))
             (:avoid-all
              ;; avoid-all means the robot is not colliding with anything except the
              ;; objects it is holding, and the object it is holding
              ;; only collides with robot
              (when (or (remove object-name-to-allow-collisions-with
                                (btr:robot-colliding-objects-without-attached))
                        (remove object-name-to-allow-collisions-with
                                (btr:robot-attached-objects-in-collision)))
                (make-instance 'common-fail:manipulation-goal-not-reached
                  :description "Robot is in collision with environment."))))))

    (unless collision-mode
      (setf collision-mode :avoid-all))
    ;; if joint state is given, first set the robot to given state,
    ;; then perform the collision check, then restore the robot to original state
    (if (or torso-offsets joint-state-msg)
        (let ((world-pose-info (btr:get-world-objects-pose-info)))
          (unwind-protect
               (progn
                 (when joint-state-msg
                   (btr:set-robot-state-from-joints
                    joint-state-msg (btr:get-robot-object)))
                 (when torso-offsets
                   (apply-torso-offsets torso-offsets))
                 (the-actual-collision-check
                  collision-mode left-hand-moves right-hand-moves))
            (btr:restore-world-poses world-pose-info)))
        (the-actual-collision-check
         collision-mode left-hand-moves right-hand-moves))))

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
                                              (rob-int:arm-joints ?robot ,arm
                                                                  ?joints)))))))
                           (unless (= (length joint-values) (length joint-names))
                             (error "[PROJECTION MOVE-JOINTS] length of joints list ~
                                     is incorrect"))
                           (mapcar (lambda (name value)
                                     (list name (* value 1.0d0)))
                                   joint-names joint-values)))))
               (assert
                (prolog:prolog
                 `(and
                   (btr:bullet-world ?world)
                   (rob-int:robot ?robot)
                   (assert ?world (btr:joint-state ?robot ,joint-name-value-list)))))
               ;; check if joint state was indeed reached
               (let* ((robot-object
                        (btr:get-robot-object))
                      (current-joint-state
                        (mapcar (lambda (joint-name-and-value)
                                  (btr:joint-state robot-object
                                                   (car joint-name-and-value)))
                                joint-name-value-list))
                      (goal-joint-state
                        (mapcar #'second joint-name-value-list)))
                 (unless (cram-tf:values-converged
                          (cram-tf:normalize-joint-angles current-joint-state)
                          (cram-tf:normalize-joint-angles goal-joint-state)
                          *projection-convergence-delta-joint*)
                   (cpl:fail 'common-fail:manipulation-goal-not-reached
                             :description
                             (format nil
                                     "Projection did not converge to goal:~%~
                                     ~a (~a)~%should have been at~%~a~%~
                                     with delta-joint of ~a."
                                     arm
                                     (cram-tf:normalize-joint-angles
                                      current-joint-state)
                                     (cram-tf:normalize-joint-angles
                                      goal-joint-state)
                                     *projection-convergence-delta-joint*))))))))
    (set-configuration :left left-configuration)
    (set-configuration :right right-configuration)))

(defun move-joints-avoiding-collision (left-configuration right-configuration
                                       &optional collision-mode)
  "Moves arm joints with the specified configuration but tries to avoid
collision by moving its torso and base"
  (cut:with-vars-strictly-bound (?torso-joint
                                 ?torso-lower-limit ?torso-upper-limit)
      (cut:lazy-car
       (prolog:prolog
        `(and
          (rob-int:robot ?robot)
          (rob-int:robot-torso-link-joint ?robot ?torso-link ?torso-joint)
          (rob-int:joint-lower-limit ?robot ?torso-joint ?torso-lower-limit)
          (rob-int:joint-upper-limit ?robot ?torso-joint ?torso-upper-limit))))
    (let* ((current-torso-angle
             (btr:joint-state (btr:get-robot-object) ?torso-joint))
           (validation-function
             (lambda (torso-offsets)
               (not (perform-collision-check
                     collision-mode
                     left-configuration
                     right-configuration
                     nil
                     torso-offsets))))
           (joint-values
             (ik:find-joint-values-for (validation-function)
               (ik:with-resampling
                   (:x
                    *base-resampling-x-limit*
                    (- *base-resampling-x-limit*)
                    *base-resampling-step*)
                 (ik:with-resampling
                     (:y
                      *base-resampling-y-limit*
                      (- *base-resampling-y-limit*)
                      *base-resampling-step*)
                   (ik:with-resampling
                       (:z
                        (- ?torso-upper-limit current-torso-angle)
                        (- ?torso-lower-limit current-torso-angle)
                        *torso-resampling-step*)
                     (move-joints left-configuration right-configuration)))))))

      (when joint-values
        (let* ((torso-offset-z
                 (cdr (assoc :z joint-values)))
               (new-torso-angle
                 (+ current-torso-angle torso-offset-z))
               (torso-offset-x
                 (cdr (assoc :x joint-values)))
               (torso-offset-y
                 (cdr (assoc :y joint-values)))
               (new-base-pose-stamped
                 (calculate-base-pose-with-torso-offsets
                  (btr:pose (btr:get-robot-object))
                  (btr:link-pose (btr:get-robot-object)
                                 cram-tf:*robot-torso-frame*)
                  torso-offset-x torso-offset-y)))
          (move-torso new-torso-angle)
          (setf (btr:pose (btr:get-robot-object)) new-base-pose-stamped)))))
  (perform-collision-check collision-mode left-configuration right-configuration))

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

(defun get-ik-joint-positions (ee-pose base-link end-effector-link joint-names
                               torso-joint-name
                               torso-joint-lower-limit torso-joint-upper-limit
                               validation-function
                               move-base)
  (when ee-pose
    (let ((current-torso-angle
            (btr:joint-state (btr:get-robot-object) torso-joint-name))
          (seed-state-msg
            (btr::make-robot-joint-state-msg
             (btr:get-robot-object)
             :joint-names joint-names))
          (disable-base-resampling
            (not move-base)))
      (multiple-value-bind (ik-solution-msg joint-values)
          (ik:find-ik-for
           (ee-pose
            base-link
            end-effector-link
            seed-state-msg
            validation-function)
           (ik:with-resampling
               (:x
                *base-resampling-x-limit*
                (- *base-resampling-x-limit*)
                *base-resampling-step*
                :disable-resampling disable-base-resampling)
             (ik:with-resampling
                 (:y
                  *base-resampling-y-limit*
                  (- *base-resampling-y-limit*)
                  *base-resampling-step*
                  :disable-resampling disable-base-resampling)
               (ik:with-resampling
                   (:z
                    (- torso-joint-upper-limit current-torso-angle)
                    (- torso-joint-lower-limit current-torso-angle)
                    *torso-resampling-step*
                    :disable-resampling disable-base-resampling)))))
        (unless ik-solution-msg
          (cpl:fail 'common-fail:manipulation-low-level-failure
                    :description
                    (format nil "~a is unreachable for EE or is in collision."
                            ee-pose)))
        (let* ((torso-offset-z
                 (cdr (assoc :z joint-values)))
               (new-torso-angle
                 (+ current-torso-angle torso-offset-z))
               (torso-offset-x
                 (cdr (assoc :x joint-values)))
               (torso-offset-y
                 (cdr (assoc :y joint-values)))
               (new-base-pose-stamped
                 (calculate-base-pose-with-torso-offsets
                  (btr:pose (btr:get-robot-object))
                  (btr:link-pose (btr:get-robot-object) cram-tf:*robot-torso-frame*)
                  torso-offset-x torso-offset-y)))
          (values (map 'list #'identity
                       (roslisp:msg-slot-value ik-solution-msg :position))
                  new-torso-angle
                  new-base-pose-stamped))))))

(defun move-tcp-one-pose (left-tcp-pose right-tcp-pose
                          &optional
                            collision-mode
                            collision-object-b
                            collision-object-b-link
                            collision-object-a
                            (move-base t))
  (declare (type (or cl-transforms-stamped:pose-stamped null)
                 left-tcp-pose right-tcp-pose)
           (type boolean move-base))
  (declare (ignore collision-object-b collision-object-b-link collision-object-a))

  (cram-tf:visualize-marker (list left-tcp-pose right-tcp-pose) :r-g-b-list '(1 0 1))
  (when right-tcp-pose
    (btr:add-vis-axis-object right-tcp-pose))
  (when left-tcp-pose
    (btr:add-vis-axis-object left-tcp-pose))
  (cut:with-vars-strictly-bound (?robot
                                 ?left-tool-frame ?right-tool-frame
                                 ?left-ee-frame ?right-ee-frame
                                 ?left-arm-joints ?right-arm-joints
                                 ?torso-link ?torso-joint
                                 ?lower-limit ?upper-limit)
      (cut:lazy-car
       (prolog:prolog
        `(and (rob-int:robot ?robot)
              (once (or (rob-int:robot-tool-frame ?robot :left ?left-tool-frame)
                        (equal ?left-tool-frame nil)))
              (once (or (rob-int:robot-tool-frame ?robot :right ?right-tool-frame)
                        (equal ?right-tool-frame nil)))
              (once (or (rob-int:end-effector-link ?robot :left ?left-ee-frame)
                        (equal ?left-ee-frame nil)))
              (once (or (rob-int:end-effector-link ?robot :right ?right-ee-frame)
                        (equal ?right-ee-frame nil)))
              (once (or (rob-int:arm-joints ?robot :left ?left-arm-joints)
                        (equal ?left-arm-joints nil)))
              (once (or (rob-int:arm-joints ?robot :right ?right-arm-joints)
                        (equal ?right-arm-joints nil)))
              (rob-int:robot-torso-link-joint ?robot ?torso-link ?torso-joint)
              (rob-int:joint-lower-limit ?robot ?torso-joint ?lower-limit)
              (rob-int:joint-upper-limit ?robot ?torso-joint ?upper-limit))))

    (let ((validation-function
            (lambda (ik-solution-msg torso-offsets)
              (not (perform-collision-check
                    collision-mode left-tcp-pose right-tcp-pose
                    ik-solution-msg torso-offsets)))))
      (multiple-value-bind (left-ik left-torso-angle left-base-pose)
          ;; TODO: the LET is a temporary hack until we get a relay running
          ;; for PR2 such that both arms IKs go over the same ROS service
          (let ((ik::*ik-service-name*
                  (if (eql ?robot :pr2)
                      "pr2_left_arm_kinematics/get_ik"
                      "kdl_ik_service/get_ik")))
            (get-ik-joint-positions
             (ee-pose-in-map->ee-pose-in-torso
              (tcp-pose->ee-pose left-tcp-pose ?left-tool-frame ?left-ee-frame))
             ?torso-link ?left-ee-frame ?left-arm-joints
             ?torso-joint ?lower-limit ?upper-limit
             validation-function
             move-base))
        (multiple-value-bind (right-ik right-torso-angle right-base-pose)
            (let ((ik::*ik-service-name*
                    (if (eql ?robot :pr2)
                        "pr2_right_arm_kinematics/get_ik"
                        "kdl_ik_service/get_ik")))
              (get-ik-joint-positions
               (ee-pose-in-map->ee-pose-in-torso
                (tcp-pose->ee-pose right-tcp-pose ?right-tool-frame ?right-ee-frame))
               ?torso-link ?right-ee-frame ?right-arm-joints
               ?torso-joint ?lower-limit ?upper-limit
               validation-function
               move-base))
          ;; set the torso to inferred position
          (cond
            ((and left-torso-angle right-torso-angle)
             (if (eq left-torso-angle right-torso-angle)
                 (move-torso left-torso-angle)
                 (cpl:fail 'common-fail:manipulation-pose-unreachable
                           :description (format nil "In MOVE-TCP goals ~
                                         for the two arms require different ~
                                         torso angles."))))
            (left-torso-angle
             (move-torso left-torso-angle))
            (right-torso-angle
             (move-torso right-torso-angle)))
          ;; set the base to inferred position
          (cond
            ((and left-base-pose right-base-pose)
             (if (funcall (cram-tf:make-euclidean-distance-filter
                           left-base-pose *base-resampling-step*)
                          right-base-pose)
                 (setf (btr:pose (btr:get-robot-object)) left-base-pose)
                 (cpl:fail 'common-fail:manipulation-pose-unreachable
                           :description (format nil "In MOVE-TCP goals ~
                                           for the two arms require different ~
                                           base-poses."))))
            (left-base-pose
             (setf (btr:pose (btr:get-robot-object)) left-base-pose))
            (right-base-pose
             (setf (btr:pose (btr:get-robot-object)) right-base-pose)))
          ;; set the arm joints to inferred position
          (move-joints left-ik right-ik)
          ;; perform one last collision check
          (perform-collision-check collision-mode left-tcp-pose right-tcp-pose))))))

(defun move-tcp (left-poses right-poses
                 &optional
                   collision-mode
                   collision-object-b
                   collision-object-b-link
                   collision-object-a
                   (move-base t))
  (declare (type (or cl-transforms-stamped:pose-stamped list null)
                 left-poses right-poses)
           (type boolean move-base))

  (multiple-value-bind (left-poses right-poses)
      (cut:equalize-two-list-lengths left-poses right-poses)

    ;; Move arms through all but last poses of `left-poses' and `right-poses'
    ;; while ignoring failures: accuracy is not so important in intermediate poses.
    (mapc (lambda (left-pose right-pose)
            (cpl:with-failure-handling
                ;; ignore intermediate pose failures
                ((common-fail:manipulation-low-level-failure (e)
                   (roslisp:ros-warn (urdf-proj move-tcp) "~a~%Ignoring." e)
                   (return)))
              (move-tcp-one-pose left-pose right-pose
                                 collision-mode collision-object-b
                                 collision-object-b-link collision-object-a
                                 move-base)))
          (butlast left-poses)
          (butlast right-poses))

    ;; Move arm to the last pose of `left-poses' and `right-poses'.
    (let ((left-pose (car (last left-poses)))
          (right-pose (car (last right-poses))))
      (cpl:with-failure-handling
          ((common-fail:manipulation-low-level-failure (e)
             ;; propagate failures up
             (roslisp:ros-error (urdf-proj move-tcp) "~a~%Failing." e)))
        (move-tcp-one-pose left-pose right-pose
                           collision-mode collision-object-b
                           collision-object-b-link collision-object-a
                           move-base)))))


#+save-for-next-time
((giskard:call-giskard-cartesian-action :goal-pose-left (cl-transforms-stamped:make-pose-stamped "l_gripper_tool_frame" 0.0 (cl-transforms:make-identity-vector) (cl-transforms:make-identity-rotation)))
 (let* ((root-link (cl-urdf:name (cl-urdf:root-link (btr:urdf (btr:get-robot-object)))))
        (robot-transform
          (handler-case
              (cl-transforms-stamped:lookup-transform
               cram-tf:*transformer* cram-tf:*fixed-frame* root-link
               :time 0.0 :timeout cram-tf:*tf-default-timeout*)
            (cl-tf:transform-stamped-error (error)
              (roslisp:ros-warn (set-robot-state-from-tf)
                                "Failed with transform-stamped-error:~%    ~a~%    ~
                                  Ignore this warning if no real robot is running."
                                error)
              NIL))))
   (when robot-transform
     (setf (btr:link-pose (btr:get-robot-object) root-link)
           (cl-transforms:transform->pose robot-transform)))))
