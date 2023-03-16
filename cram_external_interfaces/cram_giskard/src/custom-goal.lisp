(in-package :giskard)



(defun make-custom-grasp-constraint (goal-pose grasp-type
                                     &key avoid-collisions-not-much)
  
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "PrepareGraspBox"
   :parameter_value_pair
   (giskard::alist->json-string
    `(("box_pose"
       . (("message_type" . "geometry_msgs/PoseStamped")
          ("message" . ,(giskard::to-hash-table goal-pose))))
      ("grasp_type"
         . ,grasp-type)
      
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))



;;================================================================================================



(defparameter *gripper-convergence-delta-joint* 0.005 "In meters.")
(defparameter *gripper-gripped-min-position* 0.007 "In meters.")

(defun make-custom-action-goal (box-pose grasp-type)
   
  (giskard::make-giskard-goal
   :joint-constraints (giskard::make-custom-grasp-constraint box-pose grasp-type)
   :collisions (giskard::make-allow-all-collision)))

(defun ensure-custom-goal-input (action-type-or-position arm effort)
  (declare (type (or number keyword) action-type-or-position)
           (type (or keyword list) arm)
           (type (or number null) effort))
  (unless (listp arm)
    (setf arm (list arm)))
  (let* ((bindings
           (car (prolog:prolog
                 `(and (rob-int:robot ?robot)
                       (rob-int:gripper-joint ?robot ,(first arm) ?gripper-joint)
                       (rob-int:joint-lower-limit ?robot ?gripper-joint ?lower-limit)
                       (rob-int:joint-upper-limit ?robot ?gripper-joint ?upper-limit)
                       (rob-int:gripper-meter-to-joint-multiplier ?robot ?mult)))))
         (gripper-joint
           (cut:var-value '?gripper-joint bindings))
         (gripper-lower-limit
           (cut:var-value '?lower-limit bindings))
         (gripper-upper-limit
           (cut:var-value '?upper-limit bindings))
         (gripper-multiplier
           (cut:var-value '?mult bindings)))
    (when (cut:is-var gripper-joint)
      (error "[giskard] Robot gripper joint was not defined."))
    (let ((position
            (etypecase action-type-or-position
              (number
               (cond
                 ((< action-type-or-position gripper-lower-limit)
                  (roslisp:ros-warn (giskard gripper)
                                    "POSITION (~a) cannot be < ~a. Clipping."
                                    action-type-or-position gripper-lower-limit)
                  gripper-lower-limit)
                 ((> action-type-or-position gripper-upper-limit)
                  (roslisp:ros-warn (giskard gripper)
                                    "POSITION (~a) shouldn't be > ~a. Clipping."
                                    action-type-or-position gripper-upper-limit)
                  gripper-upper-limit)
                 (t
                  ;; in case the gripper is commanded in radian or so
                  (* gripper-multiplier action-type-or-position))))
              (keyword
               (ecase action-type-or-position
                 (:open gripper-upper-limit)
                 (:close gripper-lower-limit)
                 (:grip gripper-lower-limit)))))
          (effort
            (or effort
                (etypecase action-type-or-position
                  (number 30.0)
                  (keyword (ecase action-type-or-position
                             (:open 30.0)
                             (:close 30.0)
                             (:grip 15.0)))))))
      (list position effort))))

(defun ensure-custom-goal-reached (box-pose)
 

  )

(defun call-custom-action (&key
                              action-timeout
                              box-pose grasp-type)
  
              (giskard::call-action
               :action-goal (make-custom-action-goal box-pose grasp-type)
               :action-timeout action-timeout))



;;==============================================================================================


(defparameter *gripper-convergence-delta-joint* 0.005 "In meters.")
(defparameter *gripper-gripped-min-position* 0.007 "In meters.")

(defun make-custom-return-action-goal (knob-pose open-drawer)
    ;; (declare  (type cl-transforms-stamped:pose-stamped knob-pose)
    ;;           (type boolean grasp-type))
  (giskard::make-giskard-goal
   :joint-constraints (giskard::make-custom-return-grasp-constraint knob-pose open-drawer)
   :collisions (giskard::make-allow-all-collision)))

(defun ensure-custom-return-goal-input (action-type-or-position arm effort)
  

  )

(defun ensure-custom-return-goal-reached (box-pose)
 

  )

(defun call-custom-return-action (&key
                              action-timeout
                              knob-pose open-drawer)
  ;; (declare  (type cl-transforms-stamped:pose-stamped box-pose)
  ;;           (type boolean grasp-type))
  
  ;; (let* ((position-and-effort
  ;;          (ensure-gripper-goal-input action-type-or-position arm effort))
  ;;        (goal-joint-angle
  ;;          (first position-and-effort))
  ;;        (effort
  ;;          (second position-and-effort)))

    ;; (unless (listp box-pose)
    ;;   (setf box-pose (list box-pose)))

   ;; (mapcar (lambda (box-pose-element)
              (giskard::call-action
               :action-goal (make-custom-return-action-goal knob-pose open-drawer)
               :action-timeout action-timeout)
               ;; :check-goal-function (lambda (result status)
               ;;                        ;; This check is only done after the action
               ;;                        ;; and never before, therefore check
               ;;                        ;; if result and status already exist.
               ;;                        (if (and result status)
               ;;                            (ensure-custom-goal-reached
               ;;                             box-pose)
               ;;                            :goal-not-achieved-yet)))
  )



(defun make-custom-return-grasp-constraint (knob-pose open-drawer
                                     &key avoid-collisions-not-much)
  ;; (declare  (type cl-transforms-stamped:pose-stamped goal-pose))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "MoveDrawer"
   :parameter_value_pair
   (giskard::alist->json-string
    `(("knob_pose"
       . (("message_type" . "geometry_msgs/PoseStamped")
          ("message" . ,(giskard::to-hash-table knob-pose))))
      ("open_drawer"
         . ,open-drawer)
      
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))
;;================================================================================================




(defparameter *gripper-convergence-delta-joint* 0.005 "In meters.")
(defparameter *gripper-gripped-min-position* 0.007 "In meters.")

(defun make-custom-gripper-action-goal (open-gripper)
    ;; (declare  (type cl-transforms-stamped:pose-stamped knob-pose)
    ;;           (type boolean grasp-type))
  (giskard::make-giskard-goal
   :joint-constraints (giskard::make-custom-gripper-constraint open-gripper)
   :collisions (giskard::make-allow-all-collision)))

(defun ensure-custom-gripper-goal-input (action-type-or-position arm effort)
  

  )

(defun ensure-custom-gripper-goal-reached (box-pose)
 

  )

(defun call-custom-gripper-action (&key
                              action-timeout
                                     open-gripper)
  (print open-gripper)
  ;; (break)
  ;; (declare  (type cl-transforms-stamped:pose-stamped box-pose)
  ;;           (type boolean grasp-type))
  
  ;; (let* ((position-and-effort
  ;;          (ensure-gripper-goal-input action-type-or-position arm effort))
  ;;        (goal-joint-angle
  ;;          (first position-and-effort))
  ;;        (effort
  ;;          (second position-and-effort)))

    ;; (unless (listp box-pose)
    ;;   (setf box-pose (list box-pose)))

   ;; (mapcar (lambda (box-pose-element)
              (giskard::call-action
               :action-goal (make-custom-gripper-action-goal open-gripper)
               :action-timeout action-timeout)
               ;; :check-goal-function (lambda (result status)
               ;;                        ;; This check is only done after the action
               ;;                        ;; and never before, therefore check
               ;;                        ;; if result and status already exist.
               ;;                        (if (and result status)
               ;;                            (ensure-custom-goal-reached
               ;;                             box-pose)
               ;;                            :goal-not-achieved-yet)))
  )



(defun make-custom-gripper-constraint (open-gripper
                                     &key avoid-collisions-not-much)
  ;; (declare  (type cl-transforms-stamped:pose-stamped goal-pose))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "MoveGripper"
   :parameter_value_pair
   (giskard::alist->json-string
    `(("open_gripper"
         . ,open-gripper)
      
      ,@(if avoid-collisions-not-much
            `(("weight" . ,(roslisp-msg-protocol:symbol-code
                           'giskard_msgs-msg:constraint
                           :weight_above_ca))
              (("weight" . (roslisp-msg-protocol:symbol-code
                            'giskard_msgs-msg:constraint
                            :weight_below_ca)))))))))
