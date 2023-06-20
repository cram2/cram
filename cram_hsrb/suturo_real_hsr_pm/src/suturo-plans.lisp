(in-package :su-real)

;; @author Luca Krohm
;; @TODO failurehandling
(defun pick-up (&key
                  ((:collision-mode ?collision-mode))
                  ((:collision-object-b ?collision-object-b))
                  ((:collision-object-b-link ?collision-object-b-link))
                  ((:collision-object-a ?collision-object-a))
                  ((:move-base ?move-base))
                  ((:prefer-base ?prefer-base))
                  ((:straight-line ?straight-line))
                  ((:align-planes-left ?align-planes-left))
                  ((:align-planes-right ?align-planes-right))
                  ((:precise-tracking ?precise-tracking))
                  ((:object-type ?object-type))
                  ((:goal-pose ?goal-pose))
                  ((:object-size ?object-size))
                  ((:object-name ?object-name))
                &allow-other-keys)
  "Receives parameters from action-designator, and then executes the corresponding motions"
  (declare (type boolean ?move-base ?prefer-base ?straight-line ?precise-tracking
                 ?align-planes-left ?align-planes-right))

  (cpl:with-retry-counters ((manip-retries 1))
    (cpl:with-failure-handling
        ((common-fail:gripper-closed-completely (e)
           (roslisp:ros-warn (suturo-pickup grasp-object)
                             "Some manipulation failure happened: ~a"
                             e)
           (cpl:do-retry manip-retries
             (roslisp:ros-warn (suturo-pickup grasp-object) "Retrying...")
             (exe:perform (desig:a motion
                        (type :retracting)
                        (collision-mode ?collision-mode)
                        (object-name ?object-name)))
             (su-demos::perc-robot)
             ;; add "looking" to old object-position before perceiving again
             (let* ((?source-object-desig
                      (desig:an object
                                (type ?object-type)))
                    ;; detect object and save the return value
                    (?object-desig
                      (exe:perform (desig:an action
                                             (type detecting)
                                             (object ?source-object-desig)))))
               (roslisp:with-fields 
                   ((?pose
                     (cram-designators::pose cram-designators:data))) 
                   ?object-desig
                 (setf ?goal-pose ?pose)))
             (cpl:retry))))
      
      (let ((?object-height (cl-transforms:z ?object-size)))
        (exe:perform (desig:a motion
                              (type aligning-height)
                              (collision-mode ?collision-mode)
                              (collision-object-b ?collision-object-b)
                              (collision-object-b-link ?collision-object-b-link)
                              (collision-object-a ?collision-object-a)
                              (allow-base ?move-base)
                              (prefer-base ?prefer-base)
                              (straight-line ?straight-line)
                              (align-planes-left ?align-planes-left)
                              (align-planes-right ?align-planes-right)
                              (precise-tracking ?precise-tracking)
                              (goal-pose ?goal-pose)
                              (object-height ?object-height)
                              (object-name ?object-name)))
      
      (exe:perform (desig:a motion
                        (type gripper-motion)
                        (:open-close :open)
                        (effort 0.1)))
      
        (exe:perform (desig:a motion
                              (type reaching)
                              (collision-mode ?collision-mode)
                              (goal-pose ?goal-pose)
                              (object-size ?object-size)
                              (object-name ?object-name)))
        
        (cpl:pursue
          (cpl:seq
            (exe:perform (desig:a motion
                                  (type gripper-motion)
                                  (:open-close :close)
                                  (effort 0.1)))
            (sleep 1)
            (su-demos::call-text-to-speech-action "Managed to grasp the object"))
          (cpl:seq
            (exe:perform
             (desig:an action
                       (type monitoring-joint-state)
                       (joint-name "hand_l_proximal_joint")))
            (su-demos::call-text-to-speech-action "Failed to grasp the object, retrying")
            (sleep 1)
            (cpl:fail 'common-fail:gripper-closed-completely
                      :description "Object slipped")))
          ))
      
      
  (exe:perform (desig:a motion
                        (type :lifting)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (object-name ?object-name)))

  (exe:perform (desig:a motion
                        (type :retracting)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (object-name ?object-name)))
    ))

;; @author Luca Krohm
;; @TODO failurehandling
(defun place (&key
                ((:collision-mode ?collision-mode))
                ((:collision-object-b ?collision-object-b))
                ((:collision-object-b-link ?collision-object-b-link))
                ((:collision-object-a ?collision-object-a))
                ((:move-base ?move-base))
                ((:prefer-base ?prefer-base))
                ((:straight-line ?straight-line))
                ((:align-planes-left ?align-planes-left))
                ((:align-planes-right ?align-planes-right))
                ((:precise-tracking ?precise-tracking))
                ((:goal-pose ?goal-pose))
                ((:object-height ?object-height))
                ((:from-above ?from-above))
                ((:neatly ?neatly))
              &allow-other-keys)
  "Receives parameters from action-designator, and then executes the corresponding motions"
  (declare (type boolean ?move-base ?prefer-base ?straight-line ?precise-tracking
                 ?align-planes-left ?align-planes-right))

  (exe:perform (desig:a motion
                        (type aligning-height)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (goal-pose ?goal-pose)
                        (object-height ?object-height)
                        (from-above ?from-above)))

  (exe:perform (desig:a motion
                        (type placing)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (goal-pose ?goal-pose)
                        (object-height ?object-height)
                        (from-above ?from-above)))

  (when ?neatly
    (exe:perform (desig:a motion
                          (type placing-neatly)
                          (collision-mode ?collision-mode)
                          (collision-object-b ?collision-object-b)
                          (collision-object-b-link ?collision-object-b-link)
                          (collision-object-a ?collision-object-a)
                          (allow-base ?move-base)
                          (prefer-base ?prefer-base)
                          (straight-line ?straight-line)
                          (align-planes-left ?align-planes-left)
                          (align-planes-right ?align-planes-right)
                          (precise-tracking ?precise-tracking)
                          (goal-pose ?goal-pose))))

  (exe:perform (desig:a motion
                        (type gripper-motion)
                        (:open-close :open)
                        (effort 0.1)))
  
  (exe:perform (desig:a motion
                        (type :retracting)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking))))


;; @author Luca Krohm
;; @TODO failurehandling
(defun open-door (&key
                    ((:collision-mode ?collision-mode))
                    ((:collision-object-b ?collision-object-b))
                    ((:collision-object-b-link ?collision-object-b-link))
                    ((:collision-object-a ?collision-object-a))
                    ((:move-base ?move-base))
                    ((:prefer-base ?prefer-base))
                    ((:straight-line ?straight-line))
                    ((:align-planes-left ?align-planes-left))
                    ((:align-planes-right ?align-planes-right))
                    ((:precise-tracking ?precise-tracking))
                    ((:handle-link ?handle-link))
                    ((:joint-angle ?joint-angle))
              &allow-other-keys)
  "Receives parameters from action-designator, and then executes the corresponding motions"
  (declare (type boolean ?move-base ?prefer-base ?straight-line ?precise-tracking
                 ?align-planes-left ?align-planes-right))

  (exe:perform (desig:a motion
                        (type gripper-motion)
                        (:open-close :open)
                        (effort 0.1)))
    
  (exe:perform (desig:a motion
                        (type reaching)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (object-name ?handle-link)))
    
  (exe:perform (desig:a motion
                        (type gripper-motion)
                        (:open-close :close)
                        (effort 0.1)))

  (exe:perform (desig:a motion
                        (type pulling)
                        (arm :left)
                        (collision-object-b-link ?handle-link)
                        (joint-angle ?joint-angle)))

  (exe:perform (desig:a motion
                        (type gripper-motion)
                        (:open-close :open)
                        (effort 0.1)))
    
  (exe:perform (desig:a motion
                        (type :retracting)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)
                        (allow-base ?move-base)
                        (prefer-base ?prefer-base)
                        (straight-line ?straight-line)
                        (align-planes-left ?align-planes-left)
                        (align-planes-right ?align-planes-right)
                        (precise-tracking ?precise-tracking)
                        (tip-link t))))

;; @author Luca Krohm
(defun open-gripper (&key
                     ((:effort ?effort))
                     &allow-other-keys)
  (call-gripper-action (abs ?effort)))

;; @author Luca Krohm
(defun close-gripper (&key
                     ((:effort ?effort))
                     &allow-other-keys)
  (call-gripper-action (* -1 (abs ?effort))))

;; @author Luca Krohm
;; @TODO failurehandling
;; @TODO put the transforms etc into the designator, like its done in cram
(defun su-pour (&key
                  ((:collision-mode ?collision-mode))
                  ((:collision-object-b ?collision-object-b))
                  ((:collision-object-b-link ?collision-object-b-link))
                  ((:collision-object-a ?collision-object-a))
                  ((:object-size ?object-size))
                  ((:target-object ?target-object))
                  ((:target-size ?target-size))
                  ((:target-name ?target-name))
                &allow-other-keys)
  "Receives parameters from action-designator, and then executes the corresponding motions"

  (let* (;; pouring pose relative to the bowl.
         ;; (width of the bowl + pouring object) / -2
         ;; puts the target on the very lefthand side of the bowl
         ;; (height of the bowl + pouring object) / 2
         ;; puts the target just above the upper edge of the bowl
         (?relative-pour-pose (cl-transforms:make-3d-vector
                       0
                       (/ (+ (cl-transforms:y ?target-size)
                             (cl-transforms:y ?object-size))
                          2)
                       (/ (+ (cl-transforms:z ?target-size)
                             (cl-transforms:z ?object-size))
                          2)))
         ;; object pose to object transform
         (?object-transform (cl-tf:lookup-transform cram-tf:*transformer* "base_footprint" ?target-object));; (man-int::get-object-transform ?target-object))
         ;; rel pose to rel transform
         (?rel-pose-transform (cl-tf2::make-pose-stamped
                               "base_footprint" 0
                               ?relative-pour-pose
                               (cl-tf2::make-quaternion 0 0 0 1)))
         ;; moves the bowlpose like specified in ?relative-pour-pose, creating ?pour-pose-transform
         (?pour-pose-transform (cram-tf:apply-transform
                            (cl-tf:lookup-transform cram-tf:*transformer* "map" "base_footprint")
                            (cram-tf:apply-transform ?object-transform
                                                    (cram-tf:pose-stamped->transform-stamped
                                                     ?rel-pose-transform
                                                     "base_footprint"))))
         ;; pour transform to pour pose
         (?pour-pose (cram-tf:transform->pose-stamped
                       "map" 0
                       ?pour-pose-transform)))

    (let ((?height 0.2215))
      (exe:perform (desig:a motion
                            (type aligning-height)
                            (collision-mode ?collision-mode)
                            (goal-pose ?pour-pose)
                            (object-height ?height)
                            (object-name ?target-name))))
    
    (exe:perform (desig:a motion
                          (type reaching)
                          (collision-mode ?collision-mode)
                          (goal-pose ?pour-pose)
                          (object-size ?object-size)
                          (object-name ?target-name)))

    (exe:perform (desig:a motion
                          (type tilting)
                          (tilt-direction "right")
                          (tilt-angle 2.0d0)
                          (collision-mode ?collision-mode)))
                          

    (exe:perform (desig:a motion
                          (type tilting)
                          (tilt-angle 0.0d0)
                          (collision-mode ?collision-mode)))

    
    (exe:perform (desig:a motion
                        (type :retracting)
                        (collision-mode ?collision-mode)
                        (collision-object-b ?collision-object-b)
                        (collision-object-b-link ?collision-object-b-link)
                        (collision-object-a ?collision-object-a)))
    ))      




 ;; not current used
(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform 
    ((object-type (eql :bowl))
     arm
     (grasp (eql :su-top-left)))
  '((-0.01 0.245 0.020)(0 0 0 1)))
