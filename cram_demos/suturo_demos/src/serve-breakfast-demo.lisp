(in-package :su-demos)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; LUCA TODO
;; rewrite~/SUTURO/SUTURO_WSS/planning_ws/src/cram/cram_external_interfaces/cram_giskard/src/collision-scene.lisp to function without using the bulletworld as reasoning tool, but rather use knowledge as reasoning tool. For example "update-object-pose-in-collision-scene"

;; Rewrite or duplicate and change the following functions (in order to preserve the original implementation in case its vital to other plans):
;; make-giskard-environment-request, uses btr in on the very bottom

;; reset-collision-scene

;; update-object-pose-in-collision-scene

;; add-object-to-collision-scene

;; detach-object-in-collision-scene

;; attach-object-to-arm-in-collision-scene

;; full-update-collision-scene

;; (cram-occasions-events:on-event
;;      (make-instance 'cram-plan-occasions-events:object-perceived-event
;;                          :object-designator desig
;;                          :perception-source :whatever))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defparameter *objects* '(:CerealBox :milk :spoon :bowl))

(defun serve-breakfast-demo()
  ;;(call-text-to-speech-action "Starting demo")

  (with-knowledge-result ()
        `("init_serve_breakfast")
      (print "Serving Breakfast plan started."))
  
  (park-robot)

  ;; (call-text-to-speech-action "Positioning in front of shelf")
  ;; Calls knowledge to receive coordinates of the shelf pose, then relays that pose to navigation
  (with-knowledge-result (shelf table)
      `(and ("has_urdf_name" object1 "open_shelf:shelf:shelf_base_center")
            ("object_rel_pose" object1 "perceive" shelf)
            ("has_urdf_name" object2 "left_table:table:table_front_edge_center")
            ("object_rel_pose" object2 "perceive" (list ("direction" "-y")) table))
    (move-hsr (make-pose-stamped-from-knowledge-result shelf))
    (sleep 1)
  
  

    ;; (park-robot)
    
    ;; (let ((?handle-link "iai_kitchen/shelf:shelf:shelf_door_left:handle")
    ;;       (?joint-angle -1.2))

    ;;   ;;(call-text-to-speech-action "Opening shelf door")
    ;;   (exe:perform (desig:an action
    ;;                          (type opening-door)
    ;;                          (handle-link ?handle-link)
    ;;                          (joint-angle ?joint-angle)
    ;;                          (tip-link t)
    ;;                          (collision-mode :allow-all))))

    ;; (park-robot)

    ;; (move-hsr (make-pose-stamped-from-knowledge-result shelf))

  ;; (park-robot)
    
  (let* ((?source-object-desig
           (desig:all object
                      (type :breakfast)))
         (?object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig))))
         (?current-object nil)
         (?found-cereal nil))
    (with-knowledge-result (nextobject)
        `("next_object" nextobject)
      (print "next object:")
      (print nextobject)
      ;; (break)
      (loop until (and (string= nextobject "I")
                       (eq ?found-cereal nil))
            do (let* ((?target-pose (get-target-pos nextobject)))
                   (with-knowledge-result (result)
                       `("next_object" result)
                     (setf ?current-object nextobject)
                     (setf nextobject result))
                 (print ?current-object)
                 (print nextobject)
                 ;; (break)
                   (progn
                     (move-hsr (make-pose-stamped-from-knowledge-result shelf))
                     (cond
                       ((search "CerealBox" ?current-object) (setf ?found-cereal ?current-object))
                       (t
                        (when (and (string= nextobject "I")
                                   (string= ?current-object "I")
                                   (not (eq ?found-cereal nil)))
                          (print "Inside found-cereal to current-object")
                          ;; (break)
                          (setf ?current-object ?found-cereal)
                          (setf ?found-cereal nil))
                        (cond
                          ((search "Spoon" ?current-object) (wait-for-human-signal))
                          ((search "Bowl" ?current-object) (wait-for-human-signal))
                          (t 
                           (with-knowledge-result (frame pose)
                               `(and ("object_shape_workaround" ,?current-object frame _ _ _)
                                     ("object_pose" ,?current-object pose))
                             
                         
                             
                             ;; picks up the object by executing the following motions:
                             ;; - opening the gripper
                             ;; - reaching for the object
                             ;; - closing the gripper, thus gripping the object
                             ;; - lifting the object
                             ;; - retracting the arm to retrieve the object from, for example, a shelf
                             ;;(call-text-to-speech-action "Picking up the object Cereal-Box")
                             (let ((?object-size (get-target-size ?current-object))
                                   (?object-pose (make-pose-stamped-from-knowledge-result pose)))
                               (exe:perform (desig:an action
                                                      (type picking-up)
                                                      (goal-pose ?object-pose)
                                                      (object-size ?object-size)
                                                      (collision-mode :allow-all)))))))
                       (park-robot)
                       
                       ;;(call-text-to-speech-action "Moving to target location")
                       ;; Calls knowledge to receive coordinates of the dinner table pose, then relays that pose to navigation
                       (move-hsr (make-pose-stamped-from-knowledge-result table))
                       
                       ;; places the object by executing the following motions:
                       ;; - preparing to place the object, by lifting the arm to an appropriate ?object
                       ;; - placing the object
                       ;; - opening the gripper, thus releasing the object
                       (unless (search "CerealBox" ?current-object)
                         (let ((?object-height (cl-transforms:z (get-target-size ?current-object)))
                               (?from-above (get-frontal-placing ?current-object))
                               (?neatly (get-neatly-placing ?current-object)))
                           ;;(call-text-to-speech-action "Placing object Cereal-Box")
                           ;; ?frontal-placing and ?neatly are currently the same for each object, thats why i just use the same function until after the milestone 
                           (exe:perform (desig:an action
                                                  (type :placing)
                                                  (goal-pose ?target-pose)
                                                  (object-height ?object-height)
                                                  (from-above ?from-above)
                                                  (neatly ?neatly)
                                                  (collision-mode :allow-all)))
                           (su-demos::with-knowledge-result ()
                               `("object_pose" ,?current-object ,(reformat-stamped-pose-for-knowledge (get-object-pos ?current-object)))
                           (park-robot))))))))))

    (print "stop")
    ;; (break)
    (with-knowledge-result (bowlframe)
        `(and ("has_type" bowlname ,(transform-key-to-string :bowl))
              ("object_shape_workaround" bowlname bowlframe _ _ _))
      (let ((?object-size (get-target-size ?current-object))
            (?bowl-size (get-target-size bowlframe))
            (?cereal-target-pose (get-target-pos ?current-object))
            (?milk-target-pose (get-object-pos "Milk"))
            (?bowl-frame bowlframe))
        (exe:perform (desig:an action
                             (type su-pouring)
                             (target-object ?bowl-frame)
                             (object-size ?object-size)
                             (target-size ?bowl-size)
                             (collision-mode :allow-all)))
        (park-robot)
        (move-hsr (make-pose-stamped-from-knowledge-result table))
        ;;(call-text-to-speech-action "Placing object Cereal-Box")
        (let ((?object-height (cl-transforms:z ?object-size)))
          (exe:perform (desig:an action
                                 (type :placing)
                                 (goal-pose ?cereal-target-pose)
                                 (object-height ?object-height)
                                 (neatly T)
                                 (collision-mode :allow-all))))

        (park-robot)
          ;; Calls knowledge to receive coordinates of the dinner table pose, then relays that pose to navigation
        (move-hsr (make-pose-stamped-from-knowledge-result table))

          
        (let ((?object-size (get-target-size "Milk")))
          (exe:perform (desig:an action
                                 (type picking-up)
                                 (goal-pose ?milk-target-pose);; ?milk-pose)
                                 (object-size ?object-size)
                                 (collision-mode :allow-all)))
          (park-robot)
        
          ;;(call-text-to-speech-action "Moving to target location")
          ;; Calls knowledge to receive coordinates of the dinner table pose, then relays that pose to navigation
          (move-hsr (make-pose-stamped-from-knowledge-result table))
        
          (exe:perform (desig:an action
                                 (type su-pouring)
                                 (target-object ?bowl-frame)
                                 (object-size ?object-size)
                                 (target-size ?bowl-size)
                                 (collision-mode :allow-all)))
        
        (park-robot)
        (move-hsr (make-pose-stamped-from-knowledge-result table))
        ;;(call-text-to-speech-action "Placing object Cereal-Box")
        (let ((?object-height (cl-transforms:z ?object-size)))
          (exe:perform (desig:an action
                                 (type :placing)
                                 (goal-pose ?milk-target-pose)
                                 (object-height ?object-height)
                                 (neatly T)
                                 (collision-mode :allow-all))))
          (park-robot)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;; Hardcoded stuff for debugging ;;;;;;;;;;;;

(defun park-robot ()
  "Default pose"
  (call-take-pose-action 0 0 0 0 0 -1.5 -1.5 0)
  (exe:perform (desig:a motion
                        (type gripper-motion)
                        (:open-close :close)
                        (effort 0.1))))

(defun perc-robot ()
  "Default pose"
  (call-take-pose-action 0 0 -0.65 0.25 0 -1.5 -1.5 0))

(defun wait-robot ()
  "Default pose"
  (call-take-pose-action 0 0 0 0 0 0 -1.5 0))

(defun take-new-default1 ()
  "Potential alternatives to the default pose"
  (call-take-pose-action 0 0 0 0.3 -2.6 0 1 0))

(defun take-new-default2 ()
  "Potential alternatives to the default pose"
  (call-take-pose-action 0 0 0 0.3 -2.6 1.5 -1.5 0.5))

(defun nav-zero-pos ()
  "Starting pose in IAI office lab"
  (let ((vector (cl-tf2::make-3d-vector 0 0 0))
        (rotation (cl-tf2::make-quaternion 0 0 0 1)))
    (move-hsr (cl-tf2::make-pose-stamped "map" 0 vector rotation))))

(defun get-shelf-pos ()
  (cl-tf2::make-pose-stamped
   "map" 0
   (cl-tf2::make-3d-vector 0.01 0.95 0)
   (cl-tf2::make-quaternion 0 0 1 1)))

(defun get-table-pos ()
  (cl-tf2::make-pose-stamped
   "map" 0
   (cl-tf2::make-3d-vector 0.7 -0.95 0)
   (cl-tf2::make-quaternion 0 0 0 1)))

(defun get-target-pos (obj-name)
  (cond
      ((search "Cereal" obj-name)  (cl-tf2::make-pose-stamped
                                    "map" 0
                                    (cl-tf2::make-3d-vector 2.0 -0.25 0.7)
                                    (cl-tf2::make-quaternion 0 0 0 1)))

      ((search "Milk" obj-name)  (cl-tf2::make-pose-stamped
                                    "map" 0
                                    (cl-tf2::make-3d-vector 2.0  -0.1 0.7)
                                    (cl-tf2::make-quaternion 0 0 0 1)))

      ;; ((search "Spoon" obj-name)  (cl-tf2::make-pose-stamped
      ;;                               "map" 0
      ;;                               (cl-tf2::make-3d-vector 2.05 0.3 0.7)
      ;;                               (cl-tf2::make-quaternion 0 0 0 1)))

      ((search "Bowl" obj-name)  (cl-tf2::make-pose-stamped
                                    "map" 0
                                    (cl-tf2::make-3d-vector 2.0 0.15 0.7)
                                    (cl-tf2::make-quaternion 0 0 0 1)))))

(defun get-object-pos (obj-name)
  (cond
      ((search "Cereal" obj-name)  (cl-tf2::make-pose-stamped
                                    "map" 0
                                    (cl-tf2::make-3d-vector 2.0 -0.25 0.81)
                                    (cl-tf2::make-quaternion 0 0 0 1)))

      ((search "Milk" obj-name)  (cl-tf2::make-pose-stamped
                                    "map" 0
                                    (cl-tf2::make-3d-vector 2.0 -0.1 0.8)
                                    (cl-tf2::make-quaternion 0 0 0 1)))

      ;; ((search "Spoon" obj-name)  (cl-tf2::make-pose-stamped
      ;;                               "map" 0
      ;;                               (cl-tf2::make-3d-vector 2.05 0.3 0.7)
      ;;                               (cl-tf2::make-quaternion 0 0 0 1)))

      ((search "Bowl" obj-name)  (cl-tf2::make-pose-stamped
                                    "map" 0
                                    (cl-tf2::make-3d-vector 2.0 0.15 0.75)
                                    (cl-tf2::make-quaternion 0 0 0 1)))))


(defun get-target-size (obj-name)
  (cond
      ((search "Cereal" obj-name) (cl-tf2::make-3d-vector 0.14 0.06 0.225))
      ((search "Milk" obj-name) (cl-tf2::make-3d-vector 0.09 0.06 0.2))
      ;; ((search "Spoon" obj-name) (cl-tf2::make-3d-vector 0.16 0.06 0.215))
      ((search "Bowl" obj-name) (cl-tf2::make-3d-vector 0.16 0.16 0.05))))
      
       
(defun get-frontal-placing (obj-name)
  (cond
      ((search "Cereal" obj-name) nil)
      ((search "Milk" obj-name) nil)
      ;; ((search "Spoon" obj-name) (cl-tf2::make-3d-vector 0.16 0.06 0.215))
      ((search "Bowl" obj-name) T)))

(defun get-neatly-placing (obj-name)
  (cond
      ((search "Cereal" obj-name) T)
      ((search "Milk" obj-name) T)
      ;; ((search "Spoon" obj-name) (cl-tf2::make-3d-vector 0.16 0.06 0.215))
      ((search "Bowl" obj-name) nil)))




    
  
(defun pouring-test ()
  (let* ((?source-object-desig
           (desig:an object
                     (type bowl)))
         (?object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig))))
         (?object-size1 (cl-tf2::make-3d-vector 0.16 0.16 0.05))
         (?object-size2 (cl-tf2::make-3d-vector 0.06 0.12 0.22))
         (?new-origin (cl-transforms:make-3d-vector
                       (/ (+ (cl-transforms:x ?object-size1)
                             (cl-transforms:x ?object-size2))
                          -2)
                       0
                       (/ (+ (cl-transforms:z ?object-size1)
                             (cl-transforms:z ?object-size2))
                          2)))
         (?object-transform (man-int::get-object-transform ?object-desig))
         (?temp-transform (cl-tf2::make-pose-stamped
                           "base_footprint" 0
                           ?new-origin
                           (cl-tf2::make-quaternion 0 0 0 1)))
         (?reach-transform (cram-tf:apply-transform
                            (cl-tf:lookup-transform cram-tf:*transformer* "map" "base_footprint")
                            (cram-tf:apply-transform ?object-transform
                                                    (cram-tf:pose-stamped->transform-stamped
                                                     ?temp-transform
                                                     "base_footprint"))))
         (?reach-pose (cram-tf:transform->pose-stamped
                       "map" 0
                       ?reach-transform)))
    ?reach-pose))



;; Idea:
;; Change Reaching to approaching to generalize that kind of motion.
;; Planning also give the "context" to manipulation, so manipulation can differentiate
;; between for example, picking up and pouring

(defun wait-for-human-signal ()
  (cpl:seq
    (exe:perform (desig:a motion
                          (type gripper-motion)
                          (:open-close :open)
                          (effort 0.1)))
    (wait-robot)
    (call-text-to-speech-action "Please give me the object")
    (exe:perform
             (desig:an action
                       (type monitoring-joint-state)
                       (joint-name "wrist_flex_joint")))
    (call-text-to-speech-action "Thank you")
    (exe:perform (desig:a motion
                          (type gripper-motion)
                          (:open-close :close)
                          (effort 0.1)))))

(defun luca-test (name pose)
  (cram-occasions-events:on-event
                 (make-instance 'cram-plan-occasions-events:object-detached-robot-knowrob
                   :name name
                   :pose pose)))



