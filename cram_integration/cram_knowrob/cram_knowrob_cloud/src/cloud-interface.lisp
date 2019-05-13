;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :kr-cloud)

(defun replace-all (string part replacement &key (test #'char=))
  "Returns a new string in which all the occurences of the part
is replaced with replacement.
  Taken from Common Lisp Cookbook."
  (with-output-to-string (out)
    (loop with part-length = (length part)
          for old-pos = 0 then (+ pos part-length)
          for pos = (search part string
                            :start2 old-pos
                            :test test)
          do (write-string string out
                           :start old-pos
                           :end (or pos (length string)))
          when pos do (write-string replacement out)
            while pos)))

(defun json-result->string (result-symbol)
  (when result-symbol
    (string-trim "'" (symbol-name result-symbol))))

(defun cloud-prolog-simple (query)
  (declare (type string query))
  (let* ((escaped-query (replace-all query "'" "\\'"))
         (query-answer (json-prolog:prolog-simple-1
                        (format nil "send_prolog_query('~a', @(false), ID)." escaped-query)
                        :mode 1
                        :package :kr-cloud)))
    (print (format nil "send_prolog_query('~a', @(false), ID)." escaped-query))
    (if query-answer
        (let* ((answer-id (cut:var-value '?id (car query-answer)))
               (id (json-result->string answer-id)))
          (json-prolog:prolog-simple-1 (format nil "send_next_solution('~a')" id)
                                       :mode 1
                                       :package :kr-cloud)
          (let ((result
                  (cut:var-value '?res
                                 (car
                                  (json-prolog:prolog-simple-1
                                   "read_next_prolog_query(RES)."
                                   :mode 1
                                   :package :kr-cloud)))))
            (json-prolog:prolog-simple-1 (format nil "send_finish_query('~a')" id)
                                         :mode 1
                                         :package :kr-cloud)
            (if (cut:is-var result)
                (progn
                  (roslisp:ros-warn (cloud-interface query) "Query didn't succeed!")
                  NIL)
                (when result
                  (yason:parse
                   (json-result->string result)
                   :object-as :alist)))))
        (roslisp:ros-warn (cloud-interface query) "Query didn't succeed!"))))

(defmacro getassoc (key alist)
  `(cdr (assoc ,key ,alist :test #'equal)))


(defun initialize-jsk-cloud-connection ()
  (and (json-prolog:prolog `("register_ros_package" "knowrob_cloud_logger"))
       (print "registered cloud_logger package")
       (json-prolog:prolog `("cloud_interface"
                             "https://133.11.216.21"
                             "/home/gaya/jsk.pem"
                             "apcDwBVxVR3neXBJJpEyVMFRxgliUCIkkLaC8IB0jsdA3oDccTlTLgynYpEPnbEd"))
       (print "initialized https connection")
       (json-prolog:prolog `("start_user_container"))
       (print "started docker container")
       (json-prolog:prolog `("connect_to_user_container"))
       (print "initialization complete")))

(defun initialize-iai-cloud-connection ()
  (and (json-prolog:prolog `("register_ros_package" "knowrob_cloud_logger"))
       (print "registered cloud_logger package")
       ;; (json-prolog:prolog `("cloud_interface"
       ;;                       "https://192.168.101.42"
       ;;                       "/home/ease/asil.pem"
       ;;                       "MxtU9V2cdstw3ocKXbicBGp7fAeLNxjIvcmY4CJV96DeZd7obfgvw0mR3X5j8Yrz"))
       (json-prolog:prolog-simple "cloud_interface('https://192.168.101.42','/home/ease/asil.pem','MxtU9V2cdstw3ocKXbicBGp7fAeLNxjIvcmY4CJV96DeZd7obfgvw0mR3X5j8Yrz').")
       (print "initialized https connection")
       (json-prolog:prolog `("start_user_container"))
       (print "started docker container")
       (json-prolog:prolog `("connect_to_user_container"))
       (print "initialization complete")))

(defun load-episodes (episode-ids &key (old-db-or-new :new))
  (declare (type list episode-ids))
  (let* ((episode-ids-yason-string
           (let ((stream (make-string-output-stream)))
             (yason:encode (mapcar (lambda (id) (format nil "episode~a" id))
                                   episode-ids)
                           stream)
             (get-output-stream-string stream)))
         (episode-ids-string (replace-all episode-ids-yason-string "\"" "'")))
    (cloud-prolog-simple
     "register_ros_package('knowrob_learning').")
    (cloud-prolog-simple
     "owl_parse('package://knowrob_srdl/owl/PR2.owl').")
    (ecase old-db-or-new
      (:new
       (cloud-prolog-simple
        "mng_db('Bring-Can-From-Fridge_pr2-prepare-breakfast_0').")
       (cloud-prolog-simple
        (format nil
                "load_experiments('/episodes/Bring-Can-From-Fridge/pr2-prepare-breakfast_0/', ~
                                  ~a, 'eus.owl')."
                episode-ids-string)))
      (:old
       (cloud-prolog-simple
        "mng_db('Bring-Can-From-Fridge_pr2-bring-can_0')")
       (cloud-prolog-simple
        (format nil
                "load_experiments('/episodes/Bring-Can-From-Fridge/pr2-bring-can_0/', ~
                                  ~a, 'eus.owl')."
                episode-ids-string))))
    (cloud-prolog-simple
     "owl_parse('package://knowrob_cloud_logger/owl/room73b2.owl').")
    ;; (cloud-prolog-simple
    ;;  "rdf_register_ns(jsk, 'http://knowrob.org/kb/room73b2.owl#', [keep(true)]).")
    (cloud-prolog-simple
     "owl_parse('package://iai_semantic_maps/owl/kitchen.owl').")))
;; , rdf_register_ns(iai, 'http://knowrob.org/kb/IAI-kitchen.owl#', [keep(true)])

;; grasping object: GraspExecutionCan
;; pregrasp pose: PreGraspPose

(defun generate-transform-stamped (pose-list child-frame stamp)
  (destructuring-bind ((x y z) (q1 q2 q3 w))
      pose-list
    (cl-transforms-stamped:make-transform-stamped
     cram-tf:*fixed-frame*
     child-frame
     stamp
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

(defun generate-pose-stamped (pose-list stamp)
  (destructuring-bind ((x y z) (q1 q2 q3 w))
      pose-list
    (cl-transforms-stamped:make-pose-stamped
     cram-tf:*fixed-frame*
     stamp
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

(defun robot-pose-before-action (knowrob-task-context)
  (let ((bindings
          (cloud-prolog-simple
           (format nil
                   "entity(Act, [an, action, ['task_context', '~a']]), occurs(Act, [Begin,End]), mng_lookup_transform('map', '~a', Begin, Pose16), matrix_translation(Pose16, T), matrix_rotation(Pose16, R), =([T, R], Pose)."
                   knowrob-task-context
                   cram-tf:*robot-base-frame*))))
    (generate-transform-stamped
     (getassoc "Pose" bindings)
     cram-tf:*robot-base-frame*
     (getassoc "Begin" bindings))))

;; handle: IAIFridgeDoorHandle
;; hinge: HingedJoint

(defun semantic-map-object-transform (knowrob-class-name)
  (let ((bindings
          (cloud-prolog-simple
           (format nil
                   "owl_individual_of(Object, knowrob:'~a'), current_object_pose(Object, [X, Y, Z, Q1, Q2, Q3, W]), =(Pose, [[X, Y, Z], [Q1, Q2, Q3, W]])."
                   knowrob-class-name))))
    (generate-transform-stamped
     (getassoc "Pose" bindings)
     knowrob-class-name;; (string-trim '(#\') (getassoc "Object" bindings))
     0.0)))

;; opening: OpenFridge
;; pushing-open: SwipeFridgeDoor
;; closing: CloseFridge

(defun local-semantic-map-object-transform (knowrob-class-name)
  (let ((bindings
          (car (json-prolog:prolog-simple-1
                (format nil
                        "owl_individual_of(O, knowrob:'~a'), current_object_pose(O, [X, Y, Z, Q1, Q2, Q3, W]), =(P, [[X, Y, Z], [Q1, Q2, Q3, W]])."
                        knowrob-class-name)
                :mode 1
                :package :kr-cloud))))
    (generate-transform-stamped
     (cut:var-value '?p bindings)
     knowrob-class-name ;; (string-trim '(#\') (symbol-name (cut:var-value '?o bindings)))
     0.0)))

(defun arm-used-in-action (knowrob-task-context)
  (let* ((part-binding
           (string-trim
            '(#\')
            (getassoc "Part"
                      (cloud-prolog-simple
                       (format nil
                               "entity(Act, [an, action, ['task_context', '~a']]), ~
                                rdf_has(Act, knowrob:'bodyPartUsed', literal(type(_, Part)))."
                               knowrob-task-context)))))
         (part-binding-in-lisp
           (cdr (assoc part-binding
                       '(("RARM" . :right)
                         ("LARM" . :left))
                       :test #'string=))))
    (if part-binding-in-lisp
        part-binding-in-lisp
        (error "Arm used in action can only be LARM or RARM. We have: ~a"
               part-binding))))

(defun arm-for-grasping ()
  (getassoc "Part"
            (cloud-prolog-simple
             "owl_individual_of(Grasp, knowrob:'Grasp'), rdf_has(Grasp, knowrob:'bodyPartUsed', literal(type(_, Part))).")))

(defun robot-gripper-pose-before-action (arm knowrob-task-context)
  (let ((bindings
          (cloud-prolog-simple
           (format nil
                   "entity(Act, [an, action, ['task_context', '~a']]), occurs(Act, [Begin,End]), belief_at(robot('~a',M), Begin), matrix_rotation(M,R), matrix_translation(M,T), =([T, R], Pose)."
                   knowrob-task-context
                   (ecase arm
                     (:left cram-tf:*robot-left-tool-frame*)
                     (:right cram-tf:*robot-right-tool-frame*))))))
    (generate-transform-stamped
     (getassoc "Pose" bindings)
     (ecase arm
       (:left cram-tf:*robot-left-tool-frame*)
       (:right cram-tf:*robot-right-tool-frame*))
     (getassoc "Begin" bindings))))

;; "MoveFridgeHandle"

(defun gripper-trajectory-during-action (arm knowrob-task-context)
  (let ((bindings
          (cloud-prolog-simple
           (format nil
                   "entity(Act, [an, action, ['task_context', '~a']]), sample_trajectory(Act, '~a', Samples, 0.25)."
                   knowrob-task-context
                   (ecase arm
                     (:left cram-tf:*robot-left-tool-frame*)
                     (:right cram-tf:*robot-right-tool-frame*))))))
    (mapcar (lambda (position-and-orientation)
              (generate-transform-stamped position-and-orientation
                                          (ecase arm
                                            (:left cram-tf:*robot-left-tool-frame*)
                                            (:right cram-tf:*robot-right-tool-frame*))
                                          0.0))
            (getassoc "Samples" bindings))))

(defun gripper-projected-trajectory-during-action (arm knowrob-task-context)
  (let ((bindings
          (cloud-prolog-simple
           ;; (format nil
           ;;         "entity(OpenFridgeTask, [an, action, ['task_context', 'ReachAndOpenFridgeDoor']]), owl_individual_of(JskKitchenDoor, 'http://knowrob.org/kb/knowrob.owl#IAIFridgeDoor'), rdf_has(JskKitchenDoor, 'http://knowrob.org/kb/knowrob.owl#describedInMap', 'http://knowrob.org/kb/room73b2.owl#room73b2_1'), owl_individual_of(IaiKitchenDoor, 'http://knowrob.org/kb/knowrob.owl#IAIFridgeDoor'), rdf_has(IaiKitchenDoor, 'http://knowrob.org/kb/knowrob.owl#describedInMap', 'http://knowrob.org/kb/IAI-kitchen.owl#IAIKitchenMap_PM580j'), estimate_action_by_comparing(OpenFridgeTask, JskKitchenDoor, IaiKitchenDoor, TargetAction), apply_rule_for_adapt(OpenFridgeTask, TargetAction, RuleOut), owl_individual_of(R, knowrob:'AdaptingEpisodicMemoryData'), owl_individual_of(Door, knowrob:'IAIFridgeDoor'), rdf_has(Door, knowrob:'describedInMap', 'http://knowrob.org/kb/room73b2.owl#room73b2_1'), entity(Act, [an, action, ['task_context', 'MoveFridgeHandle']]), project_arch_trajectory_samples(Act, '/r_gripper_tool_frame', Door, R, Samples, 0.25)."
           ;;         ;; knowrob-task-context
           ;;         ;; (ecase arm
           ;;         ;;   (:left cram-tf:*robot-left-tool-frame*)
           ;;         ;;   (:right cram-tf:*robot-right-tool-frame*))
           ;;         )
           "read_arch_traj(Samples).")))
    (mapcar (lambda (position-and-orientation)
              (generate-transform-stamped position-and-orientation
                                          (ecase arm
                                            (:left cram-tf:*robot-left-tool-frame*)
                                            (:right cram-tf:*robot-right-tool-frame*))
                                          0.0))
            (getassoc "Samples" bindings))))


(defun generate-distribution-files ()
  (kr-cloud::cloud-prolog-simple
   "findall(FeatureArr, (entity(_Act, [an, action, ['task_context', 'PerceiveAndOpenFridgeDoor']]), get_divided_subtasks_with_goal(_Act, 'RelocateSelf', _SuccInst, _), task_start(_SuccInst, B), belief_at(robot('base_link', _RobotPose), B), matrix_translation(_RobotPose, P), matrix_rotation(_RobotPose, R), append(P, R, FeatureList), jpl_list_to_array(FeatureList,FeatureArr)), FeatureSet), jpl_list_to_array(FeatureSet,FeatureArrArr), generate_feature_files(FeatureArrArr, 'positive.csv').")

  (kr-cloud::cloud-prolog-simple
   "findall(FeatureArr, (entity(_Act, [an, action, ['task_context', 'PerceiveAndOpenFridgeDoor']]), get_divided_subtasks_with_goal(_Act, 'RelocateSelf', _, _Insts), member(_I, _Insts), task_start(_I, B), belief_at(robot('base_link', _RobotPose), B), matrix_translation(_RobotPose, Position), matrix_rotation(_RobotPose, Rot), append(Position, Rot, FeatureList), jpl_list_to_array(FeatureList,FeatureArr)), FeatureSet), jpl_list_to_array(FeatureSet,FeatureArrArr), generate_feature_files(FeatureArrArr, 'negative.csv')."))

(defun robot-pose-distribution ()
  ;; (return-from robot-pose-distribution
  ;;   (values
  ;;    (cl-transforms-stamped:make-transform-stamped
  ;;     "map" "base_footprint" 0.0
  ;;     (cl-transforms:make-3d-vector 4.51077127456665d0 8.1215238571167d0 0)
  ;;     (cl-transforms:make-identity-rotation))
  ;;    '(0.022423092 -0.0042169616 -0.0042169616 0.007500598)))
  (let* ((bindings
           (kr-cloud::cloud-prolog-simple
            "get_likely_location('/home/ros/user_data/positive.csv', 1, '/home/ros/user_data/negative.csv', 1, Mean, Cov)."))
         (mean-binding (getassoc "Mean" bindings))
         (covariance-binding (getassoc "Cov" bindings)))
    (values
     (cl-transforms-stamped:make-transform-stamped
      cram-tf:*fixed-frame*
      cram-tf:*robot-base-frame*
      0.0
      (cl-transforms:make-3d-vector (first mean-binding) (second mean-binding) 0)
      (cl-transforms:axis-angle->quaternion
       (cl-transforms:make-3d-vector 0 0 1)
       (cma:degrees->radians (third mean-binding))))
     (make-array '(2 2)
                 :initial-contents
                 (list (subseq covariance-binding 0 2)
                       (subseq covariance-binding 3 5))))))


;; entity(Act, [an, action, ['task_context', 'MoveFridgeHandle']]),
;; occurs(Act, [Begin,End]),
;; color_directed_trajectory('/r_gripper_tool_frame', Begin, End, 0.1).

;; owl_individual_of(Obj, knowrob:'IAIFridgeDoorHandle'),
;; current_object_pose(Obj, [X, Y, Z, W, Q1, Q2, Q3]),
;; =(T, [X, Y, Z]), =(R, [W, Q1, Q2, Q3]),
;; show(cube(base), [ pose(T,R), scale([0.1,0.1,0.1]),
;; color([0.3,0.3,0.1])]).
