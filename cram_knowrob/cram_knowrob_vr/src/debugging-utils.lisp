;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
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

;;; Contains functions which are usefull for debugging any errors within the code

(in-package :kvr)

;;NOTE deprecated. Doesn't work anymore
(defun reset-simulation ()
  "Resets the simulation and belief state. Re-spawns the objects at their
initial position."
  (cram-occasions-events:clear-belief)
  (spawn-urdf-items))

;; killing object from bullet world. ex: 'axes
(defun kill-obj (object)
  "Removes an object from the bullet world."
  (btr-utils:kill-object object))

(defun make-transform-hand-std-pr2 (object)
  "Make a transform from human hand to the standart pr2"
  (cl-tf:transform* (query-hand-location-by-object-type object "Start")
                    (human-to-robot-hand-transform)
                    cram-pr2-description::*standard-to-pr2-gripper-transform*))

(defun get-robot-in-map-pose ()
  "Get the position of the robot within the map frame."
  (cl-tf:transform->transform-stamped "map" "base_footprint" 0.0
                                      (cl-tf:pose->transform
                                       (btr:pose
                                        (btr:get-robot-object)))))

;; look up transfrom from tf. ex: "l_wrist_roll_link" "l_gripper_l_finger_tip_link"
(defun lookup-tf-transform (parent_frame child_frame)
  "Looks up the tf transform."
  (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
    (cram-tf::lookup-transform cram-tf::*transformer* parent_frame child_frame)))


;;- more bullet checking/utils functions
(defun check-obj-in-world (object-name)
  "Check if the object is in the current world."
  (btr:object btr:*current-bullet-world* object-name))


(defun run-simulation-physics ()
  "simulates the world for a second."
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:simulate ?world 10))))

(defun check-stability-of-sim ()
  "checks if the simulation is stable, or if run for a longer time, some objects would change their position. If the result is anything but NIL, the world is stable."
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:simulate ?world 100))))


(defun set-axes (object)
  "Sets the axes to the robots hands and to the position of the grasping pose of the human. "
  (let* ((transf_r)
         (transf_l))
    (setq transf_r (car
                    (cram-projection::projection-environment-result-result
                     (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
                       (cram-tf::lookup-transform cram-tf::*transformer* "map" "r_gripper_r_finger_tip_link" )))))
    (setq transf_l (car
                    (cram-projection::projection-environment-result-result
                     (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
                       (cram-tf::lookup-transform cram-tf::*transformer* "map" "l_gripper_l_finger_tip_link" )))))
    
    (setq transf_r
          (cl-tf:make-transform
           (cl-tf:translation transf_r)
           (cl-tf:rotation transf_r)))
    (move-object transf_r 'axes)
    (setq transf_l
          (cl-tf:make-transform
           (cl-tf:translation transf_l)
           (cl-tf:rotation transf_l)))
    
    (move-object transf_r 'axes)
    (move-object transf_l 'axes2)
    (move-object
     (query-hand-location-by-object-type
      (object-type-filter-prolog object) "Start")
     'axes3)))


(defun reset-robot ()
  (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
    (cpl:top-level
      (cpl:seq
        (exe:perform
         (desig:an action
                   (type positioning-arm)
                   (left-configuration park)
                   (right-configuration park)))))))


;;if in back 'cereal-5
(defun is-in-view (name-of-object)
  "Checks if the object is in view of the robot.
NAME-OF-OBJECT: The name of the object instance, for which it should be checked if it is still in view. "
  (prolog:prolog `(and (btr:bullet-world ?world)
                              (cram-robot-interfaces:robot ?robot)
                              (btr:visible ?world ?robot ,name-of-object))))


;; query for a pose, CAR the resulting list, pass the transform to this function 
(defun spawn-unreal-arrow (pose arrow-name &optional (color '(1 0 1)))
  "spawns an arrow object at the given `pose' with the given `name'.
Applies the unreal world/semantic map offset to the pose.
`color' is optionaly a list of r g b values. Default is pink '(1 0 1)."
  (let ((pose-with-offset
          (cl-tf:make-pose
           (cl-tf:make-3d-vector
            ;; Add the offset used to offset the semantic map also onto the pose
            (+ (cl-tf:x (cl-tf:origin pose)) *semantic-map-offset-x*)
            (+ (cl-tf:y (cl-tf:origin pose)) *semantic-map-offset-y*)
            (cl-tf:z (cl-tf:origin pose)))
           (cl-tf:orientation pose))))
    ;; spawn arrow
    (btr-utils:spawn-object (intern arrow-name) :arrow
                            :world btr:*current-bullet-world*
                            :mass 1.0
                            :color color
                            :pose pose-with-offset)))

(defvar *arrow-z-offset* 0.2) ;; Z offset so that arrow won't spawn in the floor
(defun spawn-btr-arrow (pose arrow-name &optional (color '(1 0 1)))
  "spawns an arrow object at the given `pose' with the given `name'.
`color' is optionaly a list of r g b values. Default is pink '(1 0 1)."
  ;; spawn arrow
  (let* ((final-arrow-pose
           (cl-tf:make-pose
            (cl-tf:make-3d-vector
             (cl-tf:x (cl-tf:origin pose))
             (cl-tf:y (cl-tf:origin pose))
             ;; (+ (cl-tf:z (cl-tf:origin pose)) *arrow-z-offset*
                1.0)
            (cl-tf:orientation pose))))
    
    (btr-utils:spawn-object (intern arrow-name) :arrow
                            :world btr:*current-bullet-world*
                            :mass 1.0
                            :color color
                            :pose final-arrow-pose)))


(defun convert-into-poses-list (lazy-poses-list)
  "converts a given list of transforms or pose-stamped's into a list of poses.
Can be used directly on query results.
Accepts a  `lazy-poses-list' of cl-tf:transform or cl-tf:pose-stamped,
Returns: list of cl-tf:pose."
  (let* ((poses-list)
          ;; convert lazy list into normal list
         (temp-list (cut:force-ll lazy-poses-list)))
    
    ;; check what type the given list is
    (case (cl-tf::type-of (car temp-list))
      ;; convert transforms -> poses
      ('cl-tf::transform (setq poses-list
                               (mapcar (lambda (transform)
                                         (cl-tf:transform->pose transform))
                                       temp-list)))
      ;; convert poses-stamped -> poses
      ('cl-tf::pose-stamped (setq poses-list
                                  (mapcar (lambda (pose-stamped)
                                            (cl-tf:pose-stamped->pose pose-stamped))
                                          temp-list)))
      ('t (print "invalid type")))
    ;; return poses list
    poses-list))


(defun spawn-one-arrow (obj-type time)
  "spawn arrow of only one episode.
`obj-type' type of object as string.
`time' start or end of episode as string."
  ;;spawn unreal arrow. without pose modificiations.
  ;; at object location unreal (object-pose)
  (spawn-unreal-arrow (car
                       (convert-into-poses-list
                        (query-object-location-by-object-type obj-type time)))
                      "unreal-arrow-object" '(1 0 1))

  ;; at object location btr (look-pose)
  (spawn-btr-arrow (car
                    (convert-into-poses-list
                     (umap-P-uobj-through-surface-ll obj-type time)))
                   "btr-arrow-object" '(0 1 0))

  ;; at camera location unreal (original camera pose)
  (spawn-unreal-arrow (car
                       (convert-into-poses-list
                        (query-camera-location-by-object-type obj-type time)))
                      "unreal-arrow-camera" '(1 0 0))

  ;; at base location btr (unreal camera with transformations)
  ;; TODO this should also work for END
  (spawn-btr-arrow (car
                    (convert-into-poses-list
                     (umap-T-ucamera-through-surface-ll obj-type time)))
                   "btr-arrow-base" '(1 0 0)))

(defvar *prefix-counter* 0)

(defun arrow-prefix ()
  "creates a prefix for an arrow name so that they remain unique"
  (let ((name (format nil "~a" (incf *prefix-counter* 1))))
    name))


(defun spawn-multiple-arrows (obj-type time)
  "spawn as many arrows as there are poses.
`obj-type' type of object as string.
`time' start or end of episode as string."
  ;; get all the lists
  (let ((unreal-object-poses (convert-into-poses-list
                              (query-object-location-by-object-type obj-type time)))
        
        (btr-object-poses (convert-into-poses-list
                           (umap-P-uobj-through-surface-ll obj-type time)))
        
        (unreal-camera-poses (convert-into-poses-list
                              (query-camera-location-by-object-type obj-type time)))

        (btr-base-poses (convert-into-poses-list
                         (umap-T-ucamera-through-surface-ll obj-type time))))

  
  ;;spawn unreal arrow. without pose modificiations.
  ;; at object location unreal (object-pose)
    (loop for pose in unreal-object-poses
          do (spawn-unreal-arrow pose
                                 (format nil "ur-arrow-object-~a" (arrow-prefix))
                                 '(0.5 0 0.5)))
     
    ;; at object location btr (look-pose)
    (loop for pose in btr-object-poses
          do (spawn-btr-arrow pose
                              (format nil "btr-arrow-object-~a" (arrow-prefix))
                              '(0.5 0 0.5)))

    ;; at camera location unreal (original camera pose)
    (loop for pose in unreal-camera-poses
          do (spawn-unreal-arrow pose
                                 (format nil "ur-arrow-camera-~a" (arrow-prefix))
                                 '(0.7 0 0)))
    
    ;; at base location btr (unreal camera with transformations)
    ;; TODO this should also work for END
    (loop for pose in btr-base-poses
          do (spawn-btr-arrow pose
                              (format nil "btr-arrow-base-~a" (arrow-prefix))
                              '(0.7 0 0)))
    
    (loop for pose in btr-base-poses
          do (spawn-btr-arrow (cl-tf:pose-stamped->pose
                               (map-T-camera->map-P-base
                                (cl-tf:pose->transform pose)))
                              (format nil "btr-robot-base-~a" (arrow-prefix))
                              '(0 0.7 0)))
    ))



(defun spawn-arrows-to-test-objects (type)
  (let* ((poses-list (umap-P-uobj-through-surface-from-list-ll type "Start"))
        (obj-pose (convert-into-poses-list
                   (list (slot-value (car poses-list) 'obj-pose))))
        (base-pose (convert-into-poses-list
                    (list (slot-value (car poses-list) 'base-pose)))))
    
    (spawn-btr-arrow (car obj-pose) "btr-arrow-object-pose" '(1 0 0))
    (spawn-btr-arrow (car base-pose) "btr-arrow-base-pose" '(0 1 0))))

(defun spawn-arrows-from-list (list)
  ;; eg for ?visibility and such. list of poses
  (dolist (pose list)
    (spawn-btr-arrow pose (arrow-prefix) '(0 0 1))))

(defun spawn-visibility-arrows (pose-stamped-list &optional desig-color)
  "spawns arrows according to the ?visibility or ?reachability lists.
`pose-stamped-list' is the given ?visibility or ?reachability list.
`desig-color' can be set to 'vis or 'reach depending on which list it is.
'vis arrows = dark green
'reach arrows = dark blue"
  (let ((poses-list (convert-into-poses-list (car pose-stamped-list)))
        (color (cond ((eq desig-color 'vis)
                      '(1.0 0.6 0.0))
                     ((eq desig-color 'reach)
                      '(0 0 0.5))
                     ((eq desig-color 'del)
                      '(0 0.5 0.7)))))
    (dolist (pose poses-list)
      (spawn-btr-arrow pose (arrow-prefix) color))))

(defun reset-temp-lists ()
  (setq ?visibility '())
  (setq ?reachability '())
  (setq ?heuristics '())
  (setq ?deliver-reachability '()))

(defun spawn-everything ()
  (btr-utils:kill-all-objects)
  (spawn-multiple-arrows "CupEcoOrange" "Start")
  (sleep 1)
  (spawn-visibility-arrows ?visibility 'vis)
  (setq *arrow-z-offset* 3.0)
  (spawn-visibility-arrows ?deliver-reachability 'del)
  (spawn-visibility-arrows ?reachability 'reach)
  (setq *arrow-z-offset* 0.2)
  
  )
