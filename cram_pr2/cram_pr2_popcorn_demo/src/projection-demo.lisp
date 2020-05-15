;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Thomas Lipps    <tlipps@uni-bremen.de>
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

(in-package :demo)

(defparameter *object-spawning-poses*
  '((:popcorn-pot . ((-0.875 -1.45 0.195) (0 0 0 1)))
    (:popcorn-pot-lid . ((-0.78 -1.53 0.54) (0 0 0 1)))
    (:ikea-bowl-ww . ((-0.95 -1.65 0.54) (0 0 0 1)))
    (:salt . ((0.5 -1.6 0.78) (0 0 0 1)))
    (:plate . ((0.3 -1.52 0.75)(0 0 0 1)))
    (:ikea-plate . ((-0.18 -0.35 0.1) (0 0 0 1))))
  "Absolute poses in map.")

(defparameter *relative-object-spawning-poses*
  '("iai_popcorn_table_surface"
    ((:POPCORN-POT . ((-0.575 0.22 -0.505)(0 0 0 1)))
     (:POPCORN-POT-LID . ((-0.48 0.14 -0.16)(0 0 0 1)))
     (:IKEA-BOWL-WW . ((-0.65 0.020 -0.16)(0 0 0 1)))
     (:SALT . ((0.8035 0.07637 0.0738)(0 0 0 1)))
     (:PLATE . ((0.6 0.15 0.05)(0 0 0 1)))
     (:IKEA-PLATE . ((0.6 0.15 -0.16)(0 0 0 1))))))

(defparameter *look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))

(defun ensure-pose-stamped (pose)
  (when pose
    (case (type-of pose)
      ('cl-transforms-stamped:pose-stamped pose)
      ('cons (make-pose-absolute pose))
      (T (error "cannot translate pose of type ~a in cl-transforms-stamped" (type-of pose))))))

(defun get-item (?object-type)
  (find ?object-type
        (remove-if-not
         (lambda (x) 
           (equalp
            'cram-bullet-reasoning::item x))
         (btr:objects
          btr:*current-bullet-world*)
         :key #'type-of)
        :key (alexandria:compose #'car #'btr:item-types)))

(defun get-object-designator (?object-type &optional ?pose-in-map)
  (let* ((?object-name (btr:name (get-item ?object-type)))
         (?pose (if ?pose-in-map
                    (ensure-pose-stamped ?pose-in-map)
                    (cl-tf:pose->pose-stamped
                     cram-tf:*fixed-frame*
                     0.0
                     (btr:pose (get-item ?object-type)))))
         (?transform-in-base (pp-plans::pose->transform-stamped-in-base ?pose ?object-name))
         (?pose-in-base (cl-tf:ensure-pose-stamped ?transform-in-base))
         (?transform (cram-tf:pose-stamped->transform-stamped ?pose ?object-name)))
    (desig:an object
              (type ?object-type)
              (name ?object-name)
              (pose ((pose ?pose-in-base)
                     (transform ?transform-in-base)
                     (transform-in-map ?transform)
                     (pose-in-map ?pose))))))
  

(cpl:def-cram-function park-robot ()
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (exe:perform
       (desig:an action
                 (type positioning-arm)
                 (left-configuration park)
                 (right-configuration park)))
      (let ((?pose (cl-transforms-stamped:make-pose-stamped
                    cram-tf:*fixed-frame*
                    0.0
                    (cl-transforms:make-identity-vector)
                    (cl-transforms:make-identity-rotation))))
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location
                                    (pose ?pose))))))
      (exe:perform (desig:an action (type opening-gripper) (gripper (left right))))
      (exe:perform (desig:an action (type looking) (direction forward))))))

(defun make-poses-absolute (frame-and-types-with-poses)
  (when frame-and-types-with-poses
    (let ((frame (first frame-and-types-with-poses)))
      (mapcar (lambda (type-and-pose)
                (cons (car type-and-pose)
                      (make-pose-absolute (cons frame (cdr type-and-pose)))))
              (second frame-and-types-with-poses)))))

(defun make-pose-absolute (frame-and-pose)
  (when frame-and-pose
    (let* ((btr-object (btr:object btr:*current-bullet-world* (first frame-and-pose)))
           (link-pose (btr:link-pose (btr:get-environment-object) (first frame-and-pose)))
           (map-T-surface (cl-transforms:pose->transform
                           (if btr-object
                               (btr:pose btr-object)
                               link-pose))))
        (let* ((input-pose (cdr frame-and-pose))
               (pose (case (type-of input-pose)
                       ('cl-transforms-stamped:pose-stamped input-pose)
                       ('cons (cram-tf:list->pose input-pose))
                       (T (error "Unknown pose data type: only lists ~
                                  and pose-stampeds are allow as input."))))
               (surface-T-object
                 (cl-transforms:pose->transform pose))
               (map-T-object
                 (cl-transforms:transform* map-T-surface surface-T-object)))
          (cl-tf:pose->pose-stamped
           cram-tf:*fixed-frame*
           0.0
           (cl-tf:transform->pose map-T-object))))))


(defun spawn-objects (&key
                        (object-types '(:popcorn-pot
                                        :popcorn-pot-lid
                                        :salt
                                        :ikea-bowl-ww
                                        :ikea-plate)))
  
  ;; make sure mesh paths are known, kill old objects and destroy all attachments
  (btr:add-objects-to-mesh-list "cram_pr2_popcorn_demo")
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))

  ;; spawn objects
  (let* ((spawning-poses-absolute (make-poses-absolute *relative-object-spawning-poses*))
         (objects
           (mapcar (lambda (object-type)
                     (let* (;; generate object name of form :TYPE-1
                            (object-name
                              (intern (format nil "~a-1" object-type) :keyword))
                            ;; spawn object in Bullet World at default pose
                            (object
                              (btr-utils:spawn-object object-name object-type))
                            ;; calculate new pose: either random or from input list
                            (object-pose
                              ;; take the pose from the function input list
                              (cdr (assoc object-type spawning-poses-absolute))))
                       ;; move object to calculated pose on surface
                       (btr-utils:move-object object-name object-pose)
                       ;; return object
                       object))
                   object-types)))
  
    ;; stabilize world
    (btr:simulate btr:*current-bullet-world* 100)

    ;; attach the objects to the links in the kitchen
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world* :popcorn-pot-1)
                       :link "iai_popcorn_table_right_grid")
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world* :popcorn-pot-lid-1)
                       :link "iai_popcorn_table_drawer_right_main")
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world* :ikea-bowl-ww-1)
                       :link "iai_popcorn_table_drawer_right_main")
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world* :salt-1)
                       :link "iai_popcorn_table_surface")
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world* :ikea-plate-1)
                       :link "iai_popcorn_table_drawer_left_main")

    ;; return list of BTR objects
    objects))


(defun go-to-pose (?navigation-goal-as-list &key dont-move-arms)
  (let ((?ptu-goal *look-goal*)
        (?navigation-goal (ensure-pose-stamped ?navigation-goal-as-list)))
    (unless dont-move-arms
      (exe:perform (desig:an action
                             (type positioning-arm)
                             (left-configuration park)
                             (right-configuration park))))
    (exe:perform (desig:a motion
                          (type going)
                          (pose ?navigation-goal)))
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?ptu-goal)))))


(defun pick-object (?object-type ?arm ?pose-as-list &key ?grasp)
  (let ((?object (get-object-designator ?object-type ?pose-as-list)))
    (cpl:seq
      (exe:perform (desig:an action     
                             (type looking)
                             (object ?object)))
      (exe:perform (desig:an action
                             (type picking-up)
                             (arm ?arm)
                             (desig:when ?grasp
                               (grasp ?grasp))
                             (object ?object))))))

(defun place-object (?arm ?target-pose-as-list &key ?object-placed-on ?object-to-place ?attachment)
  (let ((?target-pose (ensure-pose-stamped ?target-pose-as-list)))
    (cpl:seq
      (exe:perform (desig:a motion
                            (type looking)
                            (pose ?target-pose)))
      (exe:perform 
       (desig:an action
                 (type placing)
                 (arm ?arm)
                 (desig:when ?object-to-place
                   (object ?object-to-place))
                 (target (desig:a location
                                  (desig:when ?object-placed-on
                                    (on ?object-placed-on))
                                  (desig:when ?object-to-place
                                    (for ?object-to-place))
                                  (desig:when ?attachment
                                    (attachment ?attachment))
                                  (pose ?target-pose))))))))

(defun open-drawer (drawer)
  (manipulate-drawer drawer :opening))

(defun close-drawer (drawer)
  (manipulate-drawer drawer :closing))

(defun manipulate-drawer (?drawer ?action)
  (let* ((?urdf-name (if (eq ?drawer :right)
                         'iai-popcorn-table-drawer-right-main
                         'iai-popcorn-table-drawer-left-main))
         (?drawer-obj (desig:an object
                                (type drawer)
                                (urdf-name ?urdf-name)
                                (part-of kitchen)))
         (?distance (if (eq ?action :opening)
                        0.4
                        0.0)))
    (exe:perform
     (desig:an action 
               (type ?action)
               (arm ?drawer) ;; since for opening the left drawer the
                             ;; robot should use the left arm.
               (object ?drawer-obj)
               (distance ?distance)))))

(defun move-arms (&key ?left-arm-pose ?right-arm-pose)
  (let* ((?left-arm-pose-stamped-unknown-frame (ensure-pose-stamped ?left-arm-pose))
        (?right-arm-pose-stamped-unknown-frame (ensure-pose-stamped ?right-arm-pose))
        (?left-arm-pose-stamped (when ?left-arm-pose-stamped-unknown-frame
                                  (if (equalp
                                       cram-tf:*fixed-frame*
                                       (cl-tf:frame-id ?left-arm-pose-stamped-unknown-frame))
                                      ?left-arm-pose-stamped-unknown-frame
                                      (make-pose-absolute (cons 
                                                           (cl-tf:frame-id ?left-arm-pose-stamped-unknown-frame)
                                                           ?left-arm-pose-stamped-unknown-frame)))))
        (?right-arm-pose-stamped (when ?right-arm-pose-stamped-unknown-frame 
                                   (if (equalp
                                        cram-tf:*fixed-frame*
                                        (cl-tf:frame-id ?right-arm-pose-stamped-unknown-frame))
                                       ?right-arm-pose-stamped-unknown-frame
                                       (make-pose-absolute (cons 
                                                            (cl-tf:frame-id ?right-arm-pose-stamped-unknown-frame)
                                                            ?right-arm-pose-stamped-unknown-frame))))))
        (exe:perform
          (desig:a motion
                   (type moving-tcp)
                   (desig:when ?right-arm-pose
                     (right-pose ?right-arm-pose-stamped))
                   (desig:when ?left-arm-pose
                     (left-pose ?left-arm-pose-stamped))))))

(defun park-arms ()
  (exe:perform
   (desig:an action
             (type positioning-arm)
             (left-configuration park)
             (right-configuration park))))

;; (defun perceive-object (?object-type)
;;   (let ((?object-desig
;;           (desig:an object (type ?object-type))))
;;     (exe:perform (desig:an action
;;                            (type perceiving)
;;                            (object ?object-desig)
;;                            (counter 0)
;;                            (occluding-names T)))))

;; (defun world-state-detecting (?object-type)
;;   (let ((?object-desig
;;           (desig:an object (type ?object-type))))
;;     (exe:perform (desig:a motion
;;                           (type world-state-detecting)
;;                           (object ?object-desig)
;;                           (counter 0)
;;                           (occluding-names T)))))
