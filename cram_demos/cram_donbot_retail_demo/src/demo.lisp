;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-donbot-retail-demo)

(defun spawn-objects-on-small-shelf (&optional (spawn? t))
  (sb-ext:gc :full t)
  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))
  (btr:clear-costmap-vis-object)
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  #+only-for-the-real-robot-so-commenting-out-for-now
  (unless cram-projection:*projection-environment*
    (giskard::call-giskard-environment-service
     :remove-all)
    (when (btr:get-environment-object)
      (giskard::call-giskard-environment-service
       :add-environment
       :name (roslisp-utilities:rosify-underscores-lisp-name
              (rob-int:get-environment-name))
       :pose (cl-transforms-stamped:pose->pose-stamped
              cram-tf:*fixed-frame* 0.0 (btr:pose (btr:get-environment-object)))
       :joint-state-topic "kitchen/joint_states")))

  (when (and spawn? cram-projection:*projection-environment*)
    (btr-utils:spawn-object :balea-bottle-1 :balea-bottle :pose
                            '((1.9 -1.42 1.05) (0 0 0.7 0.7))
                            :color '(1 1 1))
    (btr:add-object btr:*current-bullet-world* :box-item
                    :denkmitgeschirrreinigernature-1
                    '((1.75 -1.45 1.06) (0 0 0.7 0.7))
                    :mass 0.2
                    :color '(0 1 0 1.0)
                    :size '(0.057 0.018 0.074)
                    :item-type :dish-washer-tabs)
    ;; (btr:simulate btr:*current-bullet-world* 50)
    (btr-utils:move-robot '((1.0 0 0) (0 0 0 1)))))

(defun demo ()
  (spawn-objects-on-small-shelf)

  (let* ((?environment-name
           (rob-int:get-environment-name))
         (?search-location
           (desig:a location
                    (on (desig:an object
                                  (type shelf)
                                  (urdf-name shelf-2-base)
                                  (owl-name "shelf_system_verhuetung")
                                  (part-of ?environment-name)
                                  (level 4)))
                    (side left)
                    (range 0.2)))
         (?object
           (an object
               (type dish-washer-tabs)
               (location ?search-location)))
         (?target-location-shelf
           (desig:a location
                    (on (desig:an object
                                  (type environment)
                                  (name ?environment-name)
                                  (part-of ?environment-name)
                                  (urdf-name shelf-1-level-2-link)))
                    (for ?object)
                    (attachments (dish-washer-tabs-shelf-1-front
                                  dish-washer-tabs-shelf-1-back))))
         (?robot-name (rob-int:get-robot-name))
         (?intermediate-locaiton-robot
           (desig:a location
                    (on (desig:an object
                                  (type robot)
                                  (name ?robot-name)
                                  (part-of ?environment-name)
                                  ;; (owl-name "donbot_tray")
                                  (urdf-name plate)))
                    (for ?object)
                    (attachments (donbot-tray-front donbot-tray-back)))))

    (exe:perform
     (desig:an action
               (type transporting)
               (object ?object)
               (target ?intermediate-locaiton-robot)
               ;; (target ?target-location-shelf)
               ))

    (exe:perform
     (desig:an action
               (type transporting)
               (object ?object)
               (target ?target-location-shelf)))

    ;; (setf ?object
    ;;       (desig:copy-designator
    ;;        (perform (a motion
    ;;                    (type :world-state-detecting)
    ;;                    (object (an object
    ;;                                (name denkmitgeschirrreinigernature-1)))))
    ;;        :new-description
    ;;        `((:location ,(a location
    ;;                         (on (an object
    ;;                                 (type robot)
    ;;                                 (name ?robot-name)
    ;;                                 (urdf-name plate)
    ;;                                 (owl-name "donbot_tray"))))))))

    ;; look at separators
    ;; (exe:perform
    ;;  (desig:an action
    ;;            (type looking)
    ;;            (direction right-separators)))
    ;; (cpl:sleep 5.0)
))



















(defun demo-hardcoded (&optional (?item-type :dish-washer-tabs)
                         (park-drive-look? t))

  (spawn-objects-on-small-shelf)

  (let ((?object
          (an object
              (type ?item-type)
              (location (a location
                           (on (an object
                                   (type shelf)
                                   (urdf-name shelf-2-level-3-link)
                                   (owl-name "shelf_system_verhuetung")))))))

        (?target-pose-front
          (cl-transforms-stamped:make-pose-stamped
           cram-tf:*fixed-frame*
           0.0
           (cl-transforms:make-3d-vector 4.06 -0.64 1.05)
           (cl-transforms:make-quaternion 0 0 1 0)))

        (?target-pose-back
          (cl-transforms-stamped:make-pose-stamped
           cram-tf:*fixed-frame*
           0.0
           (cl-transforms:make-3d-vector 4.06 -0.64 1.05)
           (cl-transforms:make-quaternion 0 0 0 1))))

    (when park-drive-look?
      ;; park arm
      (exe:perform
       (desig:an action
                 (type parking-arms)
                 (arms (left))))

      ;; drive to pick up
      (let ((?pose (cl-transforms-stamped:make-pose-stamped
                    "map" 0.0
                    (cl-transforms-stamped:make-3d-vector
                     2.2660367329915365d0
                     ;; -0.16621163686116536d0
                     -0.26
                     0.0)
                    (cl-transforms:make-quaternion
                     0.0d0 0.0d0 -0.020739689469337463d0 0.9997849464416504d0))))
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location (pose ?pose))))))

      ;; look at the shelf
      (exe:perform
       (desig:an action
                 (type looking)
                 (direction right))))

    ;; perceive
    ;; TODO: make this using WITH-FAILURE-HANDLING and write about
    ;; that in deliverable
    (let (percept-believable
          (?grasp :back))
      (loop until percept-believable
            do (let* ((perceived-object
                        (exe:perform
                         (desig:an action
                                   (type detecting)
                                   (object ?object))))
                      (perceived-object-pose
                        (btr:object-pose
                         (desig:desig-prop-value perceived-object :name)))
                      ;; (perceived-object-pose
                      ;;   (man-int:get-object-pose-in-map perceived-object))
                      (perceived-object-pose-z
                        (cl-transforms:z
                         (cl-transforms:origin perceived-object-pose)))
                      (perceived-object-orientation
                        (cl-transforms:orientation perceived-object-pose))
                      (perceived-object-orientation-axis
                        (cl-transforms:quaternion->axis-angle
                         perceived-object-orientation))
                      (perceived-object-orientation-angle
                        (* (nth-value
                            1
                            (cl-transforms:quaternion->axis-angle
                             perceived-object-orientation))
                           (if (> (cl-transforms:z perceived-object-orientation-axis)
                                  0)
                               1
                               -1))))
                 (if (> (cl-transforms:normalize-angle
                         perceived-object-orientation-angle)
                        0)
                     (setf ?grasp :front)
                     (setf ?grasp :back))
                 (setf percept-believable
                       (and (> perceived-object-pose-z 1.02)
                            (< perceived-object-pose-z 1.2)
                            (> (abs (cl-transforms:z
                                     perceived-object-orientation-axis))
                               (abs (cl-transforms:x
                                     perceived-object-orientation-axis)))
                            (> (abs (cl-transforms:z
                                     perceived-object-orientation-axis))
                               (abs (cl-transforms:y
                                     perceived-object-orientation-axis)))
                            (> (abs perceived-object-orientation-angle) 1.0)
                            (< (abs perceived-object-orientation-angle) 2.0)))))

      (let ((picking-up-action
              (desig:an action
                        (type picking-up)
                        (grasp ?grasp)
                        (object ?object))))
        ;; (proj-reasoning:check-picking-up-collisions picking-up-action)
        ;; picking up
        (exe:perform picking-up-action))

      ;; placing on the back
      (let* ((?robot-name (rob-int:get-robot-name))
             (?robot-link-name "plate")
             (?pose-in-map (cram-tf:frame-to-pose-in-fixed-frame ?robot-link-name))
             (?transform-in-map (cram-tf:pose-stamped->transform-stamped
                                 ?pose-in-map ?robot-link-name))
             (?pose-in-base (cram-tf:ensure-pose-in-frame
                             ?pose-in-map cram-tf:*robot-base-frame*
                             :use-zero-time t))
             (?transform-in-base (cram-tf:pose-stamped->transform-stamped
                                  ?pose-in-base ?robot-link-name))
             (?attachment (ecase ?grasp
                            (:front :donbot-tray-front)
                            (:back :donbot-tray-back))))
        (exe:perform
         (an action
             (type placing)
             (object ?object)
             (target (a location
                        (on (an object
                                (type robot)
                                (name ?robot-name)
                                (urdf-name plate)
                                (owl-name "donbot_tray")
                                (pose ((pose ?pose-in-base)
                                       (transform ?transform-in-base)
                                       (pose-in-map ?pose-in-map)
                                       (transform-in-map ?transform-in-map)))))
                        (for ?object)
                        (attachment ?attachment)))))

        (setf ?object
              (desig:copy-designator
               (perform (a motion
                           (type :world-state-detecting)
                           (object (an object
                                       (name denkmitgeschirrreinigernature-1)))))
               :new-description
               `((:location ,(a location
                                (on (an object
                                        (type robot)
                                        (name ?robot-name)
                                        (urdf-name plate)
                                        (owl-name "donbot_tray")))))))))

      ;; park arm
      (exe:perform
       (desig:an action
                 (type parking-arms)
                 (arms (left))))

      ;; drive to place
      (let ((?pose (cl-transforms-stamped:make-pose-stamped
                    "map" 0.0
                    (cl-transforms-stamped:make-3d-vector
                     ;; 2.7235169607604437d0
                     3.4
                     -0.13619131858235267d0
                     0.0d0)
                    (cl-transforms:make-quaternion
                     0.0d0
                     0.0d0
                     0.6886594891548157d0
                     0.7250849008560181d0))))
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location (pose ?pose))))))


      ;; look at separators
      ;; (exe:perform
      ;;  (desig:an action
      ;;            (type looking)
      ;;            (direction right-separators)))
      ;; (cpl:sleep 5.0)

      ;; look at tray
      ;; (exe:perform
      ;;  (desig:an action
      ;;            (type looking)
      ;;            (direction down)))

      ;; ;; reperceive object
      ;; (let* ((?robot-name
      ;;          (rob-int:get-robot-name)))
      ;;   (setf ?object
      ;;         (perform
      ;;          (an action
      ;;              (type detecting)
      ;;              (object (an object
      ;;                          (type ?item-type)
      ;;                          (location (a location
      ;;                                       (on (an object
      ;;                                               (type robot)
      ;;                                               (name ?robot-name)
      ;;                                               (urdf-name plate)
      ;;                                               (owl-name "donbot_tray")))))))))))


      (exe:perform
       (an action
           (type picking-up)
           (grasp ?grasp)
           (object ?object)))

      ;; place on the big shelf
      (let ((?target-pose
              (ecase ?grasp
                (:front ?target-pose-front)
                (:back ?target-pose-back))))
        (exe:perform
         (an action
             (type placing)
             (object ?object)
             (target (a location
                        (pose ?target-pose)))))))))




(defun pick-object-from-small-shelf (&optional (?item-type :dish-washer-tabs)
                                       (park-drive-look? t))
  (spawn-objects-on-small-shelf)

  (let ((?object
          (an object
              (type ?item-type)
              (location (a location
                           (on (an object
                                   (type shelf)
                                   (urdf-name shelf-2-level-3-link)
                                   (owl-name "shelf_system_verhuetung"))))))))

    (when park-drive-look?
      ;; park arm
      (exe:perform
       (desig:an action
                 (type parking-arms)
                 (arms (left))))

      ;; drive to pick up
      (let ((?pose (cl-transforms-stamped:make-pose-stamped
                    "map" 0.0
                    (cl-transforms-stamped:make-3d-vector
                     2.2660367329915365d0 -0.16621163686116536d0 0.0)
                    (cl-transforms:make-quaternion
                     0.0d0 0.0d0 -0.020739689469337463d0 0.9997849464416504d0))))
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location (pose ?pose))))))

      ;; look at the shelf
      (exe:perform
       (desig:an action
                 (type looking)
                 (direction right))))

    ;; perceive
    ;; TODO: make this using WITH-FAILURE-HANDLING and write about
    ;; that in deliverable
    (let (percept-believable
          (?grasp :back))
      (loop until percept-believable
            do (let* ((perceived-object
                        (exe:perform
                         (an action
                             (type detecting)
                             (object ?object))))
                      (perceived-object-pose
                        (btr:object-pose
                         (desig:desig-prop-value perceived-object :name)))
                      ;; (perceived-object-pose
                      ;;   (man-int:get-object-pose-in-map perceived-object))
                      (perceived-object-pose-z
                        (cl-transforms:z
                         (cl-transforms:origin perceived-object-pose)))
                      (perceived-object-orientation
                        (cl-transforms:orientation perceived-object-pose))
                      (perceived-object-orientation-axis
                        (cl-transforms:quaternion->axis-angle
                         perceived-object-orientation))
                      (perceived-object-orientation-angle
                        (* (nth-value
                            1
                            (cl-transforms:quaternion->axis-angle
                             perceived-object-orientation))
                           (if (> (cl-transforms:z perceived-object-orientation-axis)
                                  0)
                               (prog1
                                   1
                                 (setf ?grasp :front))
                               (prog1
                                   -1
                                 (setf ?grasp :back))))))
                 (setf percept-believable
                       (and (> perceived-object-pose-z 1.02)
                            (< perceived-object-pose-z 1.2)
                            (> (abs (cl-transforms:z
                                     perceived-object-orientation-axis))
                               (abs (cl-transforms:x
                                     perceived-object-orientation-axis)))
                            (> (abs (cl-transforms:z
                                     perceived-object-orientation-axis))
                               (abs (cl-transforms:y
                                     perceived-object-orientation-axis)))
                            (> (abs perceived-object-orientation-angle) 1.0)
                            (< (abs perceived-object-orientation-angle) 2.0)))))

      (let ((picking-up-action
              (an action
                  (type picking-up)
                  (grasp ?grasp)
                  (object ?object))))
        ;; (proj-reasoning:check-picking-up-collisions picking-up-action)
        ;; picking up
        (exe:perform picking-up-action)))

    ;; placing on the back
    (let* ((?robot-name (rob-int:get-robot-name))
           (?robot-link-name "plate")
           (?pose-in-map (cram-tf:frame-to-pose-in-fixed-frame ?robot-link-name))
           (?transform-in-map (cram-tf:pose-stamped->transform-stamped
                               ?pose-in-map ?robot-link-name))
           (?pose-in-base (cram-tf:ensure-pose-in-frame
                           ?pose-in-map cram-tf:*robot-base-frame*
                           :use-zero-time t))
           (?transform-in-base (cram-tf:pose-stamped->transform-stamped
                                ?pose-in-base ?robot-link-name)))
      (exe:perform
       (an action
           (type placing)
           (object ?object)
           (target (a location
                      (on (an object
                              (type robot)
                              (name ?robot-name)
                              (urdf-name plate)
                              (owl-name "donbot_tray")
                              (pose ((pose ?pose-in-base)
                                     (transform ?transform-in-base)
                                     (pose-in-map ?pose-in-map)
                                     (transform-in-map ?transform-in-map)))))
                      (for ?object)
                      (attachment donbot-tray-left))))))))


(defun place-object-at-big-shelf (&optional (?item-type :dish-washer-tabs)
                                    (park-drive-look? t)
                                    (?target-pose
                                     (cl-transforms-stamped:pose->pose-stamped
                                      cram-tf:*fixed-frame* 0.0
                                      (cram-tf:list->pose
                                       '((4.048458 -0.64891 1.07)
                                         (-0.7071067811865475d0
                                          0.0d0
                                          0.7071067811865475d0
                                          0.0d0))))))
  ;; (spawn-objects-on-small-shelf nil)
  (when park-drive-look?
    ;; park arm
    (exe:perform
     (desig:an action
               (type parking-arms)
               (arms (left))))

    ;; drive to place
    (let ((?pose (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms-stamped:make-3d-vector
                   2.7235169607604437d0
                   -0.13619131858235267d0
                   0.0d0)
                  (cl-transforms:make-quaternion
                   0.0d0
                   0.0d0
                   0.6886594891548157d0
                   0.7250849008560181d0))))
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?pose))))))

    ;; look at separators
    (exe:perform
     (desig:an action
               (type looking)
               (direction right-separators)))
    (cpl:sleep 5.0)

    ;; look at tray
    (exe:perform
     (desig:an action
               (type looking)
               (direction down))))

  ;; pick up from the tray
  (let* ((?robot-name
           (rob-int:get-robot-name))
         (?obj
           (perform
            (an action
                (type detecting)
                (object (an object
                            (type ?item-type)
                            (location (a location
                                         (on (an object
                                                 (type robot)
                                                 (name ?robot-name)
                                                 (urdf-name plate)
                                                 (owl-name "donbot_tray")))))))))))

    (exe:perform
     (an action
         (type picking-up)
         (grasp top)
         (object ?obj)))

    ;; place on the big shelf
    (exe:perform
     (an action
         (type placing)
         (object ?obj)
         (target (a location
                    (pose ?target-pose)))))))


#+stuff-to-test-real-robot-interfaces
(defun stuff-that-works ()
  (cram-process-modules:with-process-modules-running
      (giskard:giskard-pm)
    (cpl-impl::named-top-level (:name :top-level)
      (exe:perform
       (let ((?pose (cl-transforms-stamped:make-pose-stamped
                     "base_footprint" 0.0
                     (cl-transforms:make-3d-vector
                      -0.27012088894844055d0
                      0.5643729567527771d0
                      1.25943687558174133d0)
                     (cl-transforms:make-quaternion
                      -0.4310053586959839d0
                      0.24723316729068756d0
                      0.752766489982605d0
                      0.4318017065525055d0 ))))
         (desig:a motion
                  (type moving-tcp)
                  (left-pose ?pose)
                  (collision-mode :allow-all)))))))
