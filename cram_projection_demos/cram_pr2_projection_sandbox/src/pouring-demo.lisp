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

(in-package :proj-sand)

(defmacro with-simulated-robot (&body body)
  `(let ((results
           (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
             (cpl:top-level
               ,@body))))
     (car (cram-projection::projection-environment-result-result results))))

;; (defmacro with-real-robot (&body body)
;;   `(cram-process-modules:with-process-modules-running
;;        (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
;;                                    pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
;;      (cpl:top-level
;;        ,@body)))

(defun test-projection ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (exe:perform
       (let ((?pose (cl-tf:make-pose-stamped
                     cram-tf:*robot-base-frame* 0.0
                     (cl-transforms:make-3d-vector -0.5 0 0)
                     (cl-transforms:make-identity-rotation))))
         (desig:a motion (type going) (target (desig:a location (pose ?pose))))))
      (exe:perform
       (desig:a motion (type moving-torso) (joint-angle 0.3)))
      (exe:perform
       (desig:a motion (type opening) (gripper left)))
      (exe:perform
       (desig:a motion (type looking) (direction forward)))
      (exe:perform
       (let ((?pose (cl-tf:make-pose-stamped
                     cram-tf:*robot-base-frame* 0.0
                     (cl-transforms:make-3d-vector 0.7 0.3 0.85)
                     (cl-transforms:make-identity-rotation))))
         (desig:a motion (type moving-tcp) (left-target (desig:a location (pose ?pose)))))))))

(defun test-desigs ()
  (let ((?pose (desig:reference (desig:a location
                                         (on "CounterTop")
                                         (name "iai_kitchen_meal_table_counter_top")))))
    (desig:reference (desig:a location
                              (to see)
                              (object (desig:an object (at (desig:a location (pose ?pose)))))))))

(defun add-objects-to-mesh-list (&optional (ros-package "cram_pr2_projection_sandbox"))
  (mapcar (lambda (object-filename-and-object-extension)
            (declare (type list object-filename-and-object-extension))
            (destructuring-bind (object-filename object-extension)
                object-filename-and-object-extension
              (let ((lisp-name (roslisp-utilities:lispify-ros-underscore-name
                                object-filename :keyword)))
                (push (list lisp-name
                            (format nil "package://~a/resource/~a.~a"
                                    ros-package object-filename object-extension)
                            nil)
                      btr::*mesh-files*)
                (remove-duplicates btr::*mesh-files* :key #'car)
                lisp-name)))
          (mapcar (lambda (pathname)
                    (list (pathname-name pathname) (pathname-type pathname)))
                  (directory (physics-utils:parse-uri
                              (format nil "package://~a/resource/*.*" ros-package))))))

(defun spawn-bottle ()
  (add-objects-to-mesh-list)
  (btr-utils:kill-all-objects)
  (btr-utils:spawn-object :bottle-1 :bottle :color '(1 0.5 0))
  (btr-utils:move-object :bottle-1 (cl-transforms:make-pose
                                    (cl-transforms:make-3d-vector -2 -1.0 0.861667d0)
                                    (cl-transforms:make-identity-rotation)))
  ;; stabilize world
  (btr:simulate btr:*current-bullet-world* 100))

(defun spawn-objects ()
  (let ((object-types (add-objects-to-mesh-list)))
    ;; spawn at default location
    (let ((objects (mapcar (lambda (object-type)
                             (btr-utils:spawn-object
                              (intern (format nil "~a-1" object-type) :keyword)
                              object-type))
                           object-types)))
      ;; move on top of counter tops
      (mapcar (lambda (btr-object)
                (let* ((aabb-z (cl-transforms:z
                                (cl-bullet:bounding-box-dimensions (btr:aabb btr-object))))
                       (new-pose (cram-tf:translate-pose
                                  (desig:reference
                                   (desig:a location
                                            (on "CounterTop")
                                            (name "iai_kitchen_meal_table_counter_top")))
                                  :z-offset (/ aabb-z 2.0))))
                  (btr-utils:move-object (btr:name btr-object) new-pose)))
              objects)
      ;; bottle gets special treatment
      (btr-utils:move-object :bottle-1 (cl-transforms:make-pose
                                        (cl-transforms:make-3d-vector -2 -1.0 0.861667d0)
                                        (cl-transforms:make-identity-rotation)))))
  ;; stabilize world
  (btr:simulate btr:*current-bullet-world* 100))

(defparameter *meal-table-left-base-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.12d0 -0.42d0 0.0)
   (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))
(defparameter *meal-table-right-base-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -2.0547d0 -0.481d0 0.0d0)
   (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))
(defparameter *meal-table-left-base-look-pose*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.75d0 -0.12d0 1.11d0)
   (cl-transforms:make-identity-rotation)))
(defparameter *meal-table-right-base-look-pose*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.65335d0 0.076d0 0.758d0)
   (cl-transforms:make-identity-rotation)))
(defparameter *meal-table-left-base-look-down-pose*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.7d0 -0.12d0 0.7578d0)
   (cl-transforms:make-identity-rotation)))


(defun prepare ()
  (cpl:with-failure-handling
          ((common-fail:low-level-failure (e)
             (roslisp:ros-warn (demo step-0) "~a" e)
             (return)))

        (let ((?navigation-goal *meal-table-right-base-pose*)
              (?ptu-goal *meal-table-right-base-look-pose*))
          (cpl:par
            (pp-plans::park-arms)
            (exe:perform (desig:a motion
                                  (type going)
                                  (target (desig:a location (pose ?navigation-goal))))))
          (exe:perform (desig:a motion
                                (type looking)
                                (target (desig:a location (pose ?ptu-goal))))))))
(defun test-pr2-plans ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (prepare))))

(defun test-projection-perception ()
  (spawn-objects)
  (test-pr2-plans)
  (cpl:sleep 1)
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (exe:perform
       (let ((?object-designator
               (desig:an object (type bottle))))
         (desig:a motion
                  (type detecting)
                  (object ?object-designator)))))))

(defun test-grasp-and-place-object (&optional (?object-type :bottle) (?arm :right))
  (let ((proj-result
          (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
            (cpl:top-level
              (prepare))
            (cpl:top-level
              (let ((?bottle-desig (desig:an object (type ?object-type))))
                (flet ((step-1-inner ()
                         (let ((?perceived-bottle-desig (pp-plans::perceive ?bottle-desig)))
                           (cpl:par
                             (exe:perform (desig:an action
                                                    (type looking)
                                                    (object ?perceived-bottle-desig)))
                             (exe:perform (desig:an action
                                                    (type picking-up)
                                                    (arm ?arm)
                                                    (object ?perceived-bottle-desig)))))))
                  (cpl:with-retry-counters ((bottle-grasp-tries 2))
                    (cpl:with-failure-handling
                        ((common-fail:low-level-failure (e)
                           (roslisp:ros-warn (demo step-1) "~a" e)
                           (cpl:do-retry bottle-grasp-tries
                             (roslisp:ros-warn (demo step-1) "~a" e)
                             (prepare)
                             (cpl:retry))))

                      (step-1-inner))))
                (desig:current-desig ?bottle-desig))))))
    (cpl:sleep 1.0)
    (let ((?result-object (car (proj::projection-environment-result-result proj-result))))
      (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
        (cpl:top-level
          (exe:perform (desig:an action
                                 (type placing)
                                 (arm ?arm)
                                 (object ?result-object))))))))

(defun test-place-bottle ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (exe:perform (desig:an action
                             (type placing)
                             (arm right))))))
