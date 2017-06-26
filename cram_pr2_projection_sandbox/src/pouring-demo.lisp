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

(defun test-projection ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (exe:perform
       (let ((?pose (cl-tf:make-pose-stamped
                     cram-tf:*robot-base-frame* 0.0
                     (cl-transforms:make-3d-vector 0 0 0)
                     (cl-transforms:make-identity-rotation))))
         (desig:a motion (type going) (target (desig:a location (pose ?pose))))))
      (exe:perform
       (desig:a motion (type moving-torso) (joint-angle 0.3)))
      (exe:perform
       (desig:a motion (type opening) (gripper left)))
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
                              (obj (desig:an object (at (desig:a location (pose ?pose)))))))))

(defun add-objects-to-mesh-list (&optional (ros-package "cram_pr2_projection_sandbox"))
  (mapcar (lambda (object-filename-and-object-extension)
            (declare (type list object-filename-and-object-extension))
            (destructuring-bind (object-filename object-extension)
                object-filename-and-object-extension
              (let ((lisp-name (roslisp-utilities:lispify-ros-underscore-name
                                object-filename :keyword)))
                (pushnew (list lisp-name
                               (format nil "package://~a/resource/~a.~a"
                                       ros-package object-filename object-extension)
                               nil)
                         btr::*mesh-files*
                         :key #'car)
                lisp-name)))
          (mapcar (lambda (pathname)
                    (list (pathname-name pathname) (pathname-type pathname)))
                  (directory (physics-utils:parse-uri
                              (format nil "package://~a/resource/*.*" ros-package))))))

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
                                        (cl-transforms:make-3d-vector -2 -0.9 0.861667d0)
                                        (cl-transforms:make-identity-rotation)))))
  ;; stabilize world
  (btr:simulate btr:*current-bullet-world* 100))

(defun prepare ()
  (cpl:with-failure-handling
          ((common-fail:low-level-failure (e)
             (roslisp:ros-warn (demo step-0) "~a" e)
             (return)))

        (let ((?navigation-goal pr2-pp-plans::*meal-table-right-base-pose*)
              (?ptu-goal pr2-pp-plans::*meal-table-right-base-look-pose*))
          (cpl:par
            (pr2-pp-plans::move-pr2-arms-out-of-sight)
            (exe:perform (desig:a motion
                                  (type going)
                                  (target (desig:a location (pose ?navigation-goal)))))
            (exe:perform (desig:a motion
                                  (type looking)
                                  (target (desig:a location (pose ?ptu-goal)))))))))
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

(defun test-grasp-bottle ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (flet ((step-1-inner ()
               (let* ((?bottle-desig (desig:an object
                                               (type bottle)))
                      (?perceived-bottle-desig (pr2-pp-plans::perceive ?bottle-desig)))
                 (pr2-pp-plans::drive-and-pick-up-plan ?perceived-bottle-desig :?arm :right))))
        (cpl:with-retry-counters ((bottle-grasp-tries 2))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure (e)
                 (roslisp:ros-warn (demo step-1) "~a" e)
                 (if (pr2-pp-plans::get-object-in-hand :right)
                     (return)
                     (cpl:do-retry bottle-grasp-tries
                       (roslisp:ros-warn (demo step-1) "~a" e)
                       (prepare)
                       (cpl:retry)))))

            (step-1-inner)))))))

(defun test-place-bottle ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (exe:perform (desig:an action
                             (type placing)
                             (arm right))))))
