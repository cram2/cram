;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defparameter *object-spawning-poses-lists*
  '((:breakfast-cereal . (((1.4 0.4 0.95) (0 0 0.4 0.6))
                          ((1.38 1.1 0.95) (0 0 0.3 0.7))
                          ((1.35 0.6 0.95) (0 0 0.2 0.8))
                          ((1.42 0.95 0.95) (0 0 0.1 0.9))))
    (:cup . (((-0.78 1.6 0.9) (0 0 1 0))
             ((-0.78 0.6 0.9) (0 0 0.5 0.5))
             ((-0.75 0.65 0.9) (0 0 0.8 0.2))
             ((-1.3 0.57 0.9) (0 0 0 1))))
    (:bowl . (((1.4 0.8 0.89) (0 0 0.3 0.7))
              ((1.38 0.6 0.89) (0 0 0 1))
              ((1.36 0.88 0.89) (0 0 0.2 0.8))
              ((1.35 0.71 0.89) (0 0 0.8 0.2))))
    (:spoon . (((1.38 1.1 0.865) (0 0 0.2 0.8))
               ((1.39 0.8 0.865) (0 0 0.1 0.9))
               ((1.40 1.05 0.865) (0 0 0.9 0.1))
               ((1.41 0.5 0.865) (0 0 0.4 0.6))))))

(defparameter *object-grasping-arms-lists*
  '((:breakfast-cereal . (:right :left :right :left))
    (:cup . (:right :left :right :left))
    (:bowl . (:left :right :left :right))
    (:spoon . (:left :right :left :right))))

(defparameter *object-placing-poses-lists*
  '((:breakfast-cereal . (((-0.78 0.7 0.94) (0.062 0.7068 -0.0916 0.698))
                          ((-0.8 0.8 0.94) (0.24294 0.667 -0.269 0.6508))
                          ((-0.78 0.7 0.94) (0 0 0.5 0.5))
                          ((-1.25 0.6 0.94) (0.3279 0.629585 -0.3519 0.6101))))
    (:cup . (((1.3 0.8 0.9) (0 0 0 1))
             ((1.35 0.6 0.9) (0 0 0.2 0.8))
             ((1.32 0.98 0.9) (0 0 0.3 0.7))
             ((1.32 0.5 0.9) (0 0 0.8 0.2))))
    (:bowl . (((-0.76 1.0 0.89) (0 0 0.7071 0.7071))
              ((-0.76 1.3 0.89) (0 0 0.3 0.7))
              ((-1.19 0.65 0.89) (0 0 0.1 0.9))
              ((-0.99 0.6 0.89) (0 0 0.2 0.8))))
    (:spoon . (((-0.79 1.19 0.87) (0 0 1 0))
               ((-0.79 1.1 0.87) (0 0 1 0))
               ((-1.0 0.65 0.87) (0 0 0.5 0.5))
               ((-0.79 0.6 0.87) (0 0 0.5 0.5))))))


(defun pose-list->desig (pose-list)
  (let ((?pose (cl-transforms-stamped:pose->pose-stamped
                "map" 0.0
                (btr:ensure-pose pose-list))))
    (desig:a location (pose ?pose))))

(defun spawn-evaluation-objects (&optional (run 0))
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (let ((object-types '(:cup :bowl :spoon :breakfast-cereal)))
    ;; spawn at default location
    (let ((objects
            (mapcar (lambda (object-type)
                      (btr-utils:spawn-object
                       (intern (format nil "~a-1" object-type) :keyword)
                       object-type
                       :pose (nth run (cdr (assoc object-type *object-spawning-poses-lists*)))))
                    object-types)))
      ;; stabilize world
      (btr:simulate btr:*current-bullet-world* 100)
      ;; respawn if something fell on the floor
      (let (respawn)
        (dolist (object objects)
          (when (< (cl-transforms:z (cl-transforms:origin (btr:pose object))) 0.5)
            (setf respawn t)))
        (when respawn
          (spawn-random-there-and-back-again-objects))))))

(defun evaluation-there-and-back-again (&optional (run 0))
  (initialize)
  (when proj:*projection-environment*
    (spawn-evaluation-objects run))
  (park-robot)

  (dolist (?object-type '(:bowl :spoon :cup :breakfast-cereal))
    (let* ((?arm-to-use (nth run (cdr (assoc ?object-type *object-grasping-arms-lists*))))
           (?color (cdr (assoc ?object-type *object-colors*))))
      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (roslisp:ros-warn (pr2-demo evaluation-taba) "Failure happened: ~a~%Skipping..." e)
             (return)))
        (let* ((?object
                 (desig:an object
                           (type ?object-type)
                           (desig:when ?color
                             (color ?color))))
               (?first-location
                 (pose-list->desig
                  (nth run (cdr (assoc ?object-type *object-spawning-poses-lists*)))))
               (?second-location
                 (pose-list->desig
                  (nth run (cdr (assoc ?object-type *object-placing-poses-lists*))))))
          (exe:perform
           (desig:an action
                     (type transporting)
                     (object ?object)
                     (location ?first-location)
                     (target ?second-location)
                     (arm ?arm-to-use)))))))

  (park-robot)
  (finalize)
  cpl:*current-path*)

