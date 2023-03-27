;;;
;;; Copyright (c) 2020, Amar Fayaz <amar@uni-bremen.de>
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

(in-package :fd-plans-tests)

(defparameter *error-counter-look-up* '())

(defmethod cpl:fail :before (&rest args)
  (when (and (not (null args)) (typep (first args) 'symbol))
    (add-error-count-for-error (first args))))

(defun init-test-env ()
  (setf btr-belief:*spawn-debug-window* nil)
  (coe:clear-belief)
  (setf cram-tf:*tf-default-timeout* 2.0)
  (setf prolog:*break-on-lisp-errors* t)
  (setf proj-reasoning::*projection-reasoning-enabled* nil))

(roslisp-utilities:register-ros-init-function init-test-env)

(defun init-projection ()
  (unless (eq (roslisp:node-status) :RUNNING)
    (roslisp-utilities:startup-ros))
  (btr:clear-costmap-vis-object)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr-utils:kill-all-objects)
  (setf (btr:pose (btr:get-robot-object)) (cl-transforms:make-identity-pose))
  (reset-error-counter))

(defun make-pose-stamped (pose-list)
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (apply #'cl-transforms:make-3d-vector (first pose-list))
   (apply #'cl-transforms:make-quaternion (second pose-list))))

(defun make-pose (pose-list)
  (cl-transforms:make-pose
   (apply #'cl-transforms:make-3d-vector (first pose-list))
   (apply #'cl-transforms:make-quaternion (second pose-list))))

(defun spawn-object (pose object-type)
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:spawn-object
   (intern (format nil "~a-1" object-type) :keyword)
   object-type
   :pose pose
   :mass 0.0)
  (btr:simulate btr:*current-bullet-world* 100))

(defun error-type-to-keyword (error-type)
  (intern (format nil "~a" error-type) :keyword))

(defun reset-error-counter ()
  (setf *error-counter-look-up* '()))

(defun get-total-error-count ()
  (unless (null *error-counter-look-up*)
    (reduce (lambda (count1 count2)
              (+ count1 count2))
            *error-counter-look-up*
            :key #'cdr)))

(defun add-error-count-for-error (error-type)
  (let ((error-keyword (error-type-to-keyword error-type)))
    (if (null (assoc error-keyword *error-counter-look-up*))
        (setf *error-counter-look-up* (cons (cons error-keyword 0)
                                            *error-counter-look-up*)))
        (incf (cdr (assoc error-keyword *error-counter-look-up*)))))

(defun get-error-count-for-error (error-type)
  (let ((error-keyword (error-type-to-keyword error-type)))
    (if (null (assoc error-keyword *error-counter-look-up*))
        0
        (cdr (assoc error-keyword *error-counter-look-up*)))))

(defun make-restricted-area-cost-function ()
  (lambda (x y)
    (if (> x 1.2)
        0.0
        (if (and (> x 0.5) (> y -1.5) (< y 2.0))
            1.0
            (if (and (> x 0.0) (> y -1.5) (< y 1.0))
                1.0
                (if (and (> x -1.5) (> y -1.5) (< y 2.5))
                    1.0
                    (if (and (> x -4.0) (> y -1.0) (< y 1.0))
                        1.0
                        0.0)))))))

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'restricted-area))) 5)

(def-fact-group demo-costmap (location-costmap:desig-costmap)
  (<- (location-costmap:desig-costmap ?designator ?costmap)
    (or (rob-int:visibility-designator ?designator)
        (rob-int:reachability-designator ?designator))
    ;; make sure that the location is not on the robot itself
    ;; if it is, don't generate a costmap
    (once (or (and (desig:desig-prop ?designator (:object ?some-object))
                   (desig:current-designator ?some-object ?object)
                   (lisp-fun man-int:get-object-pose-in-map ?object ?to-reach-pose)
                   (lisp-pred identity ?to-reach-pose)
                   (-> (desig:desig-prop ?object (:location ?loc))
                       (not (man-int:location-always-reachable ?loc))
                       (true)))
              (and (desig:desig-prop ?designator (:location ?some-location))
                   (desig:current-designator ?some-location ?location)
                   ;; if the location is on the robot itself,
                   ;; don't use the costmap
                   (not (man-int:location-always-reachable ?location)))))
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function
     restricted-area
     (make-restricted-area-cost-function)
     ?costmap)))

;;;;;;;;;;;; Object Poses ;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *valid-location-on-island*
  (make-pose-stamped '((-0.8 0.76 0.9) (0 0 0 1))))

(defparameter *valid-location-on-sink-area-surface*
  (make-pose-stamped '((1.48 0.96 0.9) (0 0 0 1))))


(defparameter *valid-location-on-sink-area-surface-near-oven*
  (make-pose-stamped '((1.54 1.1 0.9) (0 0 0 1))))

(defparameter *valid-location-inside-fridge*
  (make-pose-stamped '((1.48 -1.16 0.55) (0 0 0 1))))

(defparameter *invalid-location-outside-map*
  (make-pose-stamped '((2.8 0.71 0.9) (0 0 0 1))))

;;;;;;;;;;;;; Robot Poses ;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *valid-robot-pose-towards-island*
  (make-pose-stamped '((-0.1 0.74 0) (0 0 1 0))))

(defparameter *valid-robot-pose-towards-island-near-wall*
  (make-pose-stamped '((-0.1 2.2 0) (0 0 1 0))))

(defparameter *valid-robot-pose-towards-sink-area-surface*
  (make-pose-stamped '((0.8 0.7 0) (0 0 0 1))))

(defparameter *invalid-robot-pose-towards-sink-area-surface*
  (make-pose-stamped '((1.0 0.7 0) (0 0 0 1))))
