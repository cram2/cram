;;;
;;; Copyright (c) 2019, Jonas Dech <jdech[at]uni-bremen.de
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

(in-package :cram-pr2-shopping-demo)



(defun spawn-shelf ()
  (let ((shelve-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param "shelf_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (assert (btr:object ?world :urdf :kitchen ((0 0 0) (0 0 0 1))
                                            :urdf ,shelve-urdf))))))
(defun spawn-basket ()
  (let ((basket-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param "basket_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (assert (btr:object ?world :urdf :basket ((0 0 0) (0 0 0 1))
                                            :urdf ,basket-urdf))))))


(defun spawn-kitchen ()
  (let ((kitchen-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param "kitchen_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (assert (btr:object ?world :urdf :kitchen ((0 0 0) (0 0 0 1))
                                            :urdf ,kitchen-urdf))))))

(defun spawn-robot ()
  (setf cram-robot-interfaces:*robot-urdf*
        (cl-urdf:parse-urdf
         (roslisp:get-param "robot_description")))
  (prolog:prolog
   `(and (btr:bullet-world ?world)
         (cram-robot-interfaces:robot ?robot)
         (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1))
                             :urdf ,cram-robot-interfaces:*robot-urdf*))
         (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))


(defun spawn-and-place-objects ()
  (spawn-objects)
  (place-objects)
  (btr:simulate btr:*current-bullet-world* 2))

(defun spawn-objects ()
  ;; Dove
  (prolog:prolog '(and (btr:bullet-world ?world)
                     (assert (btr:object ?world :mesh :dove ((-1 -1.06 0.7) (0 0 0 1))
                              :mass 0.2 :color (1 0 0) :mesh :dove))))

  ;; Heitmann
  (prolog:prolog '(and (btr:bullet-world ?world)
                     (assert (btr:object ?world :mesh :heitmann ((-1.3 -1.06 1) (0 0 0 1))
                              :mass 0.2 :color (1 0 0) :mesh :heitmann))))

  ;; Denkmit
  (prolog:prolog '(and (btr:bullet-world ?world)
                     (assert (btr:object ?world :mesh :denkmit ((-2 -1.1 1.3) (0 0 0 1))
                                         :mass 0.2 :color (1 0 0) :mesh :denkmit)))))

(defun place-objects ()
  (btr-utils:move-object :denkmit '((-1 -1.06 0.7) (0 0 0 1)))
  (btr-utils:move-object :heitmann '((-1.3 -1.04 1) (0 0 0 1)))
  (btr-utils:move-object :dove '((-2 -1.04 1.3) (0 0 0 1)))
  (btr:simulate btr:*current-bullet-world* 10))

(defun place-test ()
  (btr-utils:move-object :denkmit '((-1 -1.03 0.7) (0 0 0 1)))
  (btr-utils:move-object :denkmit2 '((-1.5 -1.02 1) (0 0 0 1)))
  (btr-utils:move-object :mug '((-2 -1.05 1) (0 0 0 1))))


(defun replace-denkmit ()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
     :arm :right
     :object-name :denkmit))
  (btr-utils:move-object :denkmit '((-2 -1.1 1.3) (0 0 0 1))))

(defun replace-dove ()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
     :arm :left
     :object-name :dove))
  (btr-utils:move-object :dove '((-1 -1.1 0.7) (0 0 0 1))))

(defun replace-heitmann ()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
     :arm :left
     :object-name :heitmann))
  (btr-utils:move-object :heitmann '((-1.3 -1.1 1) (0 0 0 1))))

(defun init ()
  ;; (roslisp:start-ros-node "shopping_demo")

  (cram-bullet-reasoning-belief-state::ros-time-init)
  (cram-location-costmap::location-costmap-vis-init)
  (cram-tf::init-tf)

  (prolog:prolog '(and
                   (btr:bullet-world ?world)
                   (btr:debug-window ?world)))

  (prolog:prolog '(and
                   (btr:bullet-world ?world)
                   (assert (btr:object ?world :static-plane :floor ((0 0 0) (0 0 0 1))
                                                            :normal (0 0 1) :constant 0))))
  (btr:add-objects-to-mesh-list "cram_pr2_shopping_demo"))

;; (defun make-behind-cost-function (ref-x ref-y)
;;            "`ref-x' and `ref-y' are the coordinates of the reference point according to which the behind-relation is resolved."
;;            (let* ((supp-tf (cl-transforms:make-transform
;;                                (cl-transforms:make-3d-vector ref-x ref-y 0)
;;                                (cl-transforms:make-identity-rotation)))
;;                   (world->supp-tf (cl-transforms:transform-inv supp-tf)))
;;              (lambda (x y)
;;                (let* ((point (cl-transforms:transform-point world->supp-tf
;;                                                             (cl-transforms:make-3d-vector x y 0)))
;;                       (vector-length (cl-transforms:dot-product point point)))
;;                  (if (and (> (cl-transforms:y point) 0.0d0)
;;                           (> (abs (/ (cl-transforms:x point) vector-length)) 0))
;;                      1
;;                      0.0d0)))))

;; (defmethod location-costmap:costmap-generator-name->score ((name (eql 'behind-cost-function))) 10)

;; (def-fact-group tutorial-rules (location-costmap:desig-costmap)
;;   (<- (location-costmap:desig-costmap ?designator ?costmap)
;;     (location-costmap:costmap ?costmap)
;;     ;;(desig:desig-prop ?designator (:behind ?pose))
;;     (location-costmap:costmap-add-function
;;      behind-cost-function
;;      (make-behind-cost-function -1.5 -1)
;;      ?costmap)))


(roslisp-utilities:register-ros-init-function init)
(roslisp-utilities:register-ros-init-function spawn-robot)
(roslisp-utilities:register-ros-init-function spawn-shelf)
(roslisp-utilities:register-ros-init-function spawn-and-place-objects)

(def-fact-group costmap-metadata (costmap:costmap-size
                                    costmap:costmap-origin
                                    costmap:costmap-resolution
                                    costmap:orientation-samples
                                    costmap:orientation-sample-step)
  (<- (location-costmap:costmap-size 12 12))
  (<- (location-costmap:costmap-origin -6 -6))
  (<- (location-costmap:costmap-resolution 0.04))
  (<- (location-costmap:orientation-samples 2))
  (<- (location-costmap:orientation-sample-step 0.1)))

