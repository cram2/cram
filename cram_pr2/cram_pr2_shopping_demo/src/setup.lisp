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

(defmethod man-int:get-z-offset-for-placing-distance :heuristics 20
  ((other-object (eql :basket))
   (object (eql :denkmit))
   attachment)
  0.1)


(defun spawn-shelf ()
  (let ((shelve-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param "shelf_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (assert (btr:object ?world :urdf :kitchen ((0 0 0) (0 0 0 1))
                                            :urdf ,shelve-urdf))))))
(defun spawn-basket ()
  (btr:add-object btr:*current-bullet-world* :basket :b (cl-transforms:make-pose
                                                              (cl-transforms:make-3d-vector 0.8 0.2 0.75)
                                                              (cl-transforms:make-quaternion 0 0 0 1))
                                                     :length 0.5
                                                     :width 0.3
                                                     :height 0.18
                                                     :mass 1
                                                     :handle-height 0.09)
  (btr:attach-object (btr:get-robot-object) (btr:object btr:*current-bullet-world* :b) :link "l_wrist_roll_link"))

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


(defun spawn-objects ()
  (let ((i 1))
    ;;Spawn objects in shelf 1
  (loop for j from -1.35 to -0.7 by 0.12
        do
           (let ((denkmit (concatenate 'string "denkmit" "-" (write-to-string j) "-" (write-to-string i)))
                 (heitmamn (concatenate 'string "heitmann" "-" (write-to-string j) "-" (write-to-string i)))
                 (dove (concatenate 'string "dove" "-" (write-to-string j) "-" (write-to-string i))))
             (btr-utils:spawn-object (intern denkmit) :denkmit :pose `((,j 0.9 0.5) (0 0 1 0)))
             (btr-utils:spawn-object (intern heitmamn) :heitmann :pose `((,j 0.9 0.96) (0 0 1 0)))
             (btr-utils:spawn-object (intern dove) :dove :pose `((,j 0.9 0.73) (0 0 1 0)))))
    (incf i)
    ;; Spawn objects in shelf 2
    (loop for k from 0.6 to 1.4 by 0.12
          do
             (let ((denkmit (concatenate 'string "denkmit" "-" (write-to-string k) "-" (write-to-string i)))
                   (heitmamn (concatenate 'string "heitmann" "-" (write-to-string k) "-" (write-to-string i)))
                   (dove (concatenate 'string "dove" "-" (write-to-string k) "-" (write-to-string i))))
               ;;(btr-utils:spawn-object (intern denkmit) :denkmit :pose `((,k 0.7 0.68) (0 0 1 0)) :color '(1 0 0 1))
               
               (btr-utils:spawn-object (intern heitmamn) :heitmann :pose `((,k 0.75 1.05) (0 0 1 0)))
               (btr-utils:spawn-object (intern dove) :dove :pose `((,k 0.75 1.39) (0 0 1 0)))))
    (incf i)
    ;; Spawn objects on the table
     (loop for k from -3.7 to -3.3 by 0.12
          do
             (let ((denkmit (concatenate 'string "denkmit" "-" (write-to-string k) "-" (write-to-string i)))
                   (heitmamn (concatenate 'string "heitmann" "-" (write-to-string k) "-" (write-to-string i)))
                   (dove (concatenate 'string "dove" "-" (write-to-string k) "-" (write-to-string i))))
               (btr-utils:spawn-object (intern denkmit) :denkmit :pose `((,k 0.1 0.7) (0 0 1 0)) :color '(1 0 0 1))))))


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


(roslisp-utilities:register-ros-init-function init)
(roslisp-utilities:register-ros-init-function spawn-robot)
(roslisp-utilities:register-ros-init-function spawn-shelf)
(roslisp-utilities:register-ros-init-function spawn-objects)
(roslisp-utilities:register-ros-init-function spawn-basket)

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

