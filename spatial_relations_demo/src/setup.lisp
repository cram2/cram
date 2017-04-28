;;; Copyright (c) 2014, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :bullet-reasoning-utilities)

(def-fact-group costmap-metadata ()
  (<- (location-costmap:costmap-size 12 12))
  (<- (location-costmap:costmap-origin -6 -6))
  (<- (location-costmap:costmap-resolution 0.05))

  (<- (location-costmap:costmap-padding 0.2))
  (<- (location-costmap:costmap-manipulation-padding 0.2))
  (<- (location-costmap:costmap-in-reach-distance 0.9))
  (<- (location-costmap:costmap-reach-minimal-distance 0.1)))

(def-fact-group semantic-map-data ()
  (<- (semantic-map-object-name :kitchen)))

;; (disable-location-validation-function 'btr-desig::validate-designator-solution)
;; finding a good IK solution takes forever, so disabling this thing and letting collisions happen
(disable-location-validation-function 'btr-desig::check-ik-solution)

(setf *tf-default-timeout* 0.0)
;; (setf cram-designators::*print-location-validation-function-results* t)

(defun init (&optional (ip "192.168.100.194"))
  "Start a ROS node with `ip' as a part of master-uri"
  (let ((uri (roslisp:make-uri ip 11311)))
   (unless (and (equalp roslisp:*master-uri* uri)
                (eq roslisp::*node-status* :running))
       (roslisp-utilities:startup-ros :anonymous nil :master-uri uri))))

;; roslaunch spatial_relations_demo demo.launch
(defvar *robot-urdf-lowres* nil)
(defvar *kitchen-urdf* nil)
(defun start-ros-and-bullet (&optional (pi-rotation '(0 0 0 1)) (sem-map-xyz '(0 0 0)))
  (init "localhost")
  (setf *tf-default-timeout* 1.0)
  
  (unless *robot-urdf-lowres*
    (setf *robot-urdf-lowres*
          (cl-urdf:parse-urdf (roslisp:get-param "robot_description"))))
  (unless *kitchen-urdf*
    (setf *kitchen-urdf*
          (cl-urdf:parse-urdf (roslisp:get-param "kitchen_description")))
    (setf (slot-value
           (gethash "pancake_table_table_joint"
                    (slot-value cram-bullet-reasoning-utilities::*kitchen-urdf* 'cl-urdf:joints))
           'cl-urdf:origin)
          (cl-transforms:make-transform
           (cl-transforms:make-3d-vector -2.8 -3.55 0)
           (cl-transforms:make-identity-rotation))))
  (let ((pi-rotation '(0 0 1 0))
        (sem-map-xyz '(-3.45 -4.35 0)))
    (clear-costmap-vis-object)
    (prolog
     `(and
       (clear-bullet-world)
       (bullet-world ?w)
       (debug-window ?w)
       (robot ?robot)
       (assert (object ?w :static-plane floor ((0 0 0) (0 0 0 1))
                       :normal (0 0 1) :constant 0
                       :disable-collisions-with (?robot)))
       (semantic-map-object-name ?map-name)
       (assert (object ?w :semantic-map ?map-name (,sem-map-xyz ,pi-rotation)
                       :urdf ,*kitchen-urdf*))
       (assert (object ?w :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf
                       ,*robot-urdf-lowres*))
       (robot-arms-parking-joint-states ?robot ?joint-states)
       (robot-torso-link-joint ?robot ?_ ?joint)
       (assert (joint-state ?w ?robot ?joint-states))
       (assert (joint-state ?w ?robot ((?joint 0.16825d0)))))))
  ;; (add-walls)
  )

(defun add-walls ()
  (setf cram-bullet-reasoning::*static-plane-texture*
        (concatenate
         'string
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "
         "                "))
  (prolog `(and (bullet-world ?w)
                (robot ?robot)
                (assert (object ?w :static-plane wall-behind-shelves
                                ((1.83 0 0) (0 0 0 1)) :normal (1 0 0) :constant 0
                                                       :disable-collisions-with (?robot)))
                (assert (object ?w :static-plane wall-behind-table
                                ((0 3 0) (0 0 0 1)) :normal (0 1 0) :constant 0
                                                    :disable-collisions-with (?robot)))))
  (setf cram-bullet-reasoning::*static-plane-texture*
        (concatenate
         'string
         "xxxxxxxxxxxxxxxx"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "x              x"
         "xxxxxxxxxxxxxxxx")))
