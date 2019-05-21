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

(in-package :demo)

;; roslaunch cram_boxy_assembly_demo sandbox.launch

(defvar *kitchen-urdf* nil)
(defparameter *robot-parameter* "robot_description")
(defparameter *kitchen-parameter* "kitchen_description")

(defun setup-bullet-world ()
  (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))

  (let ((robot (or rob-int:*robot-urdf*
                   (setf rob-int:*robot-urdf*
                         (cl-urdf:parse-urdf
                          (roslisp:get-param *robot-parameter*)))))
        (kitchen (or *kitchen-urdf*
                     (let ((kitchen-urdf-string
                             (roslisp:get-param *kitchen-parameter* nil)))
                       (when kitchen-urdf-string
                         (setf *kitchen-urdf* (cl-urdf:parse-urdf
                                               kitchen-urdf-string)))))))
    ;; set Boxy URDF root link to be base_footprint not odom,
    ;; as with odom lots of problems concerning object-pose in bullet happen
    (setf (slot-value rob-int:*robot-urdf* 'cl-urdf:root-link)
          (or (gethash cram-tf:*robot-base-frame*
                       (cl-urdf:links rob-int:*robot-urdf*))
              (error "[setup-bullet-world] cram-tf:*robot-base-frame* was undefined or smt.")))
    ;; get rid of Boxy's camera obstacle thing, it's bad for visibility reasoning
    ;; it's an annoying hack anyway...
    ;; (setf (slot-value
    ;;        (gethash "neck_obstacle"
    ;;                 (cl-urdf:links rob-int:*robot-urdf*))
    ;;        'cl-urdf:collision)
    ;;       NIL)
    ;; (setf (slot-value
    ;;        (gethash "neck_look_target"
    ;;                 (cl-urdf:links rob-int:*robot-urdf*))
    ;;        'cl-urdf:collision)
    ;;       NIL)

    (assert
     (cut:force-ll
      (prolog `(and
                (btr:bullet-world ?w)
                (btr:debug-window ?w)
                (btr:assert ?w (btr:object :static-plane :floor ((0 0 0) (0 0 0 1))
                                                         :normal (0 0 1) :constant 0))
                (btr:assert ?w (btr:object :urdf :kitchen ((0 0 0) (0 0 0 1))
                                                 :collision-group :static-filter
                                                 :collision-mask (:default-filter
                                                                  :character-filter)
                                                 :urdf ,kitchen
                                                 :compound T))
                (-> (cram-robot-interfaces:robot ?robot)
                    (btr:assert ?w (btr:object :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,robot))
                    (warn "ROBOT was not defined. Have you loaded a robot package?")))))))

  (let ((robot-object (btr:get-robot-object)))
    (if robot-object
        (btr:set-robot-state-from-tf cram-tf:*transformer* robot-object)
        (warn "ROBOT was not defined. Have you loaded a robot package?"))))

(defun init-projection ()
  (def-fact-group costmap-metadata ()
    (<- (location-costmap:costmap-size 12 12))
    (<- (location-costmap:costmap-origin -6 -6))
    (<- (location-costmap:costmap-resolution 0.05))

    (<- (location-costmap:costmap-padding 0.5))
    (<- (location-costmap:costmap-manipulation-padding 0.5))
    (<- (location-costmap:costmap-in-reach-distance 1.0))
    (<- (location-costmap:costmap-reach-minimal-distance 0.2))
    (<- (location-costmap:visibility-costmap-size 2.5))
    (<- (location-costmap:orientation-samples 2))
    (<- (location-costmap:orientation-sample-step 0.1)))

  (setf cram-tf:*transformer* (make-instance 'cl-tf2:buffer-client))

  (setup-bullet-world)

  (setf cram-tf:*tf-default-timeout* 2.0)

  (setf prolog:*break-on-lisp-errors* t)

  (cram-bullet-reasoning:clear-costmap-vis-object)

  (btr:add-objects-to-mesh-list "assembly_models" :directory "fixtures" :extension "stl")
  (btr:add-objects-to-mesh-list "assembly_models" :directory "battat/convention" :extension "stl"))

(roslisp-utilities:register-ros-init-function init-projection)
