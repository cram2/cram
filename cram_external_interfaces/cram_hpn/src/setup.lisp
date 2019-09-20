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

(in-package :hpn)

;; roslaunch cram_pr2_pick_place_demo sandbox.launch

(defun init-projection ()
  (def-fact-group costmap-metadata (costmap:costmap-size
                                    costmap:costmap-origin
                                    costmap:costmap-resolution
                                    costmap:orientation-samples
                                    costmap:orientation-sample-step)
    (<- (costmap:costmap-size 12 12))
    (<- (costmap:costmap-origin -6 -6))
    (<- (costmap:costmap-resolution 0.04))
    (<- (costmap:orientation-samples 2))
    (<- (costmap:orientation-sample-step 0.3)))

  (setf cram-bullet-reasoning-belief-state:*robot-parameter* "robot_description")
  (setf cram-bullet-reasoning-belief-state:*kitchen-parameter* "kitchen_description")

  (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
  (let ((robot (or rob-int:*robot-urdf*
                   (setf rob-int:*robot-urdf*
                         (cl-urdf:parse-urdf
                          (btr-belief::replace-all
                           (roslisp:get-param btr-belief:*robot-parameter*) "\\" "  "))))))

    ;; set robot's URDF root link to *robot-base-frame* as that's how going actions works
    (setf (slot-value rob-int:*robot-urdf* 'cl-urdf:root-link)
          (or (gethash cram-tf:*robot-base-frame*
                       (cl-urdf:links rob-int:*robot-urdf*))
              (error "[setup-bullet-world] cram-tf:*robot-base-frame* was undefined or smt.")))

    (assert
     (cut:force-ll
      (prolog `(and
                (btr:bullet-world ?w)
                (btr:debug-window ?w)
                (btr:assert ?w (btr:object :static-plane :floor ((0 0 0) (0 0 0 1))
                                                         :normal (0 0 1) :constant 0
                                                         :collision-mask (:default-filter)))
                (-> (rob-int:robot ?robot)
                    (and (btr:assert ?w (btr:object :urdf ?robot ((0 0 0) (0 0 0 1))
                                                    :urdf ,robot))
                         (rob-int:robot-joint-states ?robot :arm :left :park ?left-joint-states)
                         (assert (btr:joint-state ?world ?robot ?left-joint-states))
                         (rob-int:robot-joint-states ?robot :arm :right :park ?right-joint-states)
                         (assert (btr:joint-state ?world ?robot ?right-joint-states))
                         (rob-int:robot-torso-link-joint ?robot ?_ ?torso-joint)
                         (rob-int:joint-lower-limit ?robot ?torso-joint ?lower-limit)
                         (rob-int:joint-upper-limit ?robot ?torso-joint ?upper-limit)
                         (assert (btr:joint-state ?world ?robot ((?torso-joint ?upper-limit)))))
                    (warn "ROBOT was not defined. Have you loaded a robot package?")))))))

  (setf cram-tf:*tf-default-timeout* 2.0)

  (setf prolog:*break-on-lisp-errors* t)

  (cram-bullet-reasoning:clear-costmap-vis-object)

  (setf proj-reasoning::*projection-reasoning-enabled* nil)

  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo"))

(roslisp-utilities:register-ros-init-function init-projection)


(defun spawn-kitchen ()
  (let ((kitchen (or btr-belief:*kitchen-urdf*
                     (let ((kitchen-urdf-string
                             (roslisp:get-param btr-belief:*kitchen-parameter* nil)))
                       (when kitchen-urdf-string
                         (setf btr-belief:*kitchen-urdf*
                               (cl-urdf:parse-urdf kitchen-urdf-string)))))))
    (assert
     (cut:force-ll
      (prolog `(and
                (btr:bullet-world ?w)
                (btr:assert ?w (btr:object :urdf :kitchen ((0 0 0) (0 0 0 1))
                                                 :collision-group :static-filter
                                                 :collision-mask (:default-filter
                                                                  :character-filter)
                                           ,@(when kitchen
                                               `(:urdf ,kitchen))
                                           :compound T))))))))

(defun spawn-kitchen-bounding-box (name)
  (let ((aabb (btr:aabb
               (btr:rigid-body
                (btr:get-environment-object)
                (intern (format nil "KITCHEN.~(~a~)" name) :keyword)))))
    (btr:add-object btr:*current-bullet-world*
                    :box-item
                    name
                    (cram-tf:pose->list
                     (cl-transforms:make-pose
                      (cl-bullet:bounding-box-center aabb)
                      (cl-transforms:make-identity-rotation)))
                    :mass 0.1
                    :size `(,(/ (cl-transforms:x (cl-bullet:bounding-box-dimensions aabb)) 2)
                            ,(/ (cl-transforms:y (cl-bullet:bounding-box-dimensions aabb)) 2)
                            ,(/ (cl-transforms:z (cl-bullet:bounding-box-dimensions aabb)) 2))
                    :color '(0 0 1)
                    :item-type :kitchen)))

(defun spawn-item-bounding-box (name)
  (let* ((object (btr:object btr:*current-bullet-world* name))
         (aabb-dims (btr:calculate-bb-dims object))
         (pose (btr:pose object)))
    (btr:add-object btr:*current-bullet-world*
                    :box-item
                    (intern (format nil "~a-B-B" name) :keyword)
                    (cram-tf:pose->list pose)
                    :mass 0.1
                    :size `(,(/ (cl-transforms:x aabb-dims) 2)
                            ,(/ (cl-transforms:y aabb-dims) 2)
                            ,(/ (cl-transforms:z aabb-dims) 2))
                    :color '(1 0 0)
                    :item-type :kitchen)))


(defun initialize ()
  (sb-ext:gc :full t)

  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:kill-all-objects)

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  (init-projection)

  (btr:clear-costmap-vis-object))
