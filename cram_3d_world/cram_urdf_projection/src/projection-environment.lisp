;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :urdf-proj)

(defvar *last-timeline* nil)

;; (defmethod desig:resolve-designator :around (designator (role (eql 'projection-role)))
;;   (cram-projection:with-partially-ordered-clock-disabled *projection-clock*
;;     (call-next-method)))

(cram-projection:define-projection-environment urdf-bullet-projection-environment
  :special-variable-initializers
  ((cram-tf:*transformer*
    (make-instance 'cl-tf:transformer))
   ;; TODO: use custom tf topic "tf_sim"
   ;; For that first change tf2_ros/TransformListener to accept custom topic names
   (cram-tf:*broadcaster*
    (cram-tf:make-tf-broadcaster
     "tf_projection"
     cram-tf:*tf-broadcasting-interval*))
   ;; (*current-bullet-world* (cl-bullet:copy-world btr:*current-bullet-world*))
   (cram-bullet-reasoning:*current-timeline*
    (btr:timeline-init btr:*current-bullet-world*))
   (desig:*default-role*
    'projection-role)
   ;; (cut:*timestamp-function* #'projection-timestamp-function)
   ;; (*projection-clock*
   ;;  (make-instance 'cram-projection:partially-ordered-clock))
   (cram-bullet-reasoning-belief-state::*object-identifier-to-instance-mappings*
    (alexandria:copy-hash-table
     cram-bullet-reasoning-belief-state::*object-identifier-to-instance-mappings*))
   (cet:*episode-knowledge*
    cet:*episode-knowledge*))
  :process-module-definitions
  (urdf-proj-navigation
   urdf-proj-torso
   urdf-proj-neck
   urdf-proj-perception
   urdf-proj-grippers
   urdf-proj-arms)
  :startup (progn
             (cram-bullet-reasoning-belief-state:set-tf-from-bullet)
             (cram-bullet-reasoning-belief-state:update-bullet-transforms)
             (cram-tf:start-publishing-transforms cram-tf:*broadcaster*))
  :shutdown (progn
              (setf *last-timeline* cram-bullet-reasoning:*current-timeline*)
              (cram-tf:stop-publishing-transforms cram-tf:*broadcaster*)))


(def-fact-group projection-available-pms (cpm:available-process-module
                                          cpm:projection-running)

  (<- (cpm:available-process-module ?pm)
    (bound ?pm)
    (once (member ?pm (urdf-proj-navigation
                       urdf-proj-torso
                       urdf-proj-neck
                       urdf-proj-perception
                       urdf-proj-grippers
                       urdf-proj-arms)))
    (symbol-value cram-projection:*projection-environment* urdf-bullet-projection-environment))

  (<- (cpm::projection-running ?pm)
    ;; (bound ?pm)
    (once (member ?pm (urdf-proj-navigation
                       urdf-proj-torso
                       urdf-proj-neck
                       urdf-proj-perception
                       urdf-proj-grippers
                       urdf-proj-arms)))
    (symbol-value cram-projection:*projection-environment* urdf-bullet-projection-environment)))


(defmacro with-simulated-robot (&body body)
  `(let ((results
           (proj:with-projection-environment urdf-bullet-projection-environment
             (cpl-impl::named-top-level (:name :top-level)
               ,@body))))
     (car (cram-projection::projection-environment-result-result results))))

(defmacro with-projected-robot (&body args)
  "Alias for WITH-SIMULATED-ROBOT."
  `(with-simulated-robot ,@args))


#+below-is-a-very-simple-example-of-how-to-use-projection
(
 (ros-load-manifest:load-system "cram_urdf_projection" :cram-urdf-projection)
 (ros-load-manifest:load-system "cram_MYROBOT_description" :cram-MYROBOT-description)
 (ros-load-manifest:load-system "cram_executive" :cram-executive)

 ;; roslaunch cram_MYROBOT_bringup myrobot.launch

 (roslisp:start-ros-node "my_node")
 (cram-tf::init-tf)
 (coe:clear-belief)

 (urdf-proj:with-simulated-robot
  (exe:perform
   (let ((?pose (cl-transforms-stamped:make-pose-stamped
                 cram-tf:*robot-base-frame* 0.0
                 (cl-transforms:make-3d-vector -4 -5 0)
                 (cl-transforms:make-identity-rotation))))
     (desig:a motion (type going) (pose ?pose))))
  (exe:perform
   (desig:a motion (type moving-torso) (joint-angle 0.15)))
  (exe:perform
   (let ((?pose (cl-transforms-stamped:make-pose-stamped
                 cram-tf:*robot-base-frame* 0.0
                 (cl-transforms:make-3d-vector 1.0 0 1.0)
                 (cl-transforms:make-identity-rotation))))
     (desig:a motion (type looking) (pose ?pose))))
  (exe:perform
   (desig:a motion (type opening-gripper) (gripper left)))
  (exe:perform
   (desig:a motion (type moving-gripper-joint) (gripper right) (joint-angle 0.05)))
  (exe:perform
   (let ((?pose (cl-tf:make-pose-stamped
                 cram-tf:*robot-base-frame* 0.0
                 (cl-transforms:make-3d-vector 0.8 0.4 1.0)
                 (cl-transforms:make-identity-rotation))))
     (desig:a motion (type moving-tcp) (left-pose ?pose)))))
)
