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

(in-package :boxy-proj)

(defvar *last-timeline* nil)

(cram-projection:define-projection-environment boxy-bullet-projection-environment
  :special-variable-initializers
  ((cram-tf:*transformer*
    (make-instance 'cl-tf:transformer))
   ;; TODO: use custom tf topic "tf_sim"
   ;; For that first change tf2_ros/TransformListener to accept custom topic names
   ;; (*current-bullet-world* (cl-bullet:copy-world btr:*current-bullet-world*))
   (cram-bullet-reasoning:*current-timeline*
    (btr:timeline-init btr:*current-bullet-world*))
   (desig:*default-role*
    'projection-role)
   ;; (cut:*timestamp-function* #'projection-timestamp-function)
   (cram-bullet-reasoning-belief-state::*object-identifier-to-instance-mappings*
    (alexandria:copy-hash-table
     cram-bullet-reasoning-belief-state::*object-identifier-to-instance-mappings*))
   (cet:*episode-knowledge*
    cet:*episode-knowledge*))
  :process-module-definitions
  (boxy-proj-navigation
   boxy-proj-torso
   boxy-proj-ptu
   boxy-proj-perception
   boxy-proj-grippers
   boxy-proj-arms)
  :startup (progn
             (cram-bullet-reasoning-belief-state:set-tf-from-bullet)
             (cram-bullet-reasoning-belief-state:update-bullet-transforms))
  :shutdown (setf *last-timeline* cram-bullet-reasoning:*current-timeline*))


(def-fact-group boxy-available-pms (cpm:available-process-module
                                    cpm:projection-running)

  (<- (cpm:available-process-module ?pm)
    (bound ?pm)
    (once (member ?pm (boxy-proj-navigation
                       boxy-proj-torso
                       boxy-proj-ptu
                       boxy-proj-perception
                       boxy-proj-grippers
                       boxy-proj-arms)))
    (symbol-value cram-projection:*projection-environment* boxy-bullet-projection-environment))

  (<- (cpm::projection-running ?pm)
    ;; (bound ?pm)
    (once (member ?pm (boxy-proj-navigation
                       boxy-proj-torso
                       boxy-proj-ptu
                       boxy-proj-perception
                       boxy-proj-grippers
                       boxy-proj-arms)))
    (symbol-value cram-projection:*projection-environment* boxy-bullet-projection-environment)))


(defmacro with-simulated-robot (&body body)
  `(let ((results
           (proj:with-projection-environment boxy-bullet-projection-environment
             (cpl-impl::named-top-level (:name :top-level)
               ,@body))))
     (car (cram-projection::projection-environment-result-result results))))

(defmacro with-projected-robot (&body args)
  "Alias for WITH-SIMULATED-ROBOT."
  `(with-simulated-robot ,@args))


#+below-is-a-very-simple-example-of-how-to-use-projection
(
 (let ((robot (cl-urdf:parse-urdf (roslisp:get-param "robot_description")))
       (kitchen (cl-urdf:parse-urdf (roslisp:get-param "kitchen_description"))))
   (prolog:prolog `(and
                    (btr:bullet-world ?w)
                    (btr:debug-window ?w)
                    (assert ?w (btr:object :static-plane floor ((0 0 0) (0 0 0 1))
                                           :normal (0 0 1) :constant 0))
                    (assert ?w (btr:object :semantic-map sem-map ((0 0 0) (0 0 0 1))
                                           :urdf ,kitchen))
                    (cram-robot-interfaces:robot ?robot)
                    (assert ?w (btr:object :urdf ?robot ((0 0 0) (0 0 0 1))
                                           :urdf ,robot)))))

 (ros-load-manifest:load-system "cram_executive" :cram-executive)

 (proj:with-projection-environment boxy-proj::boxy-bullet-projection-environment
   (cpl:top-level
     (exe:perform
      (let ((?pose (cl-tf:make-pose-stamped
                    cram-tf:*robot-base-frame* 0.0
                    (cl-transforms:make-3d-vector -4 -5 0)
                    (cl-transforms:make-identity-rotation))))
        (desig:a motion (type going) (pose ?pose))))
     (exe:perform
      (let ((?pose (cl-tf:make-pose-stamped
                    cram-tf:*robot-base-frame* 0.0
                    (cl-transforms:make-3d-vector 0.5 0.3 1.0)
                    (cl-transforms:make-identity-rotation))))
        (desig:a motion (type moving-tcp) (left-pose ?pose))))))
)
