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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(defparameter *default-namespace* "cram_hpn")

(roslisp:def-service-callback hpn_cram_msgs-srv:SpawnWorld (item_geometry_vector)
  (roslisp:ros-info (cram-hpn spawn-world-server) "Spawning world.")
  (roslisp:make-response :success
                         (spawn-world item_geometry_vector)))

(defun run-spawn-world-server (&optional (namespace *default-namespace*))
  (roslisp:register-service
   (concatenate 'string namespace "/spawn_world")
   'hpn_cram_msgs-srv:spawnworld)
  (roslisp:ros-info (cram-hpn spawn-world-server) "Spawn world server ready."))


(roslisp:def-service-callback hpn_cram_msgs-srv:SetWorldState (robot_pose
                                                               robot_joint_state
                                                               item_in_space_vector)
  ;; (roslisp:ros-info (cram-hpn set-world-state-server) "Setting world.")
  (roslisp:make-response :success
                         (set-world-state robot_pose robot_joint_state item_in_space_vector)))

(defun run-set-world-state-server (&optional (namespace *default-namespace*))
  (roslisp:register-service
   (concatenate 'string namespace "/set_world_state")
   'hpn_cram_msgs-srv:setworldstate)
  (roslisp:ros-info (cram-hpn set-world-state-server) "Set world state server ready."))



(defun run-servers (&optional (namespace *default-namespace*))
  (run-spawn-world-server namespace)
  (run-set-world-state-server namespace)
  (commander:run-perform-server namespace))

(roslisp-utilities:register-ros-init-function run-servers)

;; (defun main ()
;;   (cram-process-modules:with-process-modules-running
;;       (rs:robosherlock-perception-pm giskard:giskard-pm ;; navp:navp-pm
;;        pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
;;     (roslisp:spin-until nil 100)))
;;
;; (defun commander:perform-with-pms-running (designator)
;;    (cpl-impl::named-top-level (:name :top-level)
;;        (exe:perform designator))
;;   ;; (sleep 0.5)
;;   )

(defparameter *real-otherwise-simulation* nil)
(defun commander:perform-with-pms-running (designator)
  (if *real-otherwise-simulation*
      (pr2-pms:with-real-robot
        (handler-case
            (exe:perform designator)
          (cpl:plan-failure (f)
            (warn "[commander] Failure happened ~a~%" f))))
      (urdf-proj:with-simulated-robot
        (handler-case
            (exe:perform designator)
          (cpl:plan-failure (f)
            (warn "[commander] Failure happened ~a~%" f))))
      ;; (sleep 0.5)
      ))
