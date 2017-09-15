;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defsystem cram-pr2-low-level
  :author "Gayane Kazhoyan"
  :maintainer "Gayane Kazhoyan"
  :license "BSD"

  :depends-on (roslisp
               roslisp-utilities
               cl-transforms
               cl-transforms-stamped
               cl-tf
               actionlib
               alexandria
               yason
               cram-language
               cram-tf
               cram-utilities
               cram-prolog
               cram-robot-interfaces
               cram-pr2-description ; for tool frames
               cram-common-failures
               sensor_msgs-msg
               geometry_msgs-msg
               visualization_msgs-msg
               moveit_msgs-msg
               moveit_msgs-srv
               pr2_controllers_msgs-msg
               trajectory_msgs-msg
               move_base_msgs-msg
               giskard_msgs-msg
               iai_robosherlock_msgs-msg
               iai_robosherlock_msgs-srv)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "low-level-common" :depends-on ("package"))
     (:file "joint-states" :depends-on ("package"))
     (:file "base-controller" :depends-on ("package"))
     (:file "nav-pcontroller" :depends-on ("package" "low-level-common" "base-controller"))
     (:file "torso" :depends-on ("package" "low-level-common"))
     (:file "gripper" :depends-on ("package" "low-level-common"))
     (:file "ptu" :depends-on ("package" "low-level-common"))
     (:file "joint-trajectory" :depends-on ("package" "low-level-common"))
     (:file "kinematics-trajectory" :depends-on ("package"
                                                 "joint-trajectory"
                                                 "joint-states"))
     (:file "giskard-common" :depends-on ("package" "low-level-common"))
     (:file "giskard-cartesian" :depends-on ("package" "low-level-common" "giskard-common"))
     (:file "giskard-joint" :depends-on ("package" "low-level-common" "giskard-common"
                                                   "joint-states"))
     (:file "yaml-builder" :depends-on ("package"))
     (:file "giskard-yaml" :depends-on ("package" "giskard-common" "yaml-builder"))
     (:file "json-parser" :depends-on ("package"))
     (:file "robosherlock-json" :depends-on ("package" "json-parser" "low-level-common"))))))
