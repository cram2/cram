;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(defsystem pr2-manipulation-process-module
  :author "Lorenz Moesenlechner"
  :license "BSD"

  :depends-on (designators-ros
               process-modules
               actionlib
               cram-roslisp-common
               cram-reasoning
               cram-plan-failures
               cram-plan-knowledge
               cram-projection
               cram-pr2-knowledge
               pr2_controllers_msgs-msg
               pr2_msgs-msg
               pr2_msgs-srv
               kinematics_msgs-srv
               arm_navigation_msgs-msg
               trivial-garbage
               std_srvs-srv
               alexandria
               semantic-map-collision-environment
               object_manipulation_msgs-msg
               object_manipulation_msgs-srv
               pr2_gripper_sensor_msgs-msg
               arm_navigation_msgs-srv
               pr2_mechanism_msgs-srv
               pr2-manipulation-knowledge
               constraint_msgs-msg
               cram-feature-constraints)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "utils" :depends-on ("package"))
             (:file "knowrob_utils" :depends-on ("package"))
             (:file "tf-utils" :depends-on ("package"))
             (:file "grasping" :depends-on ("package" "utils"))
             (:file "sensors" :depends-on ("package" "utils"))
             (:file "motion-library" :depends-on ("package"))
             (:file "controller-manager" :depends-on ("package"))
             (:file "kinematics" :depends-on ("package"))
             (:file "collision-environment" :depends-on ("package"))
             (:file "process-module"
              :depends-on ("package" "kinematics" "collision-environment" "controller-manager" "motion-library" "grasping" "sensors"))
             (:file "events" :depends-on ("package" "collision-environment"))
             (:file "designator" :depends-on ("package" "knowrob_utils" "process-module"))
             (:file "action-handlers" :depends-on ("package" "process-module" "feature-controllers" "tf-utils"))
             (:file "feature-controllers" :depends-on ("package" "controller-manager"))
             (:file "feature-controller-tests" 
              :depends-on ("package" "feature-controllers" "controller-manager" "process-module" "tf-utils"))))))
