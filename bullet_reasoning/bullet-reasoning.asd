;;;
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
;;;

(defsystem bullet-reasoning
    :author "Lorenz Moesenlechner"
    :license "BSD"
    
    :depends-on (alexandria
                 cram-reasoning
                 cl-bullet cl-bullet-vis
                 cl-json-pl-client
                 cl-urdf
                 cl-tf
                 kinematics_msgs-msg
                 kinematics_msgs-srv
                 kdl_arm_kinematics-srv
                 household_objects_database_msgs-srv
                 arm_navigation_msgs-msg
                 location-costmap
                 designators
                 cram-roslisp-common
                 cram-plan-knowledge
                 triangulate_point_cloud-srv
                 vision_srvs-srv
                 cl-semantic-map-utils
                 cram-manipulation-knowledge)
    :components
    ((:module "src"
              :components
              ((:file "package")
               (:file "utils" :depends-on ("package"))
               (:file "prolog-handlers" :depends-on ("package"))
               (:file "world-facts" :depends-on ("package"))
               (:file "pose-facts" :depends-on ("package"))
               (:file "pose-sampling-facts" :depends-on ("package" "world-facts"))
               (:file "robot-model-facts" :depends-on ("package"))
               (:file "visibility-facts" :depends-on ("package"))
               (:file "reachability-facts" :depends-on ("package"))
               (:file "semantic-map-facts" :depends-on ("package"))
               (:file "action-facts" :depends-on ("package"))
               (:file "reasoning-world" :depends-on ("package"))
               (:file "textures" :depends-on ("package"))
               (:file "objects" :depends-on ("package" "reasoning-world" "textures" "utils"))
               (:file "aabb" :depends-on ("package" "objects"))
               (:file "world-utils" :depends-on ("package"
                                                 "reasoning-world"
                                                 "objects"))
               (:file "semantic-map" :depends-on ("package"
                                                  "objects" "utils" "robot-model"
                                                  "household-objects"))
               (:file "robot-model" :depends-on ("package" "objects" "utils" "reasoning-world"))
               (:file "robot-model-utils" :depends-on ("package" "robot-model"))
               (:file "gl-scenes" :depends-on ("package" "debug-window"))
               (:file "visibility-reasoning" :depends-on ("package" "gl-scenes"))
               (:file "debug-window" :depends-on ("package"))
               (:file "household-objects" :depends-on ("package" "objects" "utils"))
               (:file "cop-objects" :depends-on ("package" "objects" "utils"))
               (:file "pose-generators" :depends-on ("package" "utils" "aabb" "world-facts"))
               (:file "reachability" :depends-on ("package" "robot-model-utils"))
               (:file "articulated-objects" :depends-on ("package" "semantic-map"))
               (:file "ros-household-object-database" :depends-on ("package" "objects" "household-objects"))
               (:module "temporal-reasoning"
                        :depends-on ("package" "reasoning-world" "world-facts")
                        :components
                        ((:file "events")
                         (:file "timeline" :depends-on ("events"))
                         (:file "prolog")))))))
