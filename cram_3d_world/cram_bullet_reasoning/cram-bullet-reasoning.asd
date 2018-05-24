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

(defsystem cram-bullet-reasoning
    :author "Lorenz Moesenlechner"
    :license "BSD"

    :depends-on (alexandria
                 cram-prolog
                 cl-bullet
                 cl-bullet-vis
                 cl-urdf
                 cl-transforms-stamped
                 cl-transforms
                 roslisp
                 moveit_msgs-srv
                 moveit_msgs-msg
                 sensor_msgs-msg
                 ;; household_objects_database_msgs-msg
                 ;; household_objects_database_msgs-srv
                 cram-location-costmap
                 cram-designators
                 roslisp-utilities
                 cram-semantic-map-utils
                 cram-object-interfaces
                 cram-robot-interfaces
                 cram-occasions-events
                 cram-utilities ; lazy in pose-generators
                 cram-occasions-events ; for temporal reasoning
                 cram-tf
                 cram-physics-utils)
    :components
    ((:module
      "src"
      :components
      ((:file "package")
       (:file "reasoning-world" :depends-on ("package"))
       (:file "textures" :depends-on ("package"))
       (:file "utils" :depends-on ("package"))
       (:file "objects" :depends-on ("package" "reasoning-world" "textures" "utils"))
       (:file "items" :depends-on ("package" "objects" "utils"))
       ;; (:file "ros-household-object-database"
       ;;  :depends-on ("package" "objects" "items"))
       (:file "aabb" :depends-on ("package" "objects"))
       (:file "debug-window" :depends-on ("package"))
       (:file "world-utils" :depends-on ("package" "reasoning-world" "objects"))
       (:file "world-facts"
        :depends-on ("package" "reasoning-world" "items" "objects" "debug-window" "world-utils"))
       (:file "prolog-handlers" :depends-on ("package" "reasoning-world" "world-facts"))
       (:module "temporal-reasoning"
        :depends-on ("package" "reasoning-world" "world-facts" "utils")
        :components
        ((:file "events")
         (:file "timeline" :depends-on ("events"))
         (:file "prolog")))
       (:file "pose-generators" :depends-on ("package" "utils" "aabb" "world-facts"))
       (:file "pose-sampling-facts" :depends-on ("package" "world-facts" "pose-generators"))
       (:file "pose-facts"
        :depends-on ("package" "aabb" "world-utils" "pose-generators" "objects" "world-facts"))
       (:file "robot-model" :depends-on ("package" "objects" "utils" "reasoning-world"))
       (:file "robot-model-utils" :depends-on ("package" "robot-model"))
       (:file "robot-model-facts"
        :depends-on ("package" "world-facts" "prolog-handlers" "robot-model" "robot-model-utils"))
       (:file "gl-scenes" :depends-on ("package" "debug-window"))
       (:file "visibility-reasoning" :depends-on ("package" "gl-scenes"))
       (:file "visibility-facts"
        :depends-on ("package" "world-facts" "visibility-reasoning" "robot-model-facts"))
       ;; (:file "reachability" :depends-on ("package" "robot-model-utils"))
       ;; (:file "reachability-facts"
       ;;  :depends-on ("package" "world-facts" "prolog-handlers" "robot-model-utils" "reachability"))
       (:file "semantic-map" :depends-on ("package" "objects" "utils"))
       (:file "simple-semantic-map" :depends-on ("package" "semantic-map"))
       (:file "urdf-semantic-map"
        :depends-on ("package" "reasoning-world" "semantic-map" "robot-model"))
       (:file "semantic-map-facts"
        :depends-on ("package" "semantic-map" "urdf-semantic-map" "world-facts"))
       (:file "articulated-objects" :depends-on ("package" "semantic-map"))
       (:file "action-facts"
        :depends-on ("package" "world-facts" "prolog-handlers" "articulated-objects"))))))
