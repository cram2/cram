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

(defsystem cram-bullet-reasoning-belief-state
  :author "Lorenz Moesenlechner"
  :license "BSD"

  :depends-on (cram-prolog
               cram-utilities
               cram-projection
               cram-designators
               cram-bullet-reasoning
               cram-occasions-events
               cram-plan-occasions-events
               cram-language ; for DEFINE-TASK-VARIABLE in utitlities
               cram-physics-utils ; for object designator mesh stuff
               ;; ros-init-function-s for time
               roslisp-utilities
               roslisp
               cram-robot-interfaces
               cram-tf
               cl-tf ; for tf broadcaster and for setting transforms from bullet to tf
               cl-transforms-stamped
               cl-transforms
               cl-bullet
               tf2_msgs-msg
               geometry_msgs-msg
               giskard_msgs-msg
               giskard_msgs-srv
               shape_msgs-msg
               cram-json-prolog)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "belief-state" :depends-on ("package"))
     (:file "time" :depends-on ("package"))
     (:file "tf" :depends-on ("package"))
     (:file "broadcaster" :depends-on ("package" "tf"))
     (:file "environment-joint-publisher" :depends-on ("package"))
     (:file "giskard-environment-client" :depends-on ("package"))
     (:file "object-perceptions" :depends-on ("package"))
     (:file "occasions" :depends-on ("package" "object-perceptions"))
     (:file "event-handlers" :depends-on ("package" "object-perceptions"
                                                    "giskard-environment-client"))))))
