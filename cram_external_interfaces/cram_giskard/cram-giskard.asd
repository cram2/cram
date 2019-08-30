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

(defsystem cram-giskard
  :author "Gayane Kazhoyan"
  :maintainer "Gayane Kazhoyan"
  :license "BSD"

  :depends-on (roslisp
               roslisp-utilities
               cl-transforms
               cl-transforms-stamped
               cram-tf
               cram-common-failures
               cram-designators
               cram-process-modules
               cram-prolog
               cram-common-designators
               cram-occasions-events ; for updating giskard collision scene on events
               cram-plan-occasions-events
               cram-bullet-reasoning ; also for updating giskard collision scene
               cram-bullet-reasoning-belief-state ; for *kitchen-parameter*
               cram-joint-states ; for joint-interface to send current joint state
               cram-simple-actionlib-client
               giskard_msgs-msg
               giskard_msgs-srv)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "collision-scene" :depends-on ("package"))
     (:file "action-client" :depends-on ("package"))
     (:file "cartesian-interface" :depends-on ("package" "action-client"))
     (:file "joint-interface" :depends-on ("package" "action-client"))
     (:file "base-goals" :depends-on ("package" "action-client" "joint-interface"))
     (:file "torso-goals" :depends-on ("package" "action-client"))
     (:file "process-module" :depends-on ("package" "cartesian-interface" "joint-interface"
                                                    "base-goals" "torso-goals"))))))
