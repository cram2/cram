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

  :depends-on (alexandria ; for flatten

               roslisp
               roslisp-utilities

               cl-transforms
               cl-transforms-stamped

               cram-designators
               cram-process-modules
               cram-prolog
               cram-occasions-events ; for updating giskard collision scene on events

               cram-simple-actionlib-client
               cram-tf
               cram-robot-interfaces
               cram-common-failures
               cram-common-designators
               cram-plan-occasions-events

               cram-bullet-reasoning ; also for updating giskard collision scene
               cram-bullet-reasoning-belief-state ; for *kitchen-parameter*

               cram-joint-states ; for joint-interface to send current joint state

               giskard_msgs-msg
               giskard_msgs-srv)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "collision-scene" :depends-on ("package"))
     (:file "action-client" :depends-on ("package" "collision-scene"))
     (:file "hash-table-conversions" :depends-on ("package"))
     (:file "making-goal-messages" :depends-on ("package" "hash-table-conversions"))
     (:file "base-goals" :depends-on ("package"
                                      "action-client"
                                      "making-goal-messages"))
     (:file "arm-goals" :depends-on ("package"
                                     "action-client"
                                     "making-goal-messages"
                                     ;; because we constrain base velocity
                                     "base-goals"))
     (:file "torso-goals" :depends-on ("package"
                                       "making-goal-messages"
                                       "action-client"))
     (:file "gripper-goals" :depends-on ("package"
                                         "making-goal-messages"
                                         "action-client"))
     (:file "neck-goals" :depends-on ("package"
                                      "making-goal-messages"
                                      "action-client"))
     (:file "environment-manipulation-goals" :depends-on ("package"
                                                          "making-goal-messages"
                                                          "action-client"
                                                          ;; because we
                                                          ;; constrain
                                                          ;; base velocity
                                                          "base-goals"))
     (:file "misc-goals" :depends-on ("package"
                                      "making-goal-messages"
                                      "action-client"))
     (:file "process-module" :depends-on ("package"
                                          "arm-goals"
                                          "base-goals"
                                          "torso-goals"
                                          "gripper-goals"
                                          "neck-goals"
                                          "environment-manipulation-goals"))))))
