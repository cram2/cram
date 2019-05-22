;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defsystem cram-urdf-projection
  :author "Gayane Kazhoyan"
  :license "BSD"

  :depends-on (alexandria ; for CURRY in low-level perception
               roslisp-utilities ; for rosify-lisp-name

               cram-projection
               cram-prolog
               cram-designators
               cram-utilities ; for lazy list stuff with prolog
               cram-process-modules

               cl-transforms
               cl-transforms-stamped
               cl-tf ; for TF transformer to be used without ROS inside projection
               cram-tf                ; for pose conversions
               cram-robot-interfaces ; for reading robot descriptions
               cram-occasions-events ; for on-event for updating TF transformer
               cram-plan-occasions-events ; for robot-state-changed handler
               cram-common-designators ; for projection process modules
               cram-common-failures ; for throwing failures in low-level.lisp

               cram-bullet-reasoning ; for moving the robot in the bullet world
               cram-bullet-reasoning-belief-state ; for special projection variable definition

               cram-ik-interface)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "low-level" :depends-on ("package"))
     (:file "process-modules" :depends-on ("package" "low-level"))
     (:file "projection-environment" :depends-on ("package" "process-modules"))
     (:file "tf" :depends-on ("package" "projection-environment"))))))
