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

(defsystem cram-mobile-pick-place-plans
  :author "Gayane Kazhoyan"
  :maintainer "Gayane Kazhoyan"
  :license "BSD"

  :depends-on (cl-transforms
               cl-transforms-stamped
               ;; cl-tf2 ; in grasping overwrite tf transformer with tf2

               roslisp ; for debug statements
               roslisp-utilities

               cram-language
               cram-prolog
               cram-designators
               cram-occasions-events
               cram-executive
               cram-utilities ; for cut:var-value of prolog stuff

               cram-tf
               cram-plan-occasions-events
               cram-common-failures
               cram-object-interfaces)

  :components
  ((:module "src"
    :components
    ((:file "package")

     ;; actions such as REACHING, LIFTING, GRASPING, GRIPPING, LOOKING-AT, etc.
     (:file "atomic-action-plans" :depends-on ("package"))
     (:file "atomic-action-designators" :depends-on ("package" "atomic-action-plans"))

     ;; PICKING-UP and PLACING actions
     (:file "pick-place-plans" :depends-on ("package" "atomic-action-designators"))
     (:file "pick-place-designators" :depends-on ("package"
                                                  "pick-place-plans"))

     ;; high-level plans such as DRIVE-AND-PICK-UP, PERCEIVE, etc.
     (:file "high-level-plans" :depends-on ("package"
                                            "atomic-action-designators"
                                            "pick-place-designators"))))))
