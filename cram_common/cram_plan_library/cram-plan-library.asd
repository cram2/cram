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

(defsystem cram-plan-library
  :author "Lorenz Moesenlechner"
  :license "BSD"
  :description "A library of plans for the IAS kitchen environment"

  :depends-on (cram-language
               cram-process-modules
               cram-designators
               cl-transforms-stamped
               cram-language-designator-support
               cram-plan-occasions-events
               cram-occasions-events
               cram-plan-failures
               cram-projection
               cram-utilities
               cram-tf
               alexandria)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "utilities" :depends-on ("package"))
             (:file "goal-declarations" :depends-on ("package"))
             (:file "perform" :depends-on ("package" "goal-declarations"))
             (:file "perceive-state" :depends-on ("package" "goal-declarations"))
             (:file "achieve-ptu" :depends-on ("package" "goal-declarations"))
             (:file "at-location" :depends-on ("package"))
             (:file "perceive-object"
              :depends-on ("package" "goal-declarations" "at-location" "utilities"))
             (:file "achieve-loc" :depends-on ("package"
                                               "goal-declarations"
                                               "achieve-object-manipulation"
                                               "achieve-container-manipulation"
                                               "achieve-ptu"
                                               "utilities"))
             (:file "achieve-object-manipulation"
              :depends-on ("package" "goal-declarations" "at-location" "utilities"))
             (:file "achieve-container-manipulation"
              :depends-on ("achieve-object-manipulation"))
             (:file "achieve-everyday-activities"
              :depends-on ("package" "achieve-container-manipulation"
                           "achieve-object-manipulation" "perceive-object"
                           "goal-declarations" "utilities" "perform"
                           "at-location"))))))
