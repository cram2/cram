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

(defsystem cram-plan-library
  :author "Lorenz Moesenlechner"
  :license "BSD"
  :description "A library of plans for the IAS kitchen environment"

  :depends-on (cram-language
               cram-reasoning
               process-modules
               designators
               cram-roslisp-common
               cram-plan-knowledge
               cram-plan-failures
               cram-projection
               designators-ros
               alexandria)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "goal-declarations" :depends-on ("package"))
             (:file "achieve-loc" :depends-on ("package"
                                               "goal-declarations"
                                               "achieve-object-manipulation"
                                               "achieve-container-manipulation"
                                               "achieve-ptu"
                                               "utilities"))
             (:file "achieve-ptu"
              :depends-on ("package" "goal-declarations" "with-designators"))
             (:file "at-location" :depends-on ("package"))
             (:file "with-designators" :depends-on ("package"))
             (:file "perceive-object"
              :depends-on ("package" "goal-declarations" "at-location" "utilities" "with-designators"))
             (:file "perceive-state" :depends-on ("package" "goal-declarations"))
             (:file "achieve-container-manipulation" :depends-on ("achieve-object-manipulation"))
             (:file "achieve-object-manipulation"
              :depends-on ("package"
                           "goal-declarations"
                           "at-location"
                           "utilities"
                           "with-designators"))
             (:file "facts" :depends-on ("package" "goal-declarations" "utilities" "perceive-object"))
             (:file "perform" :depends-on ("package" "goal-declarations"))
             (:file "utilities" :depends-on ("package"))))))
