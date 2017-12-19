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

(defsystem cram-pr2-plans
  :author "Gayane Kazhoyan"
  :maintainer "Gayane Kazhoyan"
  :license "BSD"

  :depends-on (cl-transforms
               cl-transforms-stamped
               cl-tf2
               cram-tf

               cram-language
               cram-prolog
               cram-designators
               cram-process-modules
               cram-math
               cram-occasions-events
               cram-executive
               cram-utilities ; for cut:var-value of prolog stuff

               cram-plan-occasions-events
               ;; cram-bullet-reasoning-belief-state ; for event handling ; using own stuff for now

               giskard_msgs-msg
               giskard_msgs-srv

               cram-common-failures
               ;; cram-pr2-process-modules ; only needed when running on real robot

               cram-semantic-map-costmap
               cram-robot-pose-gaussian-costmap
               pr2-reachability-costmap

               ;; cram-beliefstate ; got rid of logging hooks, going to use other logging
               )

  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "grasping" :depends-on ("package"))
     (:file "occasions-events" :depends-on ("package"))
     (:file "designators-pick-and-place" :depends-on ("package" "grasping"))
     (:file "costmaps" :depends-on ("package"))
     (:file "pick-and-place" :depends-on ("package" "grasping"
                                                    "designators-pick-and-place"
                                                    "costmaps"))
     (:file "learned-constraints-service" :depends-on ("package"))
     (:file "designators-pour" :depends-on ("package" "grasping"
                                                      "designators-pick-and-place"
                                                      "learned-constraints-service"))
     (:file "ros-uri-parser" :depends-on ("package"))
     (:file "read-controller-yamls" :depends-on ("package" "ros-uri-parser"))
     (:file "constraints-into-controller" :depends-on ("package"))
     (:file "pour" :depends-on ("package" "grasping" "designators-pour"
                                          "read-controller-yamls"
                                          "constraints-into-controller"))
     (:file "demo-plan" :depends-on ("package" "pour"))))))
