;;;
;;; Copyright (c) 2017, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defsystem cram-pr2-plan-transformation
  :author "artnie"
  :license "BSD"
  
  :depends-on (roslisp-utilities ; for ros-init-function

               cl-transforms
               cl-transforms-stamped
               cl-tf
               cram-tf

               cram-executive
               cram-prolog
               cram-projection
               cram-occasions-events

               cram-common-failures
               cram-mobile-pick-place-plans

               cram-knowrob-pick-place
               cram-cloud-logger

               cram-physics-utils     ; for reading "package://" paths
               cl-bullet ; for handling BOUNDING-BOX datastructures

               cram-bullet-reasoning-belief-state ;; using cram-bullet-reasoning
               cram-bullet-reasoning-utilities
               cram-btr-visibility-costmap ;
               cram-btr-spatial-relations-costmap ;

               cram-robot-pose-gaussian-costmap ;; contains cram-location-costmap
               cram-occupancy-grid-costmap ;; contains cram-location-costmap
               cram-location-costmap

               cram-pr2-projection      ; for with-simulated-robot, depends on bullet-resoning
               cram-pr2-projection-reasoning ; for projection-based reasoning
               cram-pr2-description ;
               cram-pr2-fetch-deliver-plans
               cram-pr2-environment-manipulation ;

               cram-semantic-map-costmap

               ;; cram-robosherlock
               ;; cram-bullet-reasoning-designators
               ;; cram-bullet-reasoning-costmap ; not using any spatial relation cms yet
               ;; cram-bullet-reasoning-designators ; not using visibility cm or collision checks
               
               )
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "setup" :depends-on ("package"))
     (:file "plan-transformation" :depends-on ("package"))
     (:file "predicates" :depends-on ("package"))
     (:file "projection-demo" :depends-on ("package"))
     (:file "demo" :depends-on ("package" "setup" "projection-demo"))
     (:file "demo-transformation-rules" :depends-on ("package" "plan-transformation" "predicates"))
     (:file "task-tree-functions" :depends-on ("package" "plan-transformation" "predicates"))
     
     ;; (:file "costmaps" :depends-on ("package"))
     ;; (:file "designators" :depends-on ("package"))
     ;; (:file "evaluation" :depends-on ("package"))
     ;; (:file "plans" :depends-on ("package" "utils" "designators"))
    ))))
