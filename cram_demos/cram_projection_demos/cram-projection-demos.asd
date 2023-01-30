;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(defsystem cram-projection-demos
  :author "Gayane Kazhoyan"
  :maintainer "Gayane Kazhoyan"
  :license "BSD"

  :depends-on (roslisp-utilities ; for ros-init-function

               cl-transforms
               ;; cl-transforms-stamped
               ;; cl-tf

               ;; cram_core
               cram-prolog ; for restricted ground costmap and to set break on errors
               cram-occasions-events ; for CLEAR-BELIEF
               cram-designators ; for the plan, costmap
               cram-executive ; for the plan
               ;; cram-language
               ;; cram-projection
               cram-utilities ;  for RANDOM-WITH-MINIMUM

               ;; cram_common
               cram-tf ; to set the default timeout to a small number, tf utils
               cram-location-costmap ; for the restricted ground costmap
               cram-robot-interfaces ; for ENV-NAME, VIS/REACH-DESIG
               cram-manipulation-interfaces ; for costmap, standard rotations
               cram-object-knowledge ; for manipulation knowledge
               cram-mobile-pick-place-plans ; for action desig implementations
               ;; cram-common-failures

               ;; cram_3d_world
               cram-bullet-reasoning
               cram-bullet-reasoning-belief-state ; for handling events
               cram-urdf-projection ; for with-simulated-robot
               cram-fetch-deliver-plans ; for action desig implementations
               cram-urdf-environment-manipulation ; for action desig implementations

               ;; costmaps
               cram-btr-visibility-costmap
               cram-btr-spatial-relations-costmap
               cram-robot-pose-gaussian-costmap
               ;; cram-occupancy-grid-costmap

               ;; robot descriptions
               cram-pr2-description
               cram-boxy-description
               cram-donbot-description
               cram-kukabot-description
               cram-tiago-description
               ;; cram-hsrb-description
               )
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "costmaps" :depends-on ("package"))
     (:file "setup" :depends-on ("package"))
     (:file "utils" :depends-on ("package"))
     (:file "assembly-demo" :depends-on ("package" "utils"))
     (:file "household-demo" :depends-on ("package" "utils"))
     (:file "retail-demo" :depends-on ("package" "utils"))
     (:file "storage-demo" :depends-on ("package" "utils"
                                                  ;; for toy parts colors
                                                  "assembly-demo"
                                                  ;; for clearing grasps
                                                  "household-demo"))
     (:file "apartment-demo" :depends-on ("package" "utils"
                                                    ;; for initializing
                                                    "household-demo"))))))
