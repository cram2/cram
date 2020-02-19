;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
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

(asdf:defsystem cram-knowrob-vr
	:depends-on (alexandria ; for shuffle
               cram-tf
               roslisp
               cram-language
               cram-json-prolog
               std_msgs-msg
               cl-transforms
               cl-transforms-stamped
               cl-tf
               cram-object-knowledge
               cram-manipulation-interfaces
               cram-semantic-map
               cram-bullet-reasoning
               cram-bullet-reasoning-utilities
               cram-bullet-reasoning-belief-state
               cram-executive
               cram-designators
               cram-prolog
               cram-common-failures
               cram-urdf-projection
               ;; cram-pr2-description
               cram-robot-interfaces
               cram-fetch-deliver-plans
               ;; costmaps are loaded for comparison with heuristics experiments
               cram-location-costmap
               cram-btr-visibility-costmap
               cram-btr-spatial-relations-costmap
               cram-robot-pose-gaussian-costmap
               ;; cram-occupancy-grid-costmap
               )
	:components

	((:module "src"
	  :components
	  ((:file "package")
     ;; whitelist of mesh files that should be used from the unreal kitchen
     (:file "mesh-list" :depends-on ("package"))
     ;; name mappings between CRAM and KnowRob
     (:file "mapping-urdf-semantic" :depends-on ("package"))
     ;; initialisation
     (:file "init" :depends-on ("package" "mesh-list" "mapping-urdf-semantic"))
     ;; communication with KnowRob is implemented here
     (:file "queries" :depends-on ("package"))
     ;; the logic of transferring VR data onto robot, all transformations etc
     (:file "query-based-calculations" :depends-on ("package"
                                                    "queries"
                                                    "mapping-urdf-semantic"))

     ;; visibility and reachability location resolution through VR
     (:file "designator-integration" :depends-on ("package"
                                                  "init" ; for *kvr-enabled*
                                                  "query-based-calculations"))
     ;; experiment log csv file generator stuff
     (:file "experiment-log-generator" :depends-on ("package"))
     ;; plans that call fetch and deliver actions
     (:file "fetch-and-deliver-based-demo" :depends-on ("package"
                                                        "init" ; for *kvr-enabled*
                                                        "experiment-log-generator"
                                                        "query-based-calculations"
                                                        "designator-integration"))
     ;; (:file "plan-execution" :depends-on ("package"))
     ;; (:file "demo-plans" :depends-on ("package"))
     ;; arrows for visualization plots and other utils
     (:file "debugging-utils" :depends-on ("package"))))))
