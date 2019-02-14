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
	:depends-on (cram-tf
               roslisp
               cram-language
               cram-json-prolog
               std_msgs-msg
               cl-transforms
               cl-transforms-stamped
               cl-tf
               cram-pr2-pick-place-demo
               cram-knowrob-pick-place
               cram-manipulation-interfaces
               cram-semantic-map)
	:components

	((:module "src"
	  :components
	  ((:file "package")
     ;; whitelist of mesh files that should be used from the unreal kitchen
     (:file "mesh-list" :depends-on ("package"))
     ;; communication with KnowRob is implemented here
     (:file "queries" :depends-on ("package"))
     ;; name mappings between CRAM and KnowRob
     (:file "mapping-urdf-semantic" :depends-on ("package"))
     ;; initialisation
     (:file "init" :depends-on ("package" "mesh-list" "mapping-urdf-semantic"))
     ;; specifies how to grasp obj
     (:file "grasping" :depends-on ("package" "queries" "mapping-urdf-semantic"))
     ;; the logic of transferring VR data onto robot, all transformations etc
     (:file "robot-positions-calculations" :depends-on ("package"
                                                        "queries"
                                                        "mapping-urdf-semantic"))
     ;; utilities for moving objects to poses
     (:file "move-utils" :depends-on ("package" "mapping-urdf-semantic"
                                                "robot-positions-calculations"))
     ;; move-to-object, pick, place and pick-and-place plans
     (:file "plans" :depends-on ("package" "move-utils" "queries"
                                           "mapping-urdf-semantic"))
     ;; calling plans with correct arguments
     (:file "plan-execution" :depends-on ("package" "plans" "move-utils" "queries"))
     ;; plans for demonstrations
     (:file "demo-plans" :depends-on ("package" "queries" "plans" "plan-execution"))
     ;; only used for debugging
     (:file "debugging-utils" :depends-on ("package" "queries"
                                                     "robot-positions-calculations"
                                                     "init"))))))
