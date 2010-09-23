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

;; Contribs to build
;; (pushnew :kipla-contrib-oro *features*)
;; (pushnew :kipla-contrib-hri *features*)

(asdf:defsystem kipla
    :name "kipla"
    :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
    :version "0.1"
    :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
    :license "GPLv3"
    :description "Cognitive kitchen planner and coordinator"
    :long-description "Cogtinive kitchen planner and coordinator"

    :depends-on (cram-utilities
                 cram-language
                 cram-reasoning
                 cram-execution-trace
                 cram-math
                 designators-ros
                 process-modules
                 roslisp
                 cram-roslisp-common
                 map-annotation
                 std_msgs-msg
                 geometry_msgs-msg
                 vision_srvs-srv
                 cogman_msgs-msg
                 actionlib
                 cljlo
                 alexandria
                 cl-json-pl-client
                 pr2_msgs-msg
                 cl-utils
                 visualization_msgs-msg
                 table-costmap
                 cram-plan-actionserver
                 jlo-navp-process-module
                 cljlo-utils
                 #+kipla-contrib-oro oro_ros-srv
                 #+kipla-contrib-oro yason
                 #+kipla-contrib-hri web_hri-srv)

    :components
    ((:module "src"
              :components
              ((:file "packages")
               (:file "config" :depends-on ("packages"))
               (:file "logging" :depends-on ("packages"))
               (:file "speech" :depends-on ("packages"))
               (:module "rete"
                        :depends-on ("packages" "config" "knowledge")
                        :components
                        ((:file "rete-utils")
                         (:file "occasions" :depends-on ("rete-utils"))))
               (:module "knowledge"
                        :depends-on ("packages" "config")
                        :components
                        ((:file "prolog-utils")
                         (:file "location-facts")
                         (:file "time")
                         (:file "tasks")
                         (:file "objects")))
               (:module "perception"
                        :depends-on ("packages" "config" "rete")
                        :components
                        ((:file "object-belief")
                         (:file "process-module"
                                :depends-on ("object-belief" "passive"))
                         (:module "cop"
                                  :depends-on ("object-belief" "process-module")
                                  :components
                                  ((:file "cop-designator")
                                   (:file "occasion-handlers")
                                   (:file "ros-connection" :depends-on ("cop-designator"))
                                   (:file "cop-search-handlers" :depends-on ("cop-designator"))))
                         (:module "passive"
                                  :depends-on ("object-belief")
                                  :components
                                  ((:file "knowrob-objects")))))
               (:module "manipulation"
                        :depends-on ("packages" "config")
                        :components
                        ((:file "ros-connection")
                         (:file "manipulation-designator")
                         (:file "process-module"
                                :depends-on ("ros-connection" "manipulation-designator"))))
               (:module "goals"
                        :depends-on ("packages"
                                     "config"
                                     "logging"
                                     "speech"
                                     "knowledge"
                                     "rete"
                                     "perception"
                                     "manipulation")
                        :components
                        ((:file "process-modules")
                         (:file "achieve")
                         (:file "achieve-loc")
                         (:file "achieve-ptu")
                         (:file "at-location")
                         (:file "perceive")
                         (:file "achieve-object-manipulation"))
                        :serial t)
               (:module "contrib"
                        :depends-on ()
                        :components
                        (#+kipla-contrib-oro
                         (:module "oro"
                                  :components
                                  ((:file "ros-connection")
                                   (:file "rete-productions")))
                         #+kipla-contrib-hri
                         (:module "hri_control"
                                  :depends-on ("oro")
                                  :components
                                  ((:file "hri")))))))
     (:module "sandbox"
              :depends-on ("src")
              :components
              ((:file "test-plans")
               (:file "drive-to-waypoints")
               (:file "launch")
               (:file "knowrob-missing-objects")))))
