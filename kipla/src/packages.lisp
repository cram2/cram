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

(defpackage :kipla-utils
    (:documentation "Utilities")
  (:use #:common-lisp)
  (:export #:pose->jlo #:jlo->pose
           #:table-cluster->jlo))

(defpackage :kipla-reasoning
  (:documentation "All the prolog and reasoning stuff used in kipla.")
  (:use #:common-lisp
        #:cram-utilities
        #:cram-language-implementation ;; we don't want the fluent operators
        #:cram-designators #:designators-ros
        #:cram-reasoning
        #:cram-execution-trace
        #:kipla-utils
        #:cram-roslisp-common
        #:map-annotation
        #:location-costmap
        #:table-costmap)
  (:shadowing-import-from #:cram-reasoning #:fail)
  (:shadowing-import-from #:cram-language #:name)
  (:import-from #:alexandria
                #:curry #:rcurry #:compose
                #:format-symbol)
  (:export
   ;; belief-state
   #:update-belief #:clear-belief #:*believed-occasions*
   #:assert-occasion #:retract-occasion #:holds
   ;; object-belief
   #:perceived-object #:queried-object #:object-properties
   #:object-pose #:object-jlo
   #:perceived-object-probability #:object-desig
   #:object-timestamp #:*perceived-objects*
   #:clear-object-belief #:update-perceived-object
   #:perceived-objects-equal? #:compatible-properties
   #:designator->production #:assert-perceived-object
   #:retract-perceived-object #:assert-desig-binding
   #:retract-desig-binding #:desig-current-perceived-object
   #:matching-object #:merge-desig-descriptions
   #:desig-bound
   #:make-new-desig-description
   ;; designators
   #:manip-desig
   #:action #:object #:location
   #:mug #:icetea #:cluster #:jug #:placemat #:coke
   #:object
   #:color #:black #:red #:white #:green #:blue
   #:at #:matches #:show #:for
   #:jlo-list #:jlo #:trajectory
   #:grasp #:navigate #:pose #:parked #:open #:lift #:carry #:put-down
   #:obj #:gripper #:close #:resolve-object-desig #:orientation
   ;; trajectory-actions
   #:trajectory-action #:side #:trajectory-type #:stored-pose-type
   #:object-type #:hand-primitive #:end-effector-pose
   #:obstacles #:grasp-distance #:supporting-plane
   #:copy-trajectory-action
   ;; cop-designators
   #:cop-desig-query-info #:cop-desig-query-info-object-classes
   #:cop-desig-query-info-object-ids #:cop-desig-query-info-poses
   #:cop-desig-query-info-matches
   #:make-cop-desig-query-info #:copy-cop-desig-query-info
   #:cop-desig-location-info #:cop-desig-location-info-poses
   #:cop-desig-info #:cop-desig-info-designator #:cop-desig-info-query
   #:cop-desig-info-location #:cop-ignore-property-p
   ;; prolog
   #:fluent
   #:fluent-value
   #:task-goal
   #:task
   #:task-status-fluent
   #:task-goal
   #:task-outcome
   #:task-result
   #:task-error
   #:task-location-context
   #:error-type
   #:holds
   #:task-status
   #:duration-includes
   #:throughout
   #:during
   ;; Symbols used in plans and thus the execution trace. Export here so we
   ;; can neatly do the reasoning in package kipla-reasoning without writing
   ;; "KIPLA::" all over the place.
   #:achieve
   #:object-in-hand
   #:object-placed-at
   #:object-in-hand-failure
   #:at-location
   #:loc
   #:robot
   #:perceive
   #:perceive-all
   #:?obj
   #:?side
   #:owl-type
   #:cowsmilk-product
   #:shape
   #:arms-at
   #:?traj
   #:looking-at
   #:?lo
   #:arm-parked
   #:?loc
   ;; Locations
   #:location-costmap #:width #:height #:origin-x #:origin-y #:resolution
   #:get-map-value #:get-cost-map #:generate-point #:gen-costmap-sample
   #:occupancy-grid-metadata #:occupancy-grid #:grid #:make-occupancy-grid
   #:copy-occupancy-grid #:invert-occupancy-grid #:set-grid-cell
   #:clear-grid-cell #:get-grid-cell
   #:grid-cells-msg->occupancy-grid
   #:grid-cells-msg->height-map
   #:height-map #:height-map-lookup #:height-map-set
   ;; other
   #+kipla-contrib-oro #:oro-call
   #+kipla-contrib-oro #:oro-assert
   #+kipla-contrib-oro #:oro-retract
   ))

(defpackage :kipla
  (:documentation "Kitchen Planner and coordinator")
  (:use #:roslisp
        #:cram-utilities
        #:cram-language ;; cram-language reexports all of package :common-lisp
        #:cram-designators
        #:cram-process-modules
        #:cram-roslisp-common
        #:kipla-reasoning
        #:kipla-utils
        #:designators-ros
        #:location-costmap
        #:table-costmap)
  (:export #:log-msg #:run-demo-counter-to-table
           #:run-demo-table-to-counter #:look-at
           #:drive-to-waypoints-main
           #:*tf*
           #:*map-fl*
           #:*table-grid-cells-fl*
           #:*table-height-map-fl*
           #:get-closest-annotation
           #:get-annotated-point)
  (:import-from #:alexandria
                #:with-gensyms #:curry #:rcurry)
  (:import-from #:cram-reasoning
                #:prolog)
  (:shadowing-import-from #:table-costmap
                          #:name))
