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

(in-package :cl-user)

(desig-props:def-desig-package cram-plan-library
  (:documentation "Library of plans for pick-and-place tasks.")
  (:use #:cpl
        #:cram-designators
        #:cram-utilities
        #:cram-process-modules
        #:roslisp
        #:cram-roslisp-common
        #:cram-plan-knowledge
        #:cram-plan-failures
        #:alexandria)
  (:import-from #:cram-reasoning #:<- #:def-fact-group
                #:lisp-fun #:lisp-pred)
  (:nicknames :plan-lib)
  (:export #:achieve
           #:perform
           #:perform-on-process-module
           #:loc
           #:object-in-hand
           #:object-placed-at
           #:container-opened
           #:perceive-object-in-gripper
           #:arms-parked
           #:arms-at
           #:looking-at
           #:at-location
           #:object-detected
           #:perceive-object
           #:the #:all #:a #:currently-visible
           #:perceive-state
           #:examine
           #:obstacles-found
           #:robot
           #:maybe-run-process-modules
           #:run-process-modules
           #:start-process-modules
           #:stop-process-modules
           ;; rete and occasions
           #:object-picked-up
           #:object-in-hand-failure
           #:object-not-found-failure
           ;; designator generation
           #:with-designators
           #:a
           #:an
           ;; prolog facts
           #:currently-visible-objects)
  (:desig-properties #:to #:see #:obj #:of #:reach #:type #:trajectory
                     #:pose #:open #:side #:grasp #:lift #:carry :reach
                     #:location #:at #:parked #:pose #:close #:gripper
                     #:follow #:pick-up #:put-down #:height #:orientation
                     #:in #:perceive #:obstacle #:cluster #:execute
                     #:action #:goal #:navigation #:park #:on #:angle
                     #:examine #:properties))
