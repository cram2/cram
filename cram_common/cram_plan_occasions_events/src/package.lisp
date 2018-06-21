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

(in-package :cl-user)

(defpackage cram-plan-occasions-events
  (:nicknames :cpoe)
  (:use #:common-lisp
        #:cram-occasions-events
        #:cram-prolog)
  (:export #:object-perceived-event
           #:robot-state-changed
           #:object-connection-event
           #:object-articulation-event
           #:object-attached #:object-detached
           #:object-removed-event
           #:object-updated-event #:event-object-name
           #:event-object-designator #:object-designator
           #:perception-source #:object #:event-object
           #:link #:event-link #:side #:event-side
           #:opening-distance

           #:environment-manipulation-event
           #:environment-event-joint-name
           #:environment-event-arm
           #:environment-event-object
           #:environment-event-distance
           #:container-handle-grasping-event
           #:container-opening-event
           #:container-closing-event

           ;; #:object-gripped
           ;; #:event-arm #:event-object #:event-grasp
           ;; #:object-released

           ;; object connection event
           #:event-arm #:event-object-name

           ;; occasion-declarations
           ;; Symbols used in plans and thus the execution trace.
           #:object-in-hand
           #:object-placed-at
           #:object-picked
           #:object-put
           #:loc
           #:looking-at
           #:arms-parked))
