;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :plan-lib)

(declare-goal perceive-object (indicator designator)
  "Tries to find an object described by `designator'. `indicator' can
  be one of the following symbols:

 THE: Throws an error if more than one or no objects can be
       found. Otherwise returns an equated designator.
 
 A: Returns the equated designator that matches the designator best,
     ignores additional objects that possibly match. Throws an error
     if no objects could be found.

 ALL: Returns all unequated designators that match `designator'.

 CURRENTLY-VISIBLE: Returns all objects that match `designator' and
                    can be detected without moving the ptu or the
                    robot."
  (roslisp:ros-info (perceive-object plan-lib) "PERCEIVE-OBJECT ~a `~a'"
  indicator designator))

;; (declare-goal object-picked (object-designator)
;;   "Tries to pick up the object `object-designator' from the current
;;   position."
;;   (roslisp:ros-info (picked plan-lib) "OBJECT-PICKED `~a'"
;;   object-designator))

;; (declare-goal object-put (object-designator location-designator)
;;   "Tries to put down the object `object-designator' to the location
;; `location-designator'."
;;   (declare (ignore location-designator))
;;   (roslisp:ros-info (put plan-lib) "OBJECT-PUT `~a'"
;;   object-designator))

;; (declare-goal object-flipped (object-designator
;;                               tool-object-designator-1
;;                               tool-object-designator-2
;;                               flipping-parameters)
;;   "Tries to flip an object `object-designator' using the instrumental
;; tools `tool-object-designator-1' and `tool-object-designator-2'."
;;   (declare (ignore tool-object-designator-1 tool-object-designator-2))
;;   (roslisp:ros-info (flip plan-lib) "OBJECT-FLIPPED `~a'" object-designator))

(declare-goal perceive-state (occasion)
  "Tries to prove that `occasion' holds. Returns a list of bindings
that fulfill `occasion' or NIL if the occasion cannot be proven."
  (roslisp:ros-info (perceive-state plan-lib) "PERCEIVE-STATE `~a'" occasion))

(declare-goal examine (object-designator properties)
  "Examines `object-designator' using `property'. Examining means that
a new, extended designator is returned. Examination tries to find the
values of `properties' and add them to `object-designator'"
  (roslisp:ros-info (examine plan-lib) "EXAMINE `~a' ~a"
                    object-designator properties))

(declare-goal achieve (occasion)
  "Achieves `occasion' if it is not yet achieved."
  (when (holds occasion)
    (ros-info (achieve plan-lib) "Occasion `~a' already achieved." occasion)
    (return nil)))

(declare-goal perform (action-designator)
  "Performs the action defined by `action-designator'. This goal
  infers which process module to use and calls pm-execute in it."
  (declare (ignore action-designator)))

(declare-goal perform-on-process-module (module action-designator)
  "Sends `action-designator' to the process module name `module'."
  (declare (ignore module action-designator)))

(declare-goal monitor-action (action-designator)
  "Blocks and monitors the execution of `action-designators' using the
  method MONITOR-PROCESS-MODULE."
  (declare (ignore action-designator)))

;; (declare-goal table-set (table-name situation-description)
;;   "Sets a table referred to by the name `table-name', according to a
;; situation described by `situation-description'."
;;   (declare (ignore table-name situation-description)))

;; (declare-goal drawer-opened (?semantic-name)
;;   (declare (ignore ?semantic-name)))
