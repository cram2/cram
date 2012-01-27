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
  (roslisp:ros-info (perceive plan-lib) "PERCEIVE-OBJECT ~a `~a'" indicator designator))

(declare-goal perceive-state (occasion)
  "Tries to prove that `occasion' holds. Returns a list of bindings
  that fulfill `occasion' or NIL if the occasion cannot be proven."
  (roslisp:ros-info (perceive plan-lib) "PERCEIVE-STATE `~a'" occasion))

(declare-goal examine (object-designator properties)
  "Examines `object-designator' using `property'. Examining means that
  a new, extended designator is returned. Examination tries to find
  the values of `properties' and add them to `object-designator'"
  (roslisp:ros-info (perceive plan-lib) "EXAMINE `~a' ~a"
                    object-designator properties))

(declare-goal achieve (occasion)
  "Achieves `occasion' if it is not yet achieved."
  (when (holds-occasion occasion)
    (ros-info (achieve plan-lib) "Occasion `~a' already achieved." occasion)
    (return nil)))
