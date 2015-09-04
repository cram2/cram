;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-robot-interfaces)

(def-fact-group trajectories (trajectory-point)
  ;; Yields points that are part of a manipulation
  ;; trajectory. ?designator is bound to an action designator. The
  ;; intended use of this predicate is for reasoning about
  ;; reachability. Since it is not always possible to generate the
  ;; exactly same points that are used for the manipulation actions
  ;; here, e.g. when grasp planning has to be done, ?point is not
  ;; necessarily an exact trajectory point but it should at least be
  ;; close to the actual point.
  (<- (trajectory-point ?designator ?point ?side)
    (fail))

  ;; The second version of the TRAJECTORY-POINT predicate takes an
  ;; additional binding, the reference pose of the robot, i.e. the
  ;; pose at which the robot will execute the trajectory. That way, it
  ;; is possible to generate better trajectory points by e.g. taking
  ;; into account the object size and the arm. For instance, to
  ;; calculate the trajectory point for the left arm for grasping a
  ;; pot, it is better to move the trajectory point to the side.
  (<- (trajectory-point ?designator ?robot-reference-pose ?point ?side)
    (fail)))
