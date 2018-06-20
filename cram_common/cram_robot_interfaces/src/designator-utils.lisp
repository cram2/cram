;;;
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

(in-package :cram-robot-interfaces)

(defgeneric compute-iks (pose-stamped &key link-name arm robot-state seed-state
                                        pose-stamped-frame tcp-in-ee-pose)
  (:documentation "Computes an inverse kinematics solution (if possible)
to position the link `link-name' in the goal pose `pose-stamped'
specified in the frame of the pose or `pose-stamped-frame' when given,
where only the links in `arm' can be moved to achieve the solution.
When given, `robot-state' is the initial pose of robot joints,
otherwise TF data is used. `seed-state' is the seed for the solver.
 When `tcp-in-ee-pose' (i.e. pose of tool center point
in end effector frame) is given the goal pose is automatically transformed
to take it into account.
Returns a ROS JointState message with solution states of the joints in the `arm'."))

(def-fact-group reachability-designators ()

  (<- (reachability-designator ?designator)
    (desig-prop ?designator (:reachable-for ?robot))
    ;; (robot ?robot)
    )

  (<- (visibility-designator ?designator)
    (desig-prop ?designator (:visible-for ?robot))
    ;; (robot ?robot)
    )

  (<- (designator-reach-pose ?designator ?pose ?side)
    (reachability-designator ?designator)
    (desig-prop ?designator (:pose ?pose))
    (once
     (-> (desig-prop ?designator (:arm ?side))
         (true)
         (and (robot ?robot)
              (arm ?robot ?side)))))

  (<- (designator-reach-pose ?designator ?point ?side)
    (reachability-designator ?designator)
    (or (desig-prop ?designator (:object ?object))
        (desig-prop ?designator (:obj ?object)))
    (desig-location-prop ?object ?pose)
    (once
     (-> (desig-prop ?designator (:arm ?side))
         (true)
         (and (robot ?robot)
              (arm ?robot ?side))))
    (lisp-fun cl-transforms:origin ?pose ?point))

  (<- (designator-reach-pose ?designator ?pose ?side)
    (reachability-designator ?designator)
    (desig-prop ?designator (:location ?location))
    (once
     (-> (desig-prop ?designator (:arm ?side))
         (true)
         (and (robot ?robot)
              (arm ?robot ?side))))
    (desig-location-prop ?designator ?pose))

  ;; (<- (designator-reach-pose ?designator ?robot-pose ?pose ?side)
  ;;   (reachability-designator ?designator)
  ;;   (desig-prop ?designator (:to :execute))
  ;;   (desig-prop ?designator (:action ?action))
  ;;   (trajectory-point ?action ?robot-pose ?pose ?side))

  ;; (<- (designator-reach-pose ?designator ?pose ?side)
  ;;   (reachability-designator ?designator)
  ;;   (desig-prop ?designator (:to :execute))
  ;;   (desig-prop ?designator (:action ?action))
  ;;   (trajectory-point ?action ?pose ?side))
  )


(defun reachability-designator-p (designator)
  (prolog `(reachability-designator ,designator)))

(defun visibility-designator-p (designator)
  (prolog `(visibility-designator ,designator)))


(def-fact-group manipulation-designators ()
  (<- (trajectory-desig? ?desig)
    (lisp-pred typep ?desig action-designator)
    (desig-prop ?desig (:type :trajectory)))

  (<- (constraints-desig? ?desig)
      (lisp-pred typep ?desig action-designator)
      (desig-prop ?desig (:type :constraints))))
