;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :pr2-pms)

(def-fact-group pr2-navigation-actions (action-desig)
    (<- (action-desig ?designator ?goal-location)
      (desig-prop ?designator (:type :navigation))
      (desig-prop ?designator (:goal ?goal-location))))

(def-fact-group pr2-ptu-actions (action-desig)
  (<- (action-desig ?desig (point ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :see))
    (desig-prop ?desig (:pose ?pose))
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (point ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :see))
    (desig-prop ?desig (:location ?loc))
    (loc-desig-location ?loc ?pose)
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (point ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :see))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (desig-location-prop ?obj ?pose)
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (follow ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :follow))
    (desig-prop ?desig (:pose ?pose))
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (follow ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :follow))
    (desig-prop ?desig (:location ?loc))
    (loc-desig-location ?loc ?pose)
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (follow ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :follow))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (obj-desig-location ?obj ?pose)
    (lisp-fun make-action-goal ?pose ?act)))


(def-fact-group pr2-manipulation-actions (action-desig)
  ;; On the PR2 we don't need an open pose
  (<- (action-desig ?desig (noop ?desig))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:pose :open)))

  (<- (action-desig ?desig (park-object ?obj ?grasp-assignments))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :park))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (current-designator ?obj ?current-obj)
    (object->grasp-assignments ?current-obj ?grasp-assignments))

  (<- (action-desig ?desig (park-arms ?arms))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :park))
    (free-arms ?arms))

  (<- (action-desig ?desig (lift nil nil ?distance))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :lift))
    (desig-prop ?desig (:obj nil))
    (-> (desig-prop ?desig (:distance ?distance))
        (true)
        (== ?distance 0.1)))

  (<- (action-desig ?desig (handover ?object ?grasp-assignments))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :handover))
    (desig-prop ?desig (:obj ?obj))
    (current-designator ?obj ?object)
    (object->grasp-assignments ?object ?grasp-assignments))

  (<- (action-desig ?desig (lift ?current-obj ?grasp-assignments ?distance))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :lift))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (not (equal ?obj nil))
    (current-designator ?obj ?current-obj)
    (object->grasp-assignments ?current-obj ?grasp-assignments)
    (-> (desig-prop ?desig (:distance ?distance))
        (true)
        (== ?distance 0.02)))

  (<- (action-desig ?desig (park ?arms ?obj ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :carry))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (current-designator ?obj ?current-obj)
    (holding-arms ?current-obj ?arms)
    (obstacles ?desig ?obstacles))

  (<- (action-desig ?desig (grasp ?desig ?current-obj))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :grasp))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (newest-effective-designator ?obj ?current-obj))

  (<- (action-desig ?desig (shove-into ?current-obj ?target-pose))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :shove-into))
    (desig-prop ?desig (:obj ?obj))
    (current-designator ?obj ?current-obj)
    (desig-prop ?desig (:pose ?target-pose)))

  (<- (action-desig ?desig (pull-open ?semantic-handle))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :pull-open))
    (desig-prop ?desig (:handle ?semantic-handle)))

  (<- (action-desig ?desig (open-container ?arm ?loc ?degree))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :open))
    (desig-prop ?desig (:location ?loc))
    (desig-prop ?desig (:degree ?degree))
    (free-arm ?arm))

  (<- (action-desig ?desig (put-down ?current-obj ?loc ?grasp-assignments))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :put-down))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (desig-prop ?desig (:at ?loc))
    (current-designator ?obj ?current-obj)
    (object->grasp-assignments ?current-obj ?grasp-assignments))

  (<- (action-desig ?desig (pull ?current-obj ?arms ?direction ?distance ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :pull))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (desig-prop ?desig (:distance ?distance))
    (desig-prop ?desig (:direction ?direction))
    (current-designator ?obj ?current-obj)
    (fail) ;; This predicate needs to be refactored
    (grasped-object-part ?obj ?grasped)
    (holding-arms ?current-obj ?arms)
    (obstacles ?desig ?obstacles))

  (<- (action-desig ?desig (push ?current-obj ?arms ?direction ?distance ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :push))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (desig-prop ?desig (:distance ?distance))
    (desig-prop ?desig (:direction ?direction))
    (current-designator ?obj ?current-obj)
    (holding-arms ?current-obj ?arms)
    (obstacles ?desig ?obstacles)))


(def-fact-group perception-designators (action-desig-projection)
  (<- (action-desig-projection ?designator ?object-designator)
    (desig-prop ?designator (:to :perceive))
    (or (desig-prop ?designator (:obj ?object-designator))
        (desig-prop ?designator (:object ?object-designator)))))


(def-fact-group manipulation-designators (action-desig-projection)
  (<- (action-desig-projection
       ?desig (execute-container-opened ?desig ?obj ?distance))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :open))
    (desig-prop ?desig (:handle ?obj))
    (desig-prop ?obj (:name ?handle-name))
    (-> (desig-prop ?desig (:side ?side))
        (== ?sides (?side))
        (true))
    (available-arms ?obj ?sides)
    (lisp-fun get-opening-distance ?handle-name ?distance))

  (<- (action-desig-projection ?desig (execute-container-closed ?desig ?obj))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :close))
    (desig-prop ?desig (:handle ?obj))
    (-> (desig-prop ?desig (:side ?side))
        (== ?sides (?side))
        (true))
    (available-arms ?obj ?sides))

  (<- (action-desig-projection ?desig (execute-lift ?desig))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :lift))
    (or (desig-prop ?desig (:obj ?_))
        (desig-prop ?desig (:object ?_))))

  (<- (action-desig-projection ?desig (execute-park ?sides ?objects-in-hand))
    (trajectory-desig? ?desig)
    (or
     (desig-prop ?desig (:to :park))
     (desig-prop ?desig (:to :carry)))
    (robot ?robot)
    (-> (desig-prop ?desig (:side ?side))
        (== ?sides ?side)
        (findall ?side (arm ?robot ?side) ?sides))
    (findall (?side ?obj ?link)
             (and
              (cram-plan-occasions-events:object-in-hand ?obj ?side) 
              (end-effector-link ?robot ?side ?link))
             ?objects-in-hand))

  (<- (action-desig-projection ?desig (execute-grasp ?desig ?obj))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :grasp))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj))))

  (<- (action-desig-projection ?desig (execute-put-down ?desig ?obj))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :put-down))
    (or (desig-prop ?desig (:obj ?obj))
        (desig-prop ?desig (:object ?obj)))
    (desig-prop ?desig (:at ?_)))

  (<- (action-desig-projection ?desig (execute-pour ?desig ?container))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (:to :pour))
    (desig-prop ?desig (:container ?container))
    (desig-prop ?desig (:at ?_))))

