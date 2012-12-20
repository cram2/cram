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

(in-package :pr2-manip-pm)

(defun make-message (type-str slots)
  (apply #'roslisp::make-message-fn type-str slots))

(def-fact-group pr2-manipulation-designators (action-desig)

  (<- (ros-message ?type ?slots ?msg)
    (lisp-fun make-message ?type ?slots ?msg))

  (<- (obstacles ?desig ?obstacles)
    (findall ?o (desig-prop ?desig (obstacle ?o))
             ?obstacles))

  (<- (handles ?desig ?handles)
    (findall ?h (desig-prop ?desig (handle ?h))
             ?handles))

  (<- (gripper-arms-in-desig ?desig ?arms)
    (current-designator ?desig ?current-desig)
    (gripped-obj-desig? ?current-desig)
    (desig-prop ?current-desig (at ?obj-loc))
    (desig-prop ?obj-loc (gripper ?_))
    (findall ?g (desig-prop ?obj-loc (gripper ?g))
             ?arms))

  (<- (gripper-arms-in-belief ?desig ?arms)
    (current-designator ?desig ?current-desig)
    (findall ?g (object-in-hand ?current-desig ?g)
             ?arms))

  (<- (holding-arms ?desig ?arms)
    (current-designator ?desig ?current-desig)
    (gripper-arms-in-belief ?current-desig ?arms))

  (<- (handled-obj-desig? ?designator)
    (obj-desig? ?designator)
    (desig-prop ?designator (handle ?_)))

  (<- (gripped-obj-desig? ?designator)
    (obj-desig? ?designator)
    (desig-prop ?designator (at ?obj-loc))
    (loc-desig? ?obj-loc)
    (desig-prop ?obj-loc (in gripper)))

  (<- (reachable-handles ?obj-desig ?arms ?handle-evaluations)
    (handles ?obj-desig ?handles)
    (symbol-value *handle-pregrasp-offset-pose* ?pregrasp-offset)
    (symbol-value *handle-grasp-offset-pose* ?grasp-offset)
    (lisp-fun handle-distances ?obj-desig ?arms
              :pregrasp-offset ?pregrasp-offset
              :grasp-offset ?grasp-offset
              :constraint-aware t
              ?handle-evaluations))

  (<- (best-grasp ?obj ?available-arms ?obstacles
                  ?arm-handle-assignments)
    (desig-prop ?obj (type ?object-type))
    (cram-manipulation-knowledge:object-type-grasp ?object-type ?_ ?sides)
    (reachable-handles ?obj ?sides ?reachable-handles)
    (length ?sides ?min-handles)
    (optimal-arm-handle-assignments ?obj ?reachable-handles ?min-handles
                                    ?sides ?arm-handle-assignments))

  (<- (optimal-arm-handle-assignments ?obj ?reachable-handles ?min-handles
                                      ?available-arms ?optimal-assignment)
    ;; NOTE(winkler): We're not using the reachable handles here
    ;; anymore. Instead, the available handles on the object are tried
    ;; and evaluated in optimal-handle-assignment. This has to be
    ;; refactored. Until then, the parameter stays here for signature
    ;; purposes.
    (lisp-fun optimal-handle-assignment ?obj ?available-arms
              nil ?min-handles ?optimal-assignment)
    (length ?optimal-assignment ?assignment-count)
    (> ?assignment-count 0))

  (<- (action-desig ?desig (container-opened ?handle :right))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to open))
    (desig-prop ?desig (handle ?handle)))

  (<- (action-desig ?desig (container-closed ?handle :right))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to close))
    (desig-prop ?desig (handle ?handle)))

  ;; On the PR2 we don't need an open pose
  (<- (action-desig ?desig (noop ?desig))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (pose open)))

  (<- (action-desig ?desig (park nil ?grippers))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to park))
    (desig-prop ?desig (obj ?obj))
    (gripper-arms-in-desig ?obj ?grippers))

  (<- (action-desig ?desig (park nil nil))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to park)))

  (<- (action-desig ?desig (lift ?arms ?distance))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to lift))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (holding-arms ?current-obj ?arms)
    (-> (desig-prop ?desig (distance ?distance))
        (true)
        (== ?distance 0.10)))

  (<- (action-desig ?desig (park ?obj ?arms ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to carry))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (holding-arms ?current-obj ?arms)
    (obstacles ?desig ?obstacles))

  ;; rule added by Georg:
  ;; right now, it is intended to be limited to grasping of
  ;; handled objects, i.e. the ones produced by the gazebo
  ;; perception process module.
  ;; later, however, it could be used as the general rule for
  ;; all grasping because the predicate 'best-grasp' can be
  ;; used as a hook for grasp planning or any other manipulation
  ;; reasoning process that chooses the correct arm/grasp setup
  (<- (action-desig ?desig (grasp-handles ?current-obj
                                          ?arm-handle-assignments
                                          ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (newest-effective-designator ?obj ?current-obj)
    (handled-obj-desig? ?current-obj)
    (handles ?current-obj ?obj-handles)
    (obstacles ?desig ?obstacles)
    (setof ?arm (cram-manipulation-knowledge:available-arms ?current-obj (?arm))
           ?arms)
    (best-grasp ?current-obj ?arms ?obstacles ?arm-handle-assignments))

  (<- (action-desig ?desig (put-down ?current-obj ?loc ?arms ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (holding-arms ?current-obj ?arms)
    (desig-prop ?desig (at ?loc))
    (obstacles ?desig ?obstacles)))

(def-fact-group manipulation-process-module (matching-process-module available-process-module)

  (<- (matching-process-module ?designator pr2-manipulation-process-module)
    (trajectory-desig? ?designator)
    (or (desig-prop ?designator (to grasp))
        (desig-prop ?designator (to put-down))
        (desig-prop ?designator (to open))
        (desig-prop ?designator (to close))
        (desig-prop ?designator (to park))
        (desig-prop ?designator (pose open))        
        (desig-prop ?designator (to lift))
        (desig-prop ?designator (to carry))))

  (<- (available-process-module pr2-manipulation-process-module)
    (symbol-value cram-projection:*projection-environment* nil)))
