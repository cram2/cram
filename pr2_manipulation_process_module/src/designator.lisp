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

  (<- (holding-grippers ?desig ?grippers)
    (gripped-obj-desig? ?desig)
    (desig-prop ?desig (at ?obj-loc))
    (findall ?g (desig-prop ?obj-loc (gripper ?g)) 
             ?grippers))

  (<- (handled-obj-desig? ?designator)
    (obj-desig? ?designator)
    (desig-prop ?designator (handle ?_)))

  (<- (gripped-obj-desig? ?designator)
    (obj-desig? ?designator)
    (desig-prop ?designator (at ?obj-loc))
    (loc-desig? ?obj-loc)
    (desig-prop ?obj-loc (in gripper)))
    

  (<- (best-grasp ?obj ?handles ?obstacles ?grasps ?arms)
    (lisp-fun calc-best-grasps-and-arms ?obj ?handles ?obstacles (?grasps ?arms)))

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

  (<- (action-desig ?desig (park nil :right))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (pose parked)))

  (<- (action-desig ?desig (lift ?grippers ?distance))
    ;; NOTE(Georg): we're blurring the distinction
    ;; between arms and grippers here. feels fishy...
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to lift))
    (desig-prop ?desig (obj ?obj))
    (current-designator ?obj ?current-obj)
    (holding-grippers ?current-obj ?grippers)
    (-> (desig-prop ?desig (distance ?distance))
        (true)
        (== ?distance 0.10)))

  (<- (action-desig ?desig (lift (:right) ?distance))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to lift))
    (-> (desig-prop ?desig (distance ?distance))
        (true)
        (== ?distance 0.10)))

  (<- (action-desig ?desig (park ?obj :right ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to carry))
    (desig-prop ?desig (obj ?obj))
    (obstacles ?desig ?obstacles))

  ;; rule added by Georg:
  ;; right now, it is intended to be limited to grasping of
  ;; handled objects, i.e. the ones produced by the gazebo
  ;; perception process module.
  ;; later, however, it could be used as the general rule for
  ;; all grasping because the predicate 'best-grasp' can be
  ;; used as a hook for grasp planning or any other manipulation
  ;; reasoning process that chooses the correct arm/grasp setup
  (<- (action-desig ?desig (grasp-slave ?obj ?grasps ?arms ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (handled-obj-desig? ?obj)
    (handles ?obj ?handles)
    (obstacles ?desig ?obstacles)
    (best-grasp ?obj ?handles ?obstacles ?grasps ?arms))

  (<- (action-desig ?desig (grasp ?object-type ?obj :right ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type ?object-type))
    (obstacles ?desig ?obstacles))

  (<- (action-desig ?desig (put-down ?obj ?loc :right ?obstacles))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (at ?loc))
    (obstacles ?desig ?obstacles)))

(def-fact-group manipulation-process-module (matching-process-module available-process-module)

  (<- (matching-process-module ?designator pr2-manipulation-process-module)
    (trajectory-desig? ?designator)
    (or (desig-prop ?designator (to grasp))
        (desig-prop ?designator (to put-down))
        (desig-prop ?designator (to open))
        (desig-prop ?designator (to close))
        (desig-prop ?designator (pose parked))
        (desig-prop ?designator (pose open))        
        (desig-prop ?designator (to lift))
        (desig-prop ?designator (to carry))))

  (<- (available-process-module pr2-manipulation-process-module)
    (symbol-value cram-projection:*projection-environment* nil)))
