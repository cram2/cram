;;;
;;; Copyright (c) 2015, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-plan-occasions-events)

(def-fact-group tasks (coe:holds)

  (<- (perform-task-of-top-level ?top-level-name ?task-node)
    (bound ?top-level-name)
    (coe:top-level-task ?top-level-name ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node (?path . ?_))
    (equal ?path (cpl:goal (exe:perform ?_) . ?_)))

  (<- (perform-task ?top-level-name ?subtree-path ?task-node)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (coe:task ?top-level-name ?subtree-path ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node (?path . ?_))
    (equal ?path (cpl:goal (exe:perform ?_) . ?_)))
  
  ;; action tasks
  (<- (task-specific-action ?top-level-name ?subtree-path ?action-type ?task ?designator)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (perform-task ?top-level-name ?subtree-path ?task)
    (coe:task-parameter ?task ?designator)
    (lisp-type ?designator desig:action-designator)
    (desig:desig-prop ?designator (:type ?action-type)))

  (<- (task-navigating-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :navigating ?task ?designator))

  (<- (task-fetching-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :fetching ?task ?designator))

  (<- (task-picking-up-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :picking-up ?task ?designator))

  (<- (task-delivering-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :delivering ?task ?designator))

  (<- (task-transporting-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :transporting ?task ?designator))

  (<- (task-next-action-sibling ?top-level-name ?subtree-path ?task ?action-type ?next-task)
    (bound ?top-level-name)
    (bound ?task)
    (coe:task-full-path ?task ?task-path)
    (bagof (?sibling-task ?sibling-path)
           (and (coe:subtask ?parent ?task)
                (coe:subtask ?parent ?sibling-task) 
                (coe:task-full-path ?sibling-task ?sibling-path))
           ?siblings)
    (lisp-fun cut:force-ll ?siblings ?sibling-last-to-first)
    (lisp-fun reverse ?sibling-last-to-first ?siblings-first-to-last)
    (lisp-fun siblings-until-task ?task-path ?siblings-first-to-last ?next-tasks)
    (member ?next-task ?next-tasks)
    (task-specific-action ?top-level-name ?subtree-path ?action-type ?next-task ?_))
  
  (<- (task-previous-action-sibling ?top-level-name ?subtree-path ?task ?action-type ?previous-task)
    (bound ?top-level-name)
    (bound ?task)
    (coe:task-full-path ?task ?task-path)
    (bagof (?sibling-task ?sibling-path)
           (and (coe:subtask ?parent ?task)
                (coe:subtask ?parent ?sibling-task) 
                (coe:task-full-path ?sibling-task ?sibling-path))
           ?siblings)
    (lisp-fun cut:force-ll ?siblings ?siblings-task-path-pairs)
    (lisp-fun siblings-until-task ?task-path ?siblings-task-path-pairs ?previous-tasks)
    (member ?previous-task ?previous-tasks)
    (task-specific-action ?top-level-name ?subtree-path ?action-type ?previous-task ?_))

  (<- (location-distance-threshold ?threshold)
    (lisp-fun get-location-distance-threshold ?threshold))

  (<- (subtasks-of-type ?root-task ?type ?desig-class ?tasks)
    (bound ?root-task)
    (bound ?type)
    (member ?desig-class (desig:action-designator desig:motion-designator))
    (lisp-fun subtasks-of-type ?root-task ?type ?desig-class ?tasks))
  
  ;; (<- (task-nearby ?task ?sibling ?threshold ?location-key)
  ;;   (coe:task-parameter ?task ?desig)
  ;;   (coe:task-parameter ?sibling ?sibling-desig)
  ;;   (desig:desig-prop ?desig (?location-key ?loc))
  ;;   (desig:desig-prop ?sibling-desig (?location-key ?sibling-loc))
  ;;   (lisp-fun location-desig-dist ?loc ?sibling-loc ?dist)
  ;;   (< ?dist ?threshold))
  
  ;; (<- (task-targets-nearby ?task ?sibling ?threshold)
  ;;   (task-nearby ?task ?sibling ?threshold :target))
  )
