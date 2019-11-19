;;;
;;; Copyright (c) 2019, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     
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

(in-package :plt)

(def-fact-group tasks (coe:holds)
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;; Transformation utils ;;;
  (<- (top-level-name ?top-level-name)
    (lisp-fun get-top-level-name ?top-level-name))

  (<- (top-level-path ?top-level-path)
    (top-level-name ?top-level-name)
    (coe:top-level-task ?top-level-name ?top-level-task)
    (coe:subtask ?top-level-task ?subtask)
    (coe:task-full-path ?subtask ?top-level-path))

  (<- (without-replacement ?task)
    (lisp-fun slot-value ?task cpl-impl::code-replacements ?replacement)
    (== ?replacement nil)
    (coe:subtask ?task ?sub)
    (lisp-fun slot-value ?sub cpl-impl::code-replacements ?sub-replacement)
    (== ?sub-replacement nil))

  ;; only used for evaluation
  (<- (task-specific-motion ?top-level-name ?subtree-path ?motion-type ?task ?designator)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (not (== nil ?subtree-path))
    (coe:top-level-task ?top-level-name ?top-level-task)
    (lisp-fun cpl:task-tree-node ?subtree-path ?top-level-task ?subtree-task)
    (lisp-fun tasks-of-type ?subtree-task ?motion-type ?desig-type ?all-matching-tasks)
    (member ?task ?all-matching-tasks)
    (coe:task-outcome ?task :succeeded)
    (coe:task-parameter ?task ?designator))
  ;;; Transformation utils ;;;
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;; Transformation rule predicates ;;;
  ;; general test if task tree contains at least 2 transporting tasks
  (<- (task-transporting ?task)
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (cpoe:location-distance-threshold ?dist-threshold)
    (cpoe:task-specific-action ?top-level-name ?path :transporting ?task ?_)
    (cpoe:task-specific-action ?top-level-name ?path :transporting ?other-task ?_)
    (without-replacement ?task)
    (without-replacement ?other-task)
    (not (== ?task ?other-task)))
  
  (<- (task-transporting-siblings (?first-path ?first-fetching-desig)
                                  (?second-path ?first-delivering-desig))
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (cpoe:task-transporting-action ?top-level-name ?path ?second-task ?_)
    (cpoe:task-transporting-action ?top-level-name ?path ?first-task ?_)
    (without-replacement ?first-task)
    (without-replacement ?second-task)
    (not (== ?first-task ?second-task))
    (cpoe:task-location-description-equal ?first-task ?second-task)
    (coe:task-full-path ?first-task ?first-path)
    (coe:task-full-path ?second-task ?second-transporting-path)
    (cpoe:task-fetching-action ?top-level-name ?first-path ?_ ?first-fetching-desig)
    (cpoe:task-delivering-action ?top-level-name ?first-path ?_ ?first-delivering-desig)
    (cpoe:task-delivering-action ?top-level-name ?second-transporting-path ?second-delivering-task ?_)
    (coe:task-full-path ?second-delivering-task ?second-path))

  (<- (task-transporting-from-container ?opening-path ?closing-path)
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (coe:top-level-task ?top-level-name ?root)
    (cpoe:subtasks-of-type ?root :transporting desig:action-designator ?transport-tasks)
    (member ?transport ?transport-tasks)
    (member ?compare ?transport-tasks)
    (coe:task-outcome ?transport :succeeded)
    (coe:task-outcome ?compare :succeeded)
    (not (== ?transport ?compare))
    (without-replacement ?transport)
    (without-replacement ?compare)
    (cpoe:task-location-description-equal ?compare ?transport)
    (cpoe:subtasks-of-type ?transport :accessing desig:action-designator ?openings)
    (cpoe:subtasks-of-type ?transport :sealing desig:action-designator ?closings)
    (member ?opening ?openings)
    (coe:task-outcome ?opening :succeeded)
    (member ?closing ?closings)
    (coe:task-outcome ?closing :succeeded)
    (coe:task-full-path ?opening ?opening-path)
    (coe:task-full-path ?closing ?closing-path))
  ;;; Transformation rule predicates ;;;
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  #+unused-predicates-for-tray-transformation
  ((<- (task-transporting-with-tray (?delivering-path))
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (location-distance-threshold ?dist-threshold)
    (cpoe:task-transporting-action ?top-level-name ?path ?last-task ?_)
    (cpoe:task-transporting-action ?top-level-name ?path ?before-last-task ?_)
    (not (== ?last-task ?before-last-task))
    (without-replacement ?last-task)
    (without-replacement ?before-last-task)
    (task-location-description-equal ?last-task ?before-last-task)
    (task-targets-nearby ?last-task ?before-last-task ?dist-threshold)
    (coe:task-full-path ?last-task ?last-path)
    (cpoe:task-delivering-action ?top-level-name ?last-path ?delivering-task ?_)
    (coe:task-full-path ?delivering-task ?delivering-path))

  (<- (task-transporting-with-tray-other-deliveries ?first-delivering-path (?other-path))
    (bound ?first-delivering-path)
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (location-distance-threshold ?dist-threshold)
    (coe:task-from-path ?first-delivering-path ?del-task)
    (cpoe:task-delivering-action ?top-level-name ?path ?other-del-task ?_)
    (not (== ?other-del-task ?del-task))
    (parent+ ?del-task ?task-transport)
    (task-type ?task-transport :transporting)
    (parent+ ?other-del-task ?other-task-transport)
    (task-type ?other-task-transport :transporting)
    (without-replacement ?task-transport) 
    (without-replacement ?other-task-transport)
    (task-location-description-equal ?task-transport ?other-task-transport)
    (task-targets-nearby ?task-transport ?other-task-transport ?dist-threshold)
    (task-full-path ?other-task-transport ?other-transp-path)
    (task-delivering-action ?top-level-name ?other-transp-path ?other-delivering-task ?_)
    (task-full-path ?other-delivering-task ?other-path))
  
  ;; Lazy parent backtracking. Should be able with subtask.
  (<- (parent+ ?task ?parent)
    (subtask ?parent ?task))
  
  (<- (parent+ ?task ?parent)
    (subtask ?tmp ?task)
    (parent+ ?tmp ?parent))
  
  (<- (task-locations-nearby ?task ?sibling ?threshold)
    (task-nearby ?task ?sibling ?threshold :location))

  (<- (task-type ?task ?action-type)
    (top-level-name ?top-level-name)
    (coe:task-full-path ?task ?task-path)
    (cpoe:task-specific-action ?top-level-name ?task-path ?action-type ?task ?_)))
  
  )
