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

(defgeneric extract-task-error (err)
  (:method ((err cpl:plan-failure))
    err)
  (:method ((err cpl:common-lisp-error-envelope))
    (cpl:envelop-error err)))

(def-fact-group tasks (coe:holds)

  ;; top-level
  (<- (top-level-task ?top-level-name ?top-level-task-node)
    (bound ?top-level-name)
    (lisp-fun cpl:get-top-level-task-tree ?top-level-name ?top-level-task-node))

  (<- (top-level-episode-knowledge ?top-level-name ?top-level-episode)
    (bound ?top-level-name)
    (lisp-fun cet:get-top-level-episode-knowledge ?top-level-name ?top-level-episode))

  ;; util
  (<- (task-full-path ?task-node ?path)
    (bound ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node ?path))

  ;; tasks of top-level
  (<- (task-of-top-level ?top-level-name ?task-node)
    (bound ?top-level-name)
    (top-level-task ?top-level-name ?top-level-task-node)
    (lisp-fun flatten-task-tree-broad ?top-level-task-node ?all-task-nodes)
    (member ?task-node ?all-task-nodes))

  ;; task for subtree
  (<- (task ?top-level-name ?subtree-path ?task-node)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (top-level-task ?top-level-name ?top-level-task)
    (lisp-fun cpl:task-tree-node ?subtree-path ?top-level-task ?subtree-task)
    ;; (lisp-fun cpl:flatten-task-tree ?subtree-task ?all-subtree-tasks)
    (lisp-fun flatten-task-tree-broad ?subtree-task ?all-subtree-tasks)
    (member ?task-node ?all-subtree-tasks))

  ;; subtask
  (<- (subtask ?task ?subtask)
    (bound ?task)
    (lisp-fun cpl:task-tree-node-children ?task ?children)
    (member (?_ . ?subtask) ?children))

  (<- (subtask ?task ?subtask)
    (not (bound ?task))
    (bound ?subtask)
    (lisp-fun cpl:task-tree-node-parent ?subtask ?task)
    (lisp-pred identity ?task))

  ;; (<- (subtask ?task ?subtask)
  ;;   (not (bound ?task))
  ;;   (not (bound ?subtask))
  ;;   (task ?task)
  ;;   (subtask ?task ?subtask))

  ;; subtask+
  (<- (subtask+ ?task ?subtask)
    (subtask ?task ?subtask))

  (<- (subtask+ ?task ?subtask)
    (subtask ?task ?tmp)
    (subtask+ ?tmp ?subtask))

  

  ;; task-sibling
  (<- (task-sibling ?task ?sibling)
    (bound ?task)
    (subtask ?parent ?task)
    (subtask ?parent ?sibling)
    (not (== ?sibling ?task)))

  ;; (<- (task-sibling ?task ?sibling)
  ;;   (not (bound ?task))
  ;;   (subtask ?parent ?sibling)
  ;;   (subtask ?parent ?task)
  ;;   (not (== ?sibling ?task)))

  ;; task-result
  (<- (task-result ?task ?result)
    (bound ?task)
    (lisp-fun cpl:task-tree-node-result ?task ?result))

  ;; task-parameter
  (<- (task-parameter ?task ?parameter)
    (bound ?task)
    (lisp-fun cpl:task-tree-node-parameters ?task ?parameters)
    (member ?parameter ?parameters))

  ;; task-status-fluent
  (<- (task-status-fluent ?task ?fluent)
    (bound ?task)
    (lisp-fun cpl:task-tree-node-status-fluent ?task ?fluent))

  (<- (fluent-value ?fluent ?value)
    (bound ?fluent)
    (not (equal ?fluent NIL))
    (lisp-fun cpl:value ?fluent ?value))

  (<- (task-status ?task ?status)
    (bound ?task)
    (task-status-fluent ?task ?fluent)
    (fluent-value ?fluent ?status))

  (<- (task-outcome ?task ?outcome)
    (bound ?task)
    (member ?outcome (:succeeded :failed :evaporated))
    (task-status ?task ?outcome))

  (<- (task-error ?task ?error)
    (bound ?task)
    (task-outcome ?task :failed)
    (task-result ?task ?result)
    (lisp-fun extract-task-error ?result ?error))

  ;; execution trace related
  (<- (coe:holds (fluent-value ?fluent ?value) ?top-level-name ?time)
    (bound ?fluent)
    (bound ?top-level-name)
    (top-level-episode-knowledge ?top-level-name ?episode)
    (lisp-pred identity ?fluent)
    (lisp-fun cpl-impl:name ?fluent ?fluent-name)
    (lisp-fun cet:episode-knowledge-fluent-durations ?fluent-name ?episode ?durations)
    (member (?value . ?duration) ?durations)
    (cram-occasions-events:duration-includes ?duration ?time))

  (<- (coe:holds (task-status ?task ?status) ?top-level-name ?time)
    (bound ?top-level-name)
    (task-status-fluent ?task ?status-fluent)
    (coe:holds (fluent-value ?status-fluent ?status) ?top-level-name ?time))

  ;; task times
  (<- (task-created-at ?top-level-name ?task ?time)
    (bound ?top-level-name)
    (bound ?task)
    (coe:holds (task-status ?task :created) ?top-level-name (coe:at ?time)))

  (<- (task-started-at ?top-level-name ?task ?time)
    (bound ?top-level-name)
    (bound ?task)
    ;; (task ?task)
    (bagof ?time
           (coe:holds (task-status ?task :running) ?top-level-name (coe:at ?time))
           ?times)
    (sort ?times < (?time . ?_)))

  (<- (task-ended-at ?top-level-name ?task ?time)
    (bound ?top-level-name)
    (bound ?task)
    ;; (task ?task)
    (member ?status (:succeeded :failed :evaporated))
    (coe:holds (task-status ?task ?status) ?top-level-name (coe:at ?time)))

  ;; task next and previous sibling
  (<- (task-next-sibling ?top-level-name ?task ?next-task)
    (bound ?top-level-name)
    (bound ?task)
    (bagof ?sibling (task-sibling ?task ?sibling) ?siblings)
    (member ?next-task ?siblings)
    (task-created-at ?top-level-name ?task ?created-time-task)
    (task-created-at ?top-level-name ?next-task ?created-time-next-task)
    (<= ?created-time-task ?created-time-next-task)
    (forall (and
             (member ?other-next-task ?siblings)
             (not (== ?next-task ?other-next-task))
             (task-created-at ?top-level-name ?other-next-task ?created-time-other-next-task)
             (<= ?created-time-task ?created-time-other-next-task))
            (<= ?created-time-next-task ?created-time-other-next-task)))

  (<- (task-previous-sibling ?top-level-name ?task ?next-task)
    (bound ?top-level-name)
    (bound ?task)
    (bagof ?sibling (task-sibling ?task ?sibling) ?siblings)
    (member ?next-task ?siblings)
    (task-created-at ?top-level-name ?task ?created-time-task)
    (task-created-at ?top-level-name ?next-task ?created-time-next-task)
    (>= ?created-time-task ?created-time-next-task)
    (forall (and
             (member ?other-next-task ?siblings)
             (not (== ?next-task ?other-next-task))
             (task-created-at ?top-level-name ?other-next-task ?created-time-other-next-task)
             (>= ?created-time-task ?created-time-other-next-task))
            (>= ?created-time-next-task ?created-time-other-next-task)))

  ;; perform tasks
  (<- (perform-task-of-top-level ?top-level-name ?task-node)
    (bound ?top-level-name)
    (task-of-top-level ?top-level-name ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node (?path . ?_))
    (equal ?path (cpl:goal (perform ?_) . ?_)))

  (<- (perform-task ?top-level-name ?subtree-path ?task-node)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (task ?top-level-name ?subtree-path ?task-node)
    (lisp-fun cpl:task-tree-node-path ?task-node (?path . ?_))
    (equal ?path (cpl:goal (perform ?_) . ?_)))

  ;; (<- (task-specific-action ?top-level-name ?subtree-path ?action-type ?task ?designator)
  ;;   (bound ?top-level-name)
  ;;   (bound ?subtree-path)
  ;;   (not (== nil ?subtree-path))
  ;;   (perform-task ?top-level-name ?subtree-path ?task)
  ;;   (task-outcome ?task :succeeded)
  ;;   (task-parameter ?task ?designator)
  ;;   (lisp-type ?designator desig:action-designator)
  ;;   (desig:desig-prop ?designator (:type ?action-type)))

 

  (<- (task-transporting-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :transporting ?task ?designator))
  
  (<- (task-navigating-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :navigating ?task ?designator))

  (<- (task-fetching-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :fetching ?task ?designator))

  (<- (task-picking-up-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :picking-up ?task ?designator))

  (<- (task-delivering-action ?top-level-name ?subtree-path ?task ?designator)
    (task-specific-action ?top-level-name ?subtree-path :delivering ?task ?designator))

    ;; task next and previous perform action sibling
  (<- (task-next-action-sibling ?top-level-name ?subtree-path ?task ?action-type ?next-task)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (bound ?task)
    ;; (bound ?action-type)
    (bagof ?sibling (task-sibling ?task ?sibling) ?siblings)
    (member ?next-task ?siblings)
    (task-specific-action ?top-level-name ?subtree-path ?action-type ?next-task ?_)
    (task-created-at ?top-level-name ?task ?created-time-task)
    (task-created-at ?top-level-name ?next-task ?created-time-next-task)
    (<= ?created-time-task ?created-time-next-task)
    (forall (and
             (member ?other-next-task ?siblings)
             (task-specific-action ?top-level-name ?subtree-path ?action-type ?other-next-task ?_)
             (not (== ?next-task ?other-next-task))
             (task-created-at ?top-level-name ?other-next-task ?created-time-other-next-task)
             (<= ?created-time-task ?created-time-other-next-task))
            (<= ?created-time-next-task ?created-time-other-next-task)))

  (<- (task-previous-action-sibling ?top-level-name ?subtree-path ?task ?action-type ?next-task)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (bound ?task)
    ;; (bound ?action-type)
    (bagof ?sibling (task-sibling ?task ?sibling) ?siblings)
    (member ?next-task ?siblings)
    (task-specific-action ?top-level-name ?subtree-path ?action-type ?next-task ?_)
    (task-created-at ?top-level-name ?task ?created-time-task)
    (task-created-at ?top-level-name ?next-task ?created-time-next-task)
    (>= ?created-time-task ?created-time-next-task)
    (forall (and
             (member ?other-next-task ?siblings)
             (not (== ?next-task ?other-next-task))
             (task-specific-action ?top-level-name ?subtree-path ?action-type ?other-next-task ?_)
             (task-created-at ?top-level-name ?other-next-task ?created-time-other-next-task)
             (>= ?created-time-task ?created-time-other-next-task))
            (>= ?created-time-next-task ?created-time-other-next-task)))

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;; Transformation utils ;;;
  (<- (task-location-description-equal ?task ?sibling)
    ;; (not (== ?task ?sibling))
    (task-parameter ?task ?desig1)
    (task-parameter ?sibling ?desig2)
    (lisp-type ?desig1 desig:action-designator)
    (lisp-type ?desig2 desig:action-designator)
    (desig:desig-prop ?desig1 (:location ?loc1))
    (desig:desig-prop ?desig2 (:location ?loc2))
    
    (or (and
         (desig:desig-prop ?loc1 (:on ?on1))
         (desig:desig-prop ?on1 (:urdf-name ?urdf-name1))
         (desig:desig-prop ?loc2 (:on ?on2))
         (desig:desig-prop ?on2 (:urdf-name ?urdf-name2))
         (equal ?urdf-name1 ?urdf-name2))
        (and
         (desig:desig-prop ?loc1 (:in ?in1))
         (desig:desig-prop ?in1 (:urdf-name ?urdf-name1))
         (desig:desig-prop ?loc2 (:in ?in2))
         (desig:desig-prop ?in2 (:urdf-name ?urdf-name2))
         (equal ?urdf-name1 ?urdf-name2))))

  (<- (task-nearby ?task ?sibling ?threshold ?location-key)
    (task-parameter ?task ?desig)
    (task-parameter ?sibling ?sibling-desig)
    (desig:desig-prop ?desig (?location-key ?loc))
    (desig:desig-prop ?sibling-desig (?location-key ?sibling-loc))
    (lisp-fun location-desig-dist ?loc ?sibling-loc ?dist)
    (< ?dist ?threshold))
  
  (<- (task-targets-nearby ?task ?sibling ?threshold)
    (task-nearby ?task ?sibling ?threshold :target))

  (<- (task-locations-nearby ?task ?sibling ?threshold)
    (task-nearby ?task ?sibling ?threshold :location))

  (<- (top-level-name ?top-level-name)
    (lisp-fun get-top-level-name ?top-level-name))

  (<- (top-level-path ?top-level-path)
    (lisp-fun get-top-level-path ?top-level-path))

  (<- (without-replacement ?task)
    (lisp-fun slot-value ?task cpl-impl::code-replacements ?replacement)
    (== ?replacement nil)
    (subtask ?task ?sub)
    (lisp-fun slot-value ?sub cpl-impl::code-replacements ?sub-replacement)
    (== ?sub-replacement nil))

  (<- (location-distance-threshold ?threshold)
    (lisp-fun get-location-distance-threshold ?threshold))

  (<- (task-of-path ?task-path ?task-node)
    (bound ?task-path)
    (top-level-name ?top-level-name)
    (top-level-task ?top-level-name ?top-level-task)
    (lisp-fun cpl:task-tree-node ?task-path ?top-level-task ?task-node))

  ;; Lazy parent backtracking.
  (<- (parent+ ?task ?parent)
    (subtask ?parent ?task))
  
  (<- (parent+ ?task ?parent)
    (subtask ?tmp ?task)
    (parent+ ?tmp ?parent))

  (<- (task-type ?task ?action-type)
    (top-level-name ?top-level-name)
    (task-full-path ?task ?task-path)
    (task-specific-action ?top-level-name ?task-path ?action-type ?task ?_))

  (<- (subtask-depth ?depth-level)
    (lisp-fun identity 4 ?depth-level))
  
  (<- (subtask-to-level ?task ?level ?action-type ?subtask)
    (task-type ?task ?action-type)
    (lisp-fun identity ?task ?subtask))
 
  (<- (subtask-to-level ?task ?level ?action-type ?subtask)
    (< 0 ?level)
    (lisp-fun 1- ?level ?lower-level)
    (subtask ?task ?lower-task)
    (subtask-to-level ?lower-task ?lower-level ?action-type ?subtask))

  (<- (action-or-motion-task ?top-level-name ?subtree-path ?action-type ?desig-type ?task-node)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (top-level-task ?top-level-name ?top-level-task)
    (lisp-fun cpl:task-tree-node ?subtree-path ?top-level-task ?subtree-task)
    (lisp-fun tasks-of-type ?subtree-task ?action-type ?desig-type ?all-matching-tasks)
    (member ?task-node ?all-matching-tasks))

  (<- (task-specific-action ?top-level-name ?subtree-path ?action-type ?task ?designator)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (not (== nil ?subtree-path))
    (action-or-motion-task ?top-level-name ?subtree-path ?action-type action-designator ?task)
    (task-outcome ?task :succeeded)
    (task-parameter ?task ?designator))

  (<- (task-specific-motion ?top-level-name ?subtree-path ?motion-type ?task ?designator)
    (bound ?top-level-name)
    (bound ?subtree-path)
    (not (== nil ?subtree-path))
    (action-or-motion-task ?top-level-name ?subtree-path ?motion-type motion-designator ?task)
    (task-outcome ?task :succeeded)
    (task-parameter ?task ?designator))
  ;;; Transformation utils ;;;
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;


  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;; Transformation rule predicates ;;;
  (<- (task-transporting ?task)
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (location-distance-threshold ?dist-threshold)
    (task-specific-action ?top-level-name ?path :transporting ?task ?_)
    (task-specific-action ?top-level-name ?path :transporting ?other-task ?_)
    (without-replacement ?task)
    (without-replacement ?other-task)
    (not (== ?task ?other-task)))
  
  
  (<- (task-transporting-siblings (?first-path ?first-fetching-desig)
                                  (?second-path ?first-delivering-desig))
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (task-transporting-action ?top-level-name ?path ?second-task ?_)
    (task-transporting-action ?top-level-name ?path ?first-task ?_)
    (without-replacement ?first-task)
    (without-replacement ?second-task)
    (not (== ?first-task ?second-task))
    (task-location-description-equal ?first-task ?second-task)
    (task-full-path ?first-task ?first-path)
    (task-full-path ?second-task ?second-path)
    (task-fetching-action ?top-level-name ?first-path ?_ ?first-fetching-desig)
    (task-delivering-action ?top-level-name ?first-path ?_ ?first-delivering-desig))

  (<- (task-transporting-with-tray (?delivering-path))
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (location-distance-threshold ?dist-threshold)
    (task-transporting-action ?top-level-name ?path ?last-task ?_)
    (task-transporting-action ?top-level-name ?path ?before-last-task ?_)
    (not (== ?last-task ?before-last-task))
    (without-replacement ?last-task)
    (without-replacement ?before-last-task)
    (task-location-description-equal ?last-task ?before-last-task)
    (task-targets-nearby ?last-task ?before-last-task ?dist-threshold)
    (task-full-path ?last-task ?last-path)
    (task-delivering-action ?top-level-name ?last-path ?delivering-task ?_)
    (task-full-path ?delivering-task ?delivering-path))

  (<- (task-transporting-with-tray-other-deliveries ?first-delivering-path (?other-path))
    (bound ?first-delivering-path)
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (location-distance-threshold ?dist-threshold)
    (task-of-path ?first-delivering-path ?del-task)
    (task-delivering-action ?top-level-name ?path ?other-del-task ?_)
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
  
  (<- (task-transporting-from-fridge ?navigate-action ?accessing-path ?closing-path)
    (top-level-name ?top-level-name)
    (top-level-path ?path)
    (top-level-task ?top-level-name ?root)
    (task-specific-action ?top-level-name ?path :transporting-from-container ?transport-action ?_)
    (task-specific-action ?top-level-name ?path :transporting-from-container ?compare ?_)
    (not (== ?transport-action ?compare))
    (without-replacement ?transport-action)
    (without-replacement ?compare)
    (location-distance-threshold ?threshold)
    (task-nearby ?compare ?transport-action ?threshold :located-at)
    (task-full-path ?transport-action ?transport-path)
    (task-specific-action ?top-level-name ?transport-path :accessing-container ?access ?_)
    (task-full-path ?access ?accessing-path)
    (task-specific-action ?top-level-name ?accessing-path :navigating ?navigate ?navigate-action)
    (task-specific-action ?top-level-name ?transport-path :closing-container ?closing ?_)
    (task-full-path ?closing ?closing-path))
  ;;; Transformation rule predicates ;;;
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 )
