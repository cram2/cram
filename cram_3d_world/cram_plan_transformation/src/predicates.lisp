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
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;; General task-tree utils ;;;
  ;; Provides the task-tree's name
  (<- (top-level-name ?top-level-name)
    (lisp-fun get-top-level-name ?top-level-name))

  ;; Provides paths of the highest-level tasks just below the root
  ;; Non-det.: Gives chronologically first paths first
  (<- (top-level-path ?top-level-path)
    (top-level-name ?top-level-name)
    (bagof ?path
           (and (coe:top-level-task ?top-level-name ?top-level-task)
                (coe:subtask ?top-level-task ?subtask)
                (coe:task-full-path ?subtask ?path))
           ?paths-reverse-bag) 
    (lisp-fun cut:force-ll ?paths-reverse-bag ?paths-reverse)
    (lisp-fun reverse ?paths-reverse ?paths) 
    (member ?top-level-path ?paths))
  
  ;; T if given ?task and direct subtasks have no code replacement
  (<- (without-replacement ?task)
    (bound ?task)
    (once
     (lisp-fun slot-value ?task cpl-impl::code-replacements ?replacement)
     (== ?replacement nil)
     (forall (coe:subtask ?task ?sub)
             (and (lisp-fun slot-value ?sub cpl-impl::code-replacements ?sub-replacement)
                  (== ?sub-replacement nil)))))

  
  ;; Needed for evaluation, since cpoe:task-specific-action is only for actions
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
  ;;; General task-tree utils ;;;
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;; Transformation utils ;;;
  ;; T if the fetch or deliver locations of ?task involve :type :fridge
  (<- (task-transport-includes-fridge ?task)
    (bound ?task)
    (top-level-name ?top-level-name)
    (coe:subtask ?task ?sub-task)
    (coe:task-full-path ?sub-task ?sub-path)
    (or
     (and
      (cpoe:task-fetching-action ?top-level-name
                                 ?sub-path
                                 ?sub-task
                                 ?sub-desig)
      (desig:desig-prop ?sub-desig (:object ?object))
      (desig:desig-prop ?object (:location ?loc)))
     (and
      (cpoe:task-delivering-action ?top-level-name
                                   ?sub-path
                                   ?sub-task
                                   ?sub-desig)
      (desig:desig-prop ?sub-desig (:target ?loc))))
    (desig:desig-prop ?loc (:in ?in))
    (desig:desig-prop ?in (:type :fridge)))

  ;; Compares urdf-name between two location designators
  (<- (task-location-description-equal ?location-1 ?location-2)
    (desig:loc-desig? ?location-1)
    (desig:loc-desig? ?location-2)
    (once
     (or (and
          (desig:desig-prop ?location-1 (:on ?region-1))
          (desig:desig-prop ?location-2 (:on ?region-2)))
         (and
          (desig:desig-prop ?location-1 (:in ?region-1))
          (desig:desig-prop ?location-2 (:in ?region-2)))
         (and
          (desig:desig-prop ?location-1 (:above ?region-1))
          (desig:desig-prop ?location-2 (:above ?region-2)))
         (and
          (desig:desig-prop ?location-1 (:below ?region-1))
          (desig:desig-prop ?location-2 (:below ?region-2)))))
    (desig:desig-prop ?region-1 (:urdf-name ?urdf-name))
    (desig:desig-prop ?region-2 (:urdf-name ?urdf-name)))

  ;; T if ?first-task and ?second-task deliver to an equal target
  ;; Provides the corresponding delivering tasks and their ?target-location
  (<- (task-transporting-to-similar-target ?first-task
                                           ?second-task
                                           ?deliver-task-1
                                           ?deliver-task-2
                                           ?target-location)
    (bound ?first-task)
    (bound ?second-task)
    (top-level-name ?top-level-name)
    (coe:subtask ?first-task ?deliver-task-1)
    (coe:task-full-path ?deliver-task-1 ?deliver-path-1)
    (cpoe:task-delivering-action ?top-level-name
                                 ?deliver-path-1
                                 ?deliver-task-1
                                 ?deliver-desig-1)
    (coe:subtask ?second-task ?deliver-task-2)
    (coe:task-full-path ?deliver-task-2 ?deliver-path-2)
    (cpoe:task-delivering-action ?top-level-name
                                 ?deliver-path-2
                                 ?deliver-task-2
                                 ?deliver-desig-2)
    (desig:desig-prop ?deliver-desig-1 (:target ?target-location-1))
    (desig:desig-prop ?deliver-desig-2 (:target ?target-location-2))
    (task-location-description-equal ?target-location-1
                                          ?target-location-2)
    (task-find-sibling-with-similar-location ?deliver-task-1 :searching
                                             ?target-location-1 ?searching-task)
    (coe:task-parameter ?searching-task ?searching-desig)
    (desig:desig-prop ?searching-desig (:location ?target-location)))

  ;; T if ?first-task and ?second-task fetch from an equal location
  ;; Provides the corresponding fetching tasks and their ?origin-location
  (<- (task-transporting-from-similar-origin ?first-task
                                             ?second-task
                                             ?fetch-task-1
                                             ?fetch-task-2
                                             ?origin-location)
    (bound ?first-task)
    (bound ?second-task)
    (top-level-name ?top-level-name)
    (coe:subtask ?first-task ?fetch-task-1)
    (coe:task-full-path ?fetch-task-1 ?fetch-path-1)
    (cpoe:task-fetching-action ?top-level-name
                                 ?fetch-path-1
                                 ?fetch-task-1
                                 ?fetch-desig-1)
    (coe:subtask ?second-task ?fetch-task-2)
    (coe:task-full-path ?fetch-task-2 ?fetch-path-2)
    (cpoe:task-fetching-action ?top-level-name
                                 ?fetch-path-2
                                 ?fetch-task-2
                                 ?fetch-desig-2)
    (desig:desig-prop ?fetch-desig-1 (:object ?target-object-1))
    (desig:desig-prop ?fetch-desig-2 (:object ?target-object-2))
    (desig:desig-prop ?target-object-1 (:location ?origin-location-1))
    (desig:desig-prop ?target-object-2 (:location ?origin-location-2))
    (task-location-description-equal ?origin-location-1
                                          ?origin-location-2)
    (task-find-sibling-with-similar-location ?fetch-task-1 :searching
                                             ?origin-location-1 ?searching-task)
    (coe:task-parameter ?searching-task ?searching-desig)
    (desig:desig-prop ?searching-desig (:location ?origin-location)))

  ;; Compares the transporting distance between two transports and
  ;; provides the ?closer-deliver-task and ?farther-deliver-task
  ;; Both tasks must fetch from the same origin for a reasonable result.
  ;; Utilizes the desig:current-solution of compared locations.
  (<- (task-closer-delivery-than-another ?first-transporting-task
                                         ?second-transporting-task
                                         ?closer-deliver-task
                                         ?farther-deliver-task)
    (bound ?first-transporting-task)
    (bound ?second-transporting-task)
    (top-level-name ?top-level-name)
    ;; lazily permute two delivery tasks
    (coe:subtask ?first-transporting-task ?first-deliver-task)
    (coe:task-full-path ?first-deliver-task ?first-deliver-path)
    (cpoe:task-delivering-action ?top-level-name
                                 ?first-deliver-path
                                 ?first-deliver-task
                                 ?_)
    (coe:subtask ?second-transporting-task ?second-deliver-task)
    (coe:task-full-path ?second-deliver-task ?second-deliver-path)
    (cpoe:task-delivering-action ?top-level-name
                                 ?second-deliver-path
                                 ?second-deliver-task
                                 ?_)
    ;; '==' works only on raw values like paths, not objects like desig or tasks
    (member ?one-path (?first-deliver-path ?second-deliver-path))
    (member ?another-path (?first-deliver-path ?second-deliver-path))
    (not (== ?one-path ?another-path))
    (coe:task-from-path ?top-level-name ?one-path ?one-deliver-task)
    (coe:task-from-path ?top-level-name ?another-path ?another-deliver-task)
    ;; infer values for distance calculation
    ;; get fetching origin
    (coe:subtask ?first-transporting-task ?first-fetching-task)
    (coe:task-full-path ?first-fetching-task ?first-fetching-path)
    (cpoe:task-fetching-action ?top-level-name ?first-fetching-path 
                               ?first-fetching-task ?first-fetching-desig)
    (desig:desig-prop ?first-fetching-desig (:object ?first-object-desig))
    (desig:desig-prop ?first-object-desig (:location ?first-object-location))
    (task-find-sibling-with-similar-location ?first-fetching-task :searching
                                             ?first-object-location ?searching-task)
    (coe:task-parameter ?searching-task ?searching-desig)
    (desig:desig-prop ?searching-desig (:location ?search-loc-desig))
    (lisp-fun desig::current-solution ?search-loc-desig ?origin-pose-stamped)
    (lisp-fun cl-tf:origin ?origin-pose-stamped ?origin)
    ;; get one target pose
    (coe:task-parameter ?one-deliver-task ?one-deliver-desig)
    (desig:desig-prop ?one-deliver-desig (:target ?one-target-desig))
    (lisp-fun desig::current-solution ?one-target-desig ?one-target-pose)
    (lisp-fun cl-tf:origin ?one-target-pose ?one-target)
    ;; get another target pose
    (coe:task-parameter ?another-deliver-task ?another-deliver-desig)
    (desig:desig-prop ?another-deliver-desig (:target ?another-target-desig))
    (lisp-fun desig::current-solution ?another-target-desig ?another-target-pose)
    (lisp-fun cl-tf:origin ?another-target-pose ?another-target)
    ;; calculate distances between origin and targets
    (lisp-fun cl-tf:v-dist ?origin ?one-target ?one-distance)
    (lisp-fun cl-tf:v-dist ?origin ?another-target ?another-distance)
    (< ?one-distance ?another-distance)
    (equal ?one-deliver-task ?closer-deliver-task)
    (equal ?another-deliver-task ?farther-deliver-task))

  ;; Provides a ?sibling of ?root-task with given qualifications
  ;; The given ?location and ?action-type specifiy the ?sibling
  ;; Non-det.: Chronologically later siblings are chosen first.
  (<- (task-find-sibling-with-similar-location ?root-task
                                               ?action-type
                                               ?location
                                               ?sibling)
    (bound ?root-task)
    (bound ?action-type)
    (bound ?location)
    (top-level-name ?top-level-name)
    (coe:task-sibling ?root-task ?sibling)
    (coe:task-full-path ?sibling ?path)
    (cpoe:task-specific-action ?top-level-name ?path ?action-type ?sibling ?designator)
    (or
     (desig:desig-prop ?designator (:location ?sibling-location))
     (desig:desig-prop ?designator (:target ?sibling-location)))
    (task-location-description-equal ?location ?sibling-location))

  ;; Provides tasks and paths of two consecutive transporting actions which have
  ;; no code replacement and the same :context.
  ;; Non-det.: Takes chronologically first actions first
  (<- (task-consecutive-transports (?first-task ?path-1) (?second-task ?path-2))
    (top-level-name ?top-level-name)
    ;; two consecutive transporting actions
    (top-level-path ?path-1)
    (coe:task-from-path ?top-level-name ?path-1 ?first-task)
    (cpoe:task-transporting-action ?top-level-name
                                   ?path-1
                                   ?first-task
                                   ?transport-desig-1)
    (once (top-level-path ?path-2) 
          (cpoe:task-next-action-sibling ?top-level-name ?path-2
                                         ?first-task ?_ ?second-task))
    (coe:task-parameter ?second-task ?transport-desig-2)
    (desig:action-desig? ?transport-desig-2)
    (desig:desig-prop ?transport-desig-2 (:type :transporting))
    ;; both untouched by transformations
    (without-replacement ?first-task)
    (without-replacement ?second-task)
    ;; equal context
    (desig:desig-prop ?transport-desig-1 (:context ?context))
    (desig:desig-prop ?transport-desig-2 (:context ?context)))
  ;;; Transformation utils ;;;
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;; Transformation rule predicates ;;;
  ;; Use BOTH-HANDS for two transports to the same TARGET location
  (<- (task-transporting-both-hands-to-target ?desigs-into-path-list
                                              ?remove-task-paths)
    ;; APPLICABILITY:
    ;; * two consecutive, non-transformed transporting actions of equal context
    ;; * transports from and to the fridge are excluded since they require the
    ;;   :right arm to be free and fridges need to be closed asap
    ;; * equal location description in both delivery targets
    (top-level-name ?top-level-name)
    (task-consecutive-transports (?first-task ?path-1) (?second-task ?path-2))
    (not (task-transport-includes-fridge ?first-task))
    (not (task-transport-includes-fridge ?second-task))
    (task-transporting-to-similar-target ?first-task ?second-task
                                         ?deliver-task-1 ?deliver-task-2
                                         ?target-location)
    ;; TRANSFORMATION PARAMETERS:
    ;; * postpone first delivery until right before the second delivery
    ;;   and eliminate superfluous searching and accessing actions on the way
    #+transformation
    (transport changes:
               1. search target     1. search target
               1. access target     1. access target     
               1. search origin     1. search origin
               1. access origin     1. access origin
               1. search object     1. search object
               1. fetch object      1. fetch object
               1. deliver object    1. seal origin
               1. seal origin       --
               1. seal target       2. search origin
               --             --->  2. access origin
               2. search target     2. search object
               2. access target     2. fetch object
               2. search origin     1. deliver object
               2. access origin     2. deliver object
               2. search object     2. seal origin
               2. fetch object      2. seal target
               2. deliver object
               2. seal origin
               2. seal target)
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; Collect designators and paths
    ;; 1. deliver object - desig
    (coe:task-parameter ?deliver-task-1 ?delivering-desig-1)
    ;; 2. deliver object - desig
    (coe:task-parameter ?deliver-task-2 ?delivering-desig-2)
    ;; 2. deliver object - path
    (coe:task-full-path ?deliver-task-2 ?delivering-path-2)       
    ;; Collect designators and paths
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    (equal ?desigs-into-path-list (((?delivering-desig-1
                                     ?delivering-desig-2)
                                    ?delivering-path-2)))
    ;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;; Collect paths to remove
    ;; 1. deliver object
    (coe:task-full-path ?deliver-task-1 ?first-delivering-path)
    ;; 1. seal target
    (task-find-sibling-with-similar-location ?deliver-task-1
                                             :sealing
                                             ?target-location
                                             ?sealing-target-task-1)
    (coe:task-full-path ?sealing-target-task-1 ?first-sealing-target-path)
    ;; 2. search target
    (task-find-sibling-with-similar-location ?deliver-task-2
                                             :searching
                                             ?target-location
                                             ?searching-target-task-2)
    (coe:task-full-path ?searching-target-task-2 ?second-searching-target-path)
    ;; 2. access target
    (task-find-sibling-with-similar-location ?deliver-task-2
                                             :accessing
                                             ?target-location
                                             ?accessing-target-task-2)
    (coe:task-full-path ?accessing-target-task-2 ?second-accessing-target-path)
    ;; Collect paths to remove
    ;;;;;;;;;;;;;;;;;;;;;;;;;;
    (equal ?remove-task-paths (?first-delivering-path
                               ?first-sealing-target-path
                               ?second-searching-target-path
                               ?second-accessing-target-path)))

  
  ;; Use BOTH-HANDS for two transports from the same ORIGIN location
  #+experimental--use-with-caution--causes-memory-faults
  (<- (task-transporting-from-origin ?desigs-into-path-list
                                     ?remove-task-paths)
    ;; APPLICABILITY:
    ;; * two consecutive, non-transformed transporting actions of equal context
    ;; * transports involving the fridge are excluded since they require the
    ;;   :right arm to me free and fridges need to be closed asap
    ;; * equal location description in both fetching origins
    (top-level-name ?top-level-name)
    (task-consecutive-transports (?first-task ?path-1) (?second-task ?path-2))
    (not (task-transport-includes-fridge ?first-task))
    (not (task-transport-includes-fridge ?second-task))
    (task-transporting-from-similar-origin ?first-task ?second-task
                                           ?fetch-task-1 ?fetch-task-2
                                           ?origin-location)
    ;; TRANSFORMATION PARAMETERS:
    ;; * access both target location before fetching, the farthest location first
    ;; * deliver to the closest target first and seal the target right after
    ;;   to ensure the robot's path is not occluded to the second delivery
    ;; * since the transporting tasks must stay in order, this transformation 
    ;;   distinguishes between two cases, depending on the two delivering target's
    ;;   distances to the origin, hence their order of execution
    (task-closer-delivery-than-another ?first-task ?second-task
                                       ?closer-delivery-task ?farther-delivery-task)
    (or
     (and
      ;; delivery target of first transport is closer than in the second
      (coe:subtask ?first-task ?closer-delivery-task)
      #+transformation
      (transport changes:
                 1. search target     2. search target
                 1. access target     2. access target     
                 1. search origin     1. search target
                 1. access origin     1. access target
                 1. search object     1. search origin
                 1. fetch object      1. access origin
                 1. deliver object    1. search object
                 1. seal origin       1. fetch object
                 1. seal target       2. search object
                 --             --->  2. fetch object
                 2. search target     1. deliver object
                 2. access target     1. seal origin
                 2. search origin     1. seal target
                 2. access origin     --
                 2. search object     2. deliver object
                 2. fetch object      2. seal target
                 2. deliver object
                 2. seal origin
                 2. seal target)
      (equal ?closer-delivery-task ?deliver-task-1)
      (equal ?farther-delivery-task ?deliver-task-2)
      (coe:task-parameter ?deliver-task-1 ?deliver-desig-1)
      (desig:desig-prop ?deliver-desig-1 (:target ?deliver-target-desig-1))
      (coe:task-parameter ?deliver-task-2 ?deliver-desig-2)
      (desig:desig-prop ?deliver-desig-2 (:target ?deliver-target-desig-2))
      
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; Collect designators and paths
      ;; Get 2. search & access target desig into 1. search target path
      ;; 2. search target - desig
      (task-find-sibling-with-similar-location ?deliver-task-2 :searching
                                               ?deliver-target-desig-2
                                               ?searching-target-task-2)
      (coe:task-parameter ?searching-target-task-2 ?searching-target-desig-2)
      ;; 2. access target - desig
      (task-find-sibling-with-similar-location ?deliver-task-2 :accessing
                                               ?deliver-target-desig-2
                                               ?access-target-task-2)
      (coe:task-parameter ?access-target-task-2 ?access-target-desig-2)
      ;; 1. search target - desig
      (task-find-sibling-with-similar-location ?deliver-task-1 :searching
                                               ?deliver-target-desig-1
                                               ?searching-target-task-1)
      (coe:task-parameter ?searching-target-task-1 ?searching-target-desig-1)
      ;; 1. search target - path
      (coe:task-full-path ?searching-target-task-1 ?searching-target-path-1)
      
      ;; Get 2. search & fetch object into 1. deliver object path
      ;; 2. search object - desig
      (once
       (and
        (coe:task-sibling ?fetch-task-2 ?searching-object-task-2)
        (coe:task-full-path ?searching-object-task-2 ?searching-object-path-2)
        (cpoe:task-previous-action-sibling
         ?top-level-name ?searching-object-path-2
         ?fetch-task-2 :searching ?searching-object-task-2)))
      (coe:task-parameter ?searching-object-task-2 ?searching-object-desig-2)
      ;; 2. fetch object - desig
      (coe:task-parameter ?fetch-task-2 ?fetch-desig-2)
      ;; 1. deliver object - desig
      ;; already inferred with ?deliver-desig-1
      ;; 1. deliver object - path
      (coe:task-full-path ?deliver-task-1 ?deliver-path-1)
      ;; Collect designators and paths
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      (equal ?desigs-into-path-list (((?searching-target-desig-2
                                       ?access-target-desig-2
                                       ?searching-target-desig-1)
                                      ?searching-target-path-1)
                                     ((?searching-object-desig-2
                                       ?fetch-desig-2
                                       ?deliver-desig-1)
                                      ?deliver-path-1)))
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; Collect paths to remove
      ;; only 2 paths stay: 2. delivery and 2. seal target
      (coe:task-full-path ?deliver-task-2 ?deliver-path-2)
      (task-find-sibling-with-similar-location ?deliver-task-2 :sealing
                                               ?deliver-target-desig-2
                                               ?sealing-target-task-2)
      (coe:task-full-path ?sealing-target-task-2 ?sealing-target-path-2)
      ;; list all paths in 2nd transport except for the two above
      (bagof ?path
             (and (coe:task-sibling ?deliver-task-2 ?sibling)
                  (coe:task-full-path ?sibling ?path)
                  (not (member ?path (?deliver-path-2 ?sealing-target-path-2))))
             ?lazy-paths)
      ;; Collect paths to remove
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;      
      (lisp-fun cut:force-ll ?lazy-paths ?remove-task-paths)
     )
     (and
      ;; delivery target of second transport is closer than in the first
      (coe:subtask ?second-task ?closer-delivery-task)
      #+transformation
      (transport changes:
                 1. search target     1. search target
                 1. access target     1. access target     
                 1. search origin     2. search target
                 1. access origin     2. access target
                 1. search object     1. search origin
                 1. fetch object      1. access origin
                 1. deliver object    1. search object
                 1. seal origin       1. fetch object
                 1. seal target       --
                 --             --->  2. search object
                 2. search target     2. fetch object
                 2. access target     2. deliver object
                 2. search origin     2. seal origin
                 2. access origin     2. seal target
                 2. search object     1. deliver object
                 2. fetch object      1. seal target
                 2. deliver object
                 2. seal origin
                 2. seal target)
      (coe:subtask ?second-task ?closer-delivery-task)
      (equal ?farther-delivery-task ?deliver-task-1)
      (equal ?closer-delivery-task ?deliver-task-2)
      (coe:task-parameter ?deliver-task-1 ?deliver-desig-1)
      (desig:desig-prop ?deliver-desig-1 (:target ?deliver-target-desig-1))
      (coe:task-parameter ?deliver-task-2 ?deliver-desig-2)
      (desig:desig-prop ?deliver-desig-2 (:target ?deliver-target-desig-2))
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; Collect designators and paths
      ;; Getting 1st deliver & seal target into 2. seal target
      ;; 1. deliver object - desig
      ;; already inferred with ?deliver-desig-1
      ;; 1. seal target - desig
      (task-find-sibling-with-similar-location ?deliver-task-1 :sealing
                                               ?deliver-target-desig-1
                                               ?sealing-target-task-1)
      (coe:task-parameter ?sealing-target-task-1 ?sealing-target-desig-1)
      ;; 2. seal target - desig
      (task-find-sibling-with-similar-location ?deliver-task-2 :sealing
                                               ?deliver-target-desig-2
                                               ?sealing-target-task-2)
      (coe:task-parameter ?sealing-target-task-2 ?sealing-target-desig-2)
      ;; 2. seal target - path
      (coe:task-full-path ?sealing-target-task-2 ?sealing-target-path-2)

      ;; Getting 2nd search & access target into 1. search origin
      ;; 2. search target -  desig
      (task-find-sibling-with-similar-location ?deliver-task-2 :searching
                                               ?deliver-target-desig-2
                                               ?searching-target-task-2)
      (coe:task-parameter ?searching-target-task-2 ?searching-target-desig-2)
      ;; 2. access target - desig
      (task-find-sibling-with-similar-location ?deliver-task-2 :accessing
                                               ?deliver-target-desig-2
                                               ?access-target-task-2)
      (coe:task-parameter ?access-target-task-2 ?access-target-desig-2)
      ;; 1. search origin - desig
      (task-find-sibling-with-similar-location ?deliver-task-1 :searching
                                               ?origin-location
                                               ?search-origin-task-1)
      (coe:task-parameter ?search-origin-task-1 ?search-origin-desig-1)
      ;; 1. search origin - path
      (coe:task-full-path ?search-origin-task-1 ?search-origin-path-1)
      ;; Collect designators and paths
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      (equal ?desigs-into-path-list (((?sealing-target-desig-2
                                       ?deliver-desig-1
                                       ?sealing-target-desig-1)
                                      ?sealing-target-path-2)
                                     ((?searching-target-desig-2
                                       ?access-target-desig-2
                                       ?search-origin-desig-1)
                                      ?search-origin-path-1)))
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; Collect paths to remove
      ;; second target search path
      (coe:task-full-path ?searching-target-task-2 ?searching-target-path-2)
      ;; second target access path
      (coe:task-full-path ?access-target-task-2 ?access-target-path-2)
      ;; first delivery path
      (coe:task-full-path ?deliver-task-1 ?deliver-path-1)
      ;; first origin sealing path
      (task-find-sibling-with-similar-location ?deliver-task-1 :sealing
                                               ?origin-location
                                               ?sealing-origin-task-1)
      (coe:task-full-path ?sealing-origin-task-1 ?sealing-origin-path-1)
      ;; first target sealing path
      (coe:task-full-path ?sealing-target-task-1 ?sealing-target-path-1)
      ;; second origin search path
      (task-find-sibling-with-similar-location ?deliver-task-2 :searching
                                               ?origin-location
                                               ?search-origin-task-2)
      (coe:task-full-path ?search-origin-task-2 ?search-origin-path-2)
      ;; second origin access path
      (task-find-sibling-with-similar-location ?deliver-task-2 :accessing
                                               ?origin-location
                                               ?access-origin-task-2)
      (coe:task-full-path ?access-origin-task-2 ?access-origin-path-2)
      ;; Collect paths to remove
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      (equal ?remove-task-paths (?deliver-path-1
                                 ?sealing-origin-path-1 ?sealing-target-path-1
                                 ?searching-target-path-2 ?access-target-path-2
                                 ?search-origin-path-2 ?access-origin-path-2)))))
  ;;; Transformation rule predicates ;;;
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;)

  #+unused-and-old-predicates-for-tray-transformation
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
    (cpoe:task-specific-action ?top-level-name ?task-path ?action-type ?task ?_))))
