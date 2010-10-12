;;;
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
;;;

(in-package :pr2-manip-pm)

(defun make-message (type-str slots)
  (apply #'roslisp::make-message-fn type-str slots))

(def-fact-group pr2-manipulation-designators (action-desig)

  (<- (ros-message ?type ?slots ?msg)
    (lisp-fun make-message ?type ?slots ?msg))

  (<- (side-id :right 0))
  (<- (side-id :left 1))

  (<- (action-desig ?desig (fridge-opened ?action (?obj ?side)))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to open))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (side ?side))
    (side-id ?side ?s-id)
    (desig-prop ?obj (type fridge))
    (ros-message "ias_drawer_executive/GenericGoal"
                 (:arm ?s-id)
                 ?action))
  
  (<- (action-desig ?desig (fridge-closed ?action (?obj ?side)))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to close))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type fridge))
    (rete-holds (object-opened ?obj ?side))
    (rete-holds (fridge-open-handle ?obj ?handle))
    (ros-message "ias_drawer_executive/GenericGoal"
                 (:arm ?handle)
                 ?action))

  (<- (action-desig ?desig (drawer-opened ?action (?obj ?side)))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to open))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (side ?side))
    (side-id ?side ?s-id)
    (desig-prop ?obj (type drawer))
    (ros-message "ias_drawer_executive/GenericGoal"
                 (:arm ?s-id)
                 ?action))
  
  (<- (action-desig ?desig (noop ?action (?obj ?side)))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to close))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type drawer))
    (rete-holds (drawer-open-handle ?obj ?handle))
    (ros-message "ias_drawer_executive/GenericGoal"
                 (:arm ?handle)
                 ?action))
  
  (<- (action-desig ?desig (plate-grasped-drawer ?action (?obj)))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type round-plate))
    (ros-message "ias_drawer_executive/GenericGoal"
                 (:arm 0)
                 ?action))

  (<- (action-desig ?desig (bottle-grasped ?action (?obj)))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to grasp))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (side ?side))
    (desig-prop ?obj (type bottle))
    (ros-message "ias_drawer_executive/GenericGoal"
                 (:arm 0)
                 ?action))

  (<- (action-desig ?desig (plate-put-down-island ?action (?obj)))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (at ?loc))
    (desig-prop ?obj (type round-plate))
    (desig-prop ?loc (name ?name-sym))
    (lisp-fun symbol-name ?name-sym ?name-str)
    (lisp-pred equal ?name-str "KITCHEN-ISLAND")
    (format "bla~%")
    (rete-holds (plate-grasped-handle ?obj ?handle))
    (ros-message "ias_drawer_executive/GenericGoal"
                 (:arm ?handle)
                 ?action))

  (<- (action-desig ?desig (bottle-put-down-island ?action (?obj)))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (at ?loc))
    (desig-prop ?obj (type bottle))
    (desig-prop ?loc (name ?name-sym))
    (lisp-fun symbol-name ?name-sym ?name-str)
    (lisp-pred equal ?name-str "KITCHEN-ISLAND")
    (rete-holds (bottle-grasped-handle ?obj ?handle))
    (ros-message "ias_drawer_executive/GenericGoal"
                 (:arm ?handle)
                 ?action))  
  
  ;; Noop stubs to be implemented
  (<- (action-desig ?desig (noop nil nil))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (pose open)))

  (<- (action-desig ?desig (noop nil nil))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (pose parked)))

  (<- (action-desig ?desig (noop nil nil))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to lift)))

  (<- (action-desig ?desig (noop nil nil))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to carry)))

  (<- (action-desig ?desig (noop nil nil))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to open)))

  (<- (action-desig ?desig (noop nil nil))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to close))))
