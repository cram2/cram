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
;;;

(in-package :pex)

(defvar *open-pot-drawer-action* nil)
(defvar *put-pot-on-oven-action* nil)
(defvar *turn-knob-action* nil)
(defvar *open-corn-drawer-action* nil)
(defvar *close-corn-drawer-action* nil)
(defvar *pick-lid-from-drawer-action* nil)
(defvar *pick-bowl-from-drawer-action* nil)
(defvar *pour-corn-action* nil)
(defvar *put-lid-on-pot-action* nil)
(defvar *put-lid-on-island-action* nil)
(defvar *pick-up-pot-action* nil)
(defvar *pour-pot-action* nil)

(defun init-manipulation-actions ()
  (setf *open-pot-drawer-action*
        (actionlib:make-action-client "get_pot_out_open_drawer" "ias_drawer_executive/GenericAction"))
  (setf *put-pot-on-oven-action*
        (actionlib:make-action-client "get_pot_out_pick_place_pot" "ias_drawer_executive/GenericAction"))
  (setf *turn-knob-action*
        (actionlib:make-action-client "manipulate_knob" "ias_drawer_executive/GenericAction"))
  (setf *open-corn-drawer-action*
        (actionlib:make-action-client "open_drawer_under_oven" "ias_drawer_executive/GenericAction"))
  (setf *close-corn-drawer-action*
        (actionlib:make-action-client "close_drawer_under_oven" "ias_drawer_executive/GenericAction"))
  (setf *pick-lid-from-drawer-action*
        (actionlib:make-action-client "get_lid_out" "ias_drawer_executive/GenericAction"))
  (setf *pick-bowl-from-drawer-action*
        (actionlib:make-action-client "get_bowl_out" "ias_drawer_executive/GenericAction"))
  (setf *pour-corn-action*
        (actionlib:make-action-client "pour_popcorn" "ias_drawer_executive/GenericAction"))
  (setf *put-lid-on-pot-action*
        (actionlib:make-action-client "put_lid_on" "ias_drawer_executive/GenericAction"))
  (setf *put-lid-on-island-action*
        (actionlib:make-action-client "remove_lid" "ias_drawer_executive/GenericAction"))
  (setf *pick-up-pot-action*
        (actionlib:make-action-client "take_pot_from_island" "ias_drawer_executive/GenericAction"))
  (setf *pour-pot-action*
        (actionlib:make-action-client "pour_ready_popcorn" "ias_drawer_executive/GenericAction")))

(cram-roslisp-common:register-ros-init-function init-manipulation-actions)

(def-fact-group popcorn-actions (action-desig)

  (<- (current-desig-location ?desig ?curr-loc)
    (obj-desig? ?desig)
    (desig-prop ?desig (at ?loc-desig))
    (lisp-fun current-desig ?loc-desig ?curr-loc))

  (<- (action-desig ?desig (open-pot-drawer))
    (desig-prop ?desig (to open))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type drawer))
    (desig-prop ?obj (name drawer-1)))

  ;; pick-up pot from drawer (noop in current action interface)
  (<- (action-desig ?desig (noop))
    (desig-prop ?desig (to pick-up))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type pot))
    (current-desig-location ?obj ?loc)
    (desig-prop ?loc (in ?in-obj))
    (desig-prop ?in-obj (type drawer)))

  (<- (action-desig ?desig (put-pot-on-oven ?obj ?loc))
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (location ?loc))
    (desig-prop ?obj (type pot))
    (desig-prop ?loc (on hotplate)))

  ;; closing the drawer the pot was in is a noop in the current action
  ;; interface
  (<- (action-desig ?desig (noop))
    (desig-prop ?desig (to close))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type drawer))
    (desig-prop ?obj (name drawer-1)))

  (<- (action-desig ?desig (open-corn-drawer))
    (desig-prop ?desig (to open))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type drawer))
    (desig-prop ?obj (name drawer-2)))

  (<- (action-desig ?desig (close-corn-drawer))
    (desig-prop ?desig (to close))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type drawer))
    (desig-prop ?obj (name drawer-2)))

  (<- (action-desig ?desig (pick-lid-from-drawer))
    (desig-prop ?desig (to pick-up))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type lid))
    (current-desig-location ?obj ?loc)
    (desig-prop ?loc (in ?in-obj))
    (desig-prop ?in-obj (type drawer)))

  (<- (action-desig ?desig (pick-bowl-from-drawer))
    (desig-prop ?desig (to pick-up))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type bowl))
    (current-desig-location ?obj ?loc)
    (desig-prop ?loc (in ?in-obj))
    (desig-prop ?in-obj (type drawer)))
  
  (<- (action-desig ?desig (pour-corn))
    (desig-prop ?desig (to pour))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type bowl)))

  ;; put-down-bowl is noop at the moment
  (<- (action-desig ?desig (noop))
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (location ?loc))
    (desig-prop ?obj (type bowl))
    (desig-prop ?loc (on kitchen-island)))
  
  (<- (action-desig ?desig (put-lid-on-pot ?obj ?loc))
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (location ?loc))
    (desig-prop ?obj (type lid))
    (desig-prop ?loc (on ?on-obj))
    (desig-prop ?on-obj (type pot)))

  ;; picking up lid from pot is a noop at the moment
  (<- (action-desig ?desig (noop))
    (desig-prop ?desig (to pick-up))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type lid))
    (current-desig-location ?obj ?loc)
    (desig-prop ?loc (on ?on-obj))
    (desig-prop ?on-obj (type pot)))
  
  (<- (action-desig ?desig (put-lid-on-island ?obj ?loc))
    (desig-prop ?desig (to put-down))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?desig (location ?loc))
    (desig-prop ?obj (type lid))
    (desig-prop ?loc (on kitchen-island)))

  (<- (action-desig ?desig (pick-up-pot))
    (desig-prop ?desig (to pick-up))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type pot))
    (current-desig-location ?obj ?loc)
    (desig-prop ?loc (on hotplate)))

  (<- (action-desig ?desig (pour-pot))
    (desig-prop ?desig (to pour))
    (desig-prop ?desig (obj ?obj))
    (desig-prop ?obj (type pot))))

(defun relocate-obj (obj new-location)
  (make-designator
   'object
   (cons `(at ,new-location)
         (remove 'at (description obj) :key #'car))
   obj))

(defun make-generic-goal (&optional (param 0))
  (roslisp:make-msg "ias_drawer_executive/GenericGoal" :arm param))

(defgeneric execute-action (name &rest params)
  (:method ((act (cl:eql 'noop)) &rest params)
    (declare (ignore params))
    (roslisp:ros-info (manipulation popcorn-executive) "NOOP"))

  (:method ((act (cl:eql 'open-pot-drawer)) &rest params)
    (declare (ignore params))
    (roslisp:ros-info (manipulation popcorn-executive) "open-pot-drawer")
    (actionlib:send-goal-and-wait *open-pot-drawer-action* (make-generic-goal)))

  (:method ((act (cl:eql 'put-pot-on-oven)) &rest params)
    (roslisp:ros-info (manipulation popcorn-executive) "put-pot-on-oven ~a" params)
    (actionlib:send-goal-and-wait *put-pot-on-oven-action* (make-generic-goal))
    (apply #'relocate-obj params))

  (:method ((act (cl:eql 'open-corn-drawer)) &rest params)
    (declare (ignore params))
    (roslisp:ros-info (manipulation popcorn-executive) "open-corn-drawer")
    (actionlib:send-goal-and-wait *open-corn-drawer-action* (make-generic-goal)))

  (:method ((act (cl:eql 'close-corn-drawer)) &rest params)
    (declare (ignore params))
    (roslisp:ros-info (manipulation popcorn-executive) "close-corn-drawer")
    (actionlib:send-goal-and-wait *close-corn-drawer-action* (make-generic-goal)))

  (:method ((act (cl:eql 'pick-lid-from-drawer)) &rest params)
    (declare (ignore params))
    (roslisp:ros-info (manipulation popcorn-executive) "pick-lid-from-drawer")
    (actionlib:send-goal-and-wait *pick-lid-from-drawer-action* (make-generic-goal)))

  (:method ((act (cl:eql 'pick-bowl-from-drawer)) &rest params)
    (declare (ignore params))
    (roslisp:ros-info (manipulation popcorn-executive) "pick-bowl-from-drawer")
    (actionlib:send-goal-and-wait *pick-bowl-from-drawer-action* (make-generic-goal)))

  (:method ((act (cl:eql 'pour-corn)) &rest params)
    (declare (ignore params))
    (roslisp:ros-info (manipulation popcorn-executive) "pour-corn")
    (actionlib:send-goal-and-wait *pour-corn-action* (make-generic-goal)))

  (:method ((act (cl:eql 'put-lid-on-pot)) &rest params)
    (roslisp:ros-info (manipulation popcorn-executive) "put-lid-on-pot ~a" params)
    (actionlib:send-goal-and-wait *put-lid-on-pot-action* (make-generic-goal))
    (apply #'relocate-obj params))

  (:method ((act (cl:eql 'put-lid-on-island)) &rest params)
    (roslisp:ros-info (manipulation popcorn-executive) "put-lid-on-island ~a" params)
    (actionlib:send-goal-and-wait *put-lid-on-island-action* (make-generic-goal))
    (apply #'relocate-obj params))

  (:method ((act (cl:eql 'pick-up-pot)) &rest params)
    (declare (ignore params))
    (roslisp:ros-info (manipulation popcorn-executive) "pick-up-pot")
    (actionlib:send-goal-and-wait *pick-up-pot-action* (make-generic-goal)))

  (:method ((act (cl:eql 'pour-pot)) &rest params)
    (declare (ignore params))
    (roslisp:ros-info (manipulation popcorn-executive) "pour-pot")
    (actionlib:send-goal-and-wait *pour-pot-action* (make-generic-goal))))

(def-process-module popcorn-manipulation-pm (action)
  (apply #'execute-action (reference action)))
