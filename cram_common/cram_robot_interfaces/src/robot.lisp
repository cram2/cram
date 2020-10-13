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

(in-package :cram-robot-interfaces)

(defparameter *robot-description-parameter* "robot_description"
  "ROS parameter that contains the robot description.")

(defvar *robot-urdf* nil
  "A cl-urdf object corresponding to parsed robot urdf.")

(defvar *robot-name* nil
  "Variable that stores the name of the current robot,
so the robot whose brain this is.")

(defun set-robot-name (new-name)
  (setf *robot-name* new-name))

(defun get-robot-name ()
  (or *robot-name* (error "[rob-int] Variable *ROBOT-NAME* is NIL.~%~
                           Have you initialized it from a URDF on the ~
                           ROS parameter server?")))

(def-fact-group robot (robot
                       robot-odom-frame robot-base-frame robot-base-link
                       robot-torso-link-joint
                       robot-joint-states robot-pose)
  (<- (robot ?robot-name)
    (symbol-value *robot-name* ?robot-name)
    (once (or (lisp-pred identity ?robot-name)
              (lisp-pred error "[rob-int] Variable *ROBOT-NAME* is NIL.~%~
                                Have you initialized it from a URDF on the ~
                                ROS parameter server?"))))

  (<- (robot-odom-frame ?robot-name ?odom-frame)
    (fail))

  (<- (robot-base-frame ?robot-name ?base-frame)
    (fail))

  ;; robot-base-frame can be a virtual link, this one should be a physical link
  (<- (robot-base-link ?robot-name ?base-link)
    (fail))

  (<- (robot-torso-link-joint ?robot-name ?torso-link ?torso-joint)
    (fail))

  (<- (robot-joint-states ?robot-name ?joints-group ?left-or-right-or-which
                          ?configuration-name ?joint-states)
    (fail))

  (<- (robot-pose ?robot-name ?pose-for-which-joint-group ?left-or-right-or-which
                  ?pose-name ?pose)
    (fail)))


(def-fact-group utils (arms arms-that-are-not-neck)
  (<- (arms ?robot-name ?arms)
    (once (or (setof ?arm (rob-int:arm ?robot-name ?arm) ?arms)
              (equal ?arms NIL))))

  (<- (arms-that-are-not-neck ?robot-name ?arms)
    (once (or (setof ?arm (and (rob-int:arm ?robot-name ?arm)
                               (not (rob-int:neck ?robot-name ?arm)))
                     ?arms)
              (equal ?arms NIL)))))
