;;;
;;; Copyright (c) 2022, Tina Van <van@uni-bremen.de>
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

(in-package :demo)

(defun my-plan-function (&key
             &allow-other-keys)
  (print "My action designator is executable"))


(def-fact-group my-actions (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (my-plan-function ?resolved-action-designator))
    (spec:property ?action-designator (:type :my-action-designator))

    (desig:designator :action ((:type :my-action-designator))
                      ?resolved-action-designator)))

(defun whisk (&key
	       ((:object ?object-designator))
	       ((:object-type ?object-type))
	       ((:object-name ?object-name))
	       ((:arms ?arms))
	       ((:grasp ?grasp))
	       ((:context ?context))
	       ((:reso ?reso))
	       ;;((:effort ?effort))
	       ;;((:gripper-opening ?gripper-opening))
	      ; ((:left-grip-container-poses ?left-grip-container-poses))
	      ; ((:right-grip-container-poses ?right-grip-container-poses))
	       ((:left-approach-poses ?left-approach-poses))
	       ((:right-approach-poses ?right-approach-poses))
	       ((:left-start-mix-poses ?left-start-mix-poses))
	       ((:right-start-mix-poses ?right-start-mix-poses))
	       ((:left-mid-mix-poses ?left-mid-mix-poses))
               ((:right-mid-mix-poses ?right-mid-mix-poses))
               ((:left-end-mix-poses ?left-end-mix-poses))
               ((:right-end-mix-poses ?right-end-mix-poses))
               ((:left-retract-poses ?left-retract-poses))
	       ((:right-retract-poses ?right-retract-poses))
             &allow-other-keys)

  ;;just some prints cause who does not like them
  (format t "My action designator is executable; ~%
             flipping: object-type ~a object-name ~a arm: ~a grasp: ~a ~%"
	  ?object-type ?object-name ?arms ?grasp)
(format t "my reso is: ~a" ?reso)

;  (roslisp:ros-info (cut-pour pour) "Approaching")

  ;grip bowl
    ;; (exe:perform
    ;;  (desig:an action
    ;;            (type approaching)
    ;;            (left-poses ?left-grip-container-poses)
    ;;            (right-poses ?right-grip-container-poses)
    ;;            ;;(desig:when ?collision-mode
    ;; 	       ;;(collision-mode ?collision-mode))))
    ;; 	       ))
    ;; (cpl:sleep 2)
  
  ;tool to center of container
    (exe:perform
     (desig:an action
               (type approaching)
               (left-poses ?left-approach-poses)
               (right-poses ?right-approach-poses)
               ;;(desig:when ?collision-mode
	       ;;(collision-mode ?collision-mode))))
	       ))
    (cpl:sleep 2)

   ;mix
 
    (exe:perform
     (desig:an action
               (type blending)
               (left-poses ?left-start-mix-poses)
               (right-poses ?right-start-mix-poses)
               ;;(desig:when ?collision-mode
  	       (collision-mode :allow-all)))
    (cpl:sleep 2)

  (if  (eq ?context :mix)
    (exe:perform
     (desig:an action
               (type blending)
               (left-poses ?left-mid-mix-poses)
               (right-poses ?right-mid-mix-poses)
               ;;(desig:when ?collision-mode
    	       (collision-mode :allow-all))))
  (cpl:sleep 2)

    (if  (eq ?context :mix-eclipse)
    (exe:perform
     (desig:an action
               (type blending)
               (left-poses ?left-mid-mix-poses)
               (right-poses ?right-mid-mix-poses)
               ;;(desig:when ?collision-mode
    	       (collision-mode :allow-all))))
  (cpl:sleep 2)

    (exe:perform
     (desig:an action
               (type blending)
               (left-poses ?left-end-mix-poses)
               (right-poses ?right-end-mix-poses)
               ;;(desig:when ?collision-mode
    	       (collision-mode :allow-all)))
  (cpl:sleep 2)

    (format t "my poses left: ~a ~%
             my poses right: ~a ~%" ?left-retract-poses
	     ?right-retract-poses)
    (exe:perform
     (desig:an action
               (type approaching)
               (left-poses ?left-retract-poses)
               (right-poses ?right-retract-poses)
               ;;(desig:when ?collision-mode
    	       (collision-mode :allow-all)))
  (cpl:sleep 2)

  )
