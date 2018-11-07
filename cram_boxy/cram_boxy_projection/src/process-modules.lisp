;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :boxy-proj)

;;;;;;;;;;;;;;;;; NAVIGATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module boxy-proj-navigation (motion-designator)
  (destructuring-bind (command argument) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-base
       (handler-case
           (drive argument))))))

;;;;;;;;;;;;;;;;; TORSO ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module boxy-proj-torso (motion-designator)
  (destructuring-bind (command argument) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-torso
       (handler-case
           (move-torso argument))))))

;;;;;;;;;;;;;;;;; PTU ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module boxy-proj-ptu (motion-designator)
  (destructuring-bind (command goal-pose goal-configuration) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-head
       (handler-case
           (look-at goal-pose goal-configuration))))))

;;;;;;;;;;;;;;;;; PERCEPTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module boxy-proj-perception (motion-designator)
  (destructuring-bind (command argument-1) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:detect
       (handler-case
           (detect argument-1))))))

;;;;;;;;;;;;;;;;; GRIPPERS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module boxy-proj-grippers (motion-designator)
  (destructuring-bind (command arg-1 arg-2 &rest arg-3) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-gripper-joint
       (handler-case
           (gripper-action arg-1 arg-2 (car arg-3)))))))

;;;;;;;;;;;;;;;;; ARMS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module boxy-proj-arms (motion-designator)
  (destructuring-bind (command arg-1 &rest arg-2) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-tcp
       (handler-case
           (move-tcp arg-1 (car arg-2))))
      (cram-common-designators::move-joints
       (handler-case
           (move-joints arg-1 (car arg-2))))
      (cram-common-designators::move-with-constraints
       (roslisp:ros-warn (boxy pms) "move-with-constraints is not supported")
       ;; (handler-case
       ;;     (move-with-constraints arg-1))
       ))))


;;;;;;;;;;;;;;;;;;;;; PREDICATES ;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group boxy-matching-pms (cpm:matching-process-module)

  (<- (cpm:matching-process-module ?motion-designator boxy-proj-navigation)
    (desig:desig-prop ?motion-designator (:type :going)))

  (<- (cpm:matching-process-module ?motion-designator boxy-proj-torso)
    (desig:desig-prop ?motion-designator (:type :moving-torso)))

  (<- (cpm:matching-process-module ?motion-designator boxy-proj-ptu)
    (desig:desig-prop ?motion-designator (:type :looking)))

  (<- (cpm:matching-process-module ?motion-designator boxy-proj-perception)
    (desig:desig-prop ?motion-designator (:type :detecting)))

  (<- (cpm:matching-process-module ?motion-designator boxy-proj-grippers)
    (or (desig:desig-prop ?motion-designator (:type :gripping))
        (desig:desig-prop ?motion-designator (:type :moving-gripper-joint))
        (and (desig:desig-prop ?motion-designator (:gripper ?_))
             (or (desig:desig-prop ?motion-designator (:type :opening))
                 (desig:desig-prop ?motion-designator (:type :closing))))))

  (<- (cpm:matching-process-module ?motion-designator boxy-proj-arms)
    (or (desig:desig-prop ?motion-designator (:type :moving-tcp))
        (desig:desig-prop ?motion-designator (:type :moving-arm-joints))
        (desig:desig-prop ?motion-designator (:type :moving-with-constraints)))))
