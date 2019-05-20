;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :urdf-proj)

;;;;;;;;;;;;;;;;; NAVIGATION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module urdf-proj-navigation (motion-designator)
  (destructuring-bind (command argument) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-base (drive argument)))))

;;;;;;;;;;;;;;;;; TORSO ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module urdf-proj-torso (motion-designator)
  (destructuring-bind (command argument) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-torso (move-torso argument)))))

;;;;;;;;;;;;;;;;; NECK ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module urdf-proj-neck (motion-designator)
  (destructuring-bind (command goal-pose goal-configuration) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-head (look-at goal-pose goal-configuration)))))

;;;;;;;;;;;;;;;;; PERCEPTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module urdf-proj-perception (motion-designator)
  (destructuring-bind (command argument-1) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:detect (detect argument-1)))))

;;;;;;;;;;;;;;;;; GRIPPERS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module urdf-proj-grippers (motion-designator)
  (destructuring-bind (command arg-1 arg-2 &rest arg-3) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-gripper-joint (gripper-action arg-1 arg-2 (car arg-3))))))

;;;;;;;;;;;;;;;;; ARMS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module urdf-proj-arms (motion-designator)
  (destructuring-bind (command arg-1 &rest arg-2) (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-tcp (move-tcp arg-1 (first arg-2) (second arg-2)
                                                  (third arg-2) (fourth arg-2) (fifth arg-2)))
      (cram-common-designators::move-joints (move-joints arg-1 (car arg-2))))))


;;;;;;;;;;;;;;;;;;;;; PREDICATES ;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group projection-matching-pms (cpm:matching-process-module)

  (<- (cpm:matching-process-module ?motion-designator urdf-proj-navigation)
    (desig:desig-prop ?motion-designator (:type :going)))

  (<- (cpm:matching-process-module ?motion-designator urdf-proj-torso)
    (desig:desig-prop ?motion-designator (:type :moving-torso)))

  (<- (cpm:matching-process-module ?motion-designator urdf-proj-neck)
    (desig:desig-prop ?motion-designator (:type :looking)))

  (<- (cpm:matching-process-module ?motion-designator urdf-proj-perception)
    (desig:desig-prop ?motion-designator (:type :detecting)))

  (<- (cpm:matching-process-module ?motion-designator urdf-proj-grippers)
    (or (desig:desig-prop ?motion-designator (:type :gripping))
        (desig:desig-prop ?motion-designator (:type :moving-gripper-joint))
        (desig:desig-prop ?motion-designator (:type :opening-gripper))
        (desig:desig-prop ?motion-designator (:type :closing-gripper))))

  (<- (cpm:matching-process-module ?motion-designator urdf-proj-arms)
    (or (desig:desig-prop ?motion-designator (:type :moving-tcp))
        (desig:desig-prop ?motion-designator (:type :moving-arm-joints)))))
