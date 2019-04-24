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

(in-package :kr-assembly)

(def-object-type-in-other-object-transform :chassis :holder-plane-horizontal :chassis-attachment
  :attachment-translation `(0.084 0.0 0.022)
  :attachment-rot-matrix *rotation-around-z-90-matrix*)

(def-object-type-in-other-object-transform :bottom-wing :chassis :wing-attachment
  :attachment-translation `(0.0 -0.02 0.04;; 0.0
                                )
  :attachment-rot-matrix *identity-matrix*)

(def-object-type-in-other-object-transform :underbody :bottom-wing :body-attachment
  :attachment-translation `(0.0 -0.025 0.02)
  :attachment-rot-matrix *rotation-around-z+90-matrix*)

(def-object-type-in-other-object-transform :upper-body :underbody :body-on-body
  :attachment-translation `(-0.025 0.0 0.0425)
  :attachment-rot-matrix *identity-matrix*)

(def-object-type-in-other-object-transform :propeller :motor-grill :propeller-attachment
  :attachment-translation `(0.0 0.0 0.002)
  :attachment-rot-matrix *identity-matrix*)

(def-object-type-in-other-object-transform :front-wheel :chassis :left-wheel-attachment
  :attachment-translation `(-0.0 -0.15 0.00)
  :attachment-rot-matrix *rotation-around-x-90-matrix*)

(def-object-type-in-other-object-transform :front-wheel :chassis :right-wheel-attachment
  :attachment-translation `(-0.0 -0.15 0.00)
  :attachment-rot-matrix  *rotation-around-x-90-matrix*)

(def-object-type-in-other-object-transform :top-wing :holder-plane-vertical :vertical-attachment
  :attachment-translation `(0.025 0 0.183)
  :attachment-rot-matrix *rotation-around-x-90-and-z-90-matrix*)

(def-object-type-in-other-object-transform :bolt :upper-body :rear-thread
  :attachment-translation `(-0.0525 0.0 -0.01;; -0.025
                                    )
  :attachment-rot-matrix *identity-matrix*)

(def-object-type-in-other-object-transform :top-wing :upper-body :wing-attachment
  :attachment-translation `(0.05 0.0 0.0025)
  :attachment-rot-matrix *rotation-around-z-90-matrix*)

(def-object-type-in-other-object-transform :bolt :top-wing :middle-thread
  :attachment-translation `(0.0 0.025 0.01;; -0.005
                                )
  :attachment-rot-matrix *identity-matrix*)

(def-object-type-in-other-object-transform :window :top-wing :window-attachment
  :attachment-translation `(0.0 -0.0525 0.0075)
  :attachment-rot-matrix *rotation-around-z+90-matrix*)

(def-object-type-in-other-object-transform :bolt :window :window-thread
  :attachment-translation `(-0.0125 0.0 -0.005;; -0.02
                                    )
  :attachment-rot-matrix *identity-matrix*)

(def-object-type-in-other-object-transform :bolt :propeller :propeller-thread
  :attachment-translation `(0.0 0.0 0.01;; -0.02
                                    )
  :attachment-rot-matrix *identity-matrix*)






#+stuff-below-uses-knowrob
(
 (defun get-object-grasping-poses (on-object-name on-object-type
                                   object-name object-type arm grasp on-object-transform)
   "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"
   (mapcar (lambda (manipulation-type)
             (get-gripper-in-base-pose
              arm on-object-transform           ; bToo
              (get-object-manipulation-transform ; gToo aka oToo
               manipulation-type "left_gripper" object-name grasp))) ; bToo * ooTg = bTg
           '(:lift :connect :pregrasp :pregrasp)))

 (defun get-object-placing-poses (on-object-name on-object-type object-name object-type
                                  arm grasp on-object-transform)
   "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"
   (flet ((get-connection-id (object-name connect-to-object-name)
            (ecase object-name
              (:axle1 (ecase connect-to-object-name
                        (:chassis1 :axle-snap-in-front)))
              (:axle2 (ecase connect-to-object-name
                        (:chassis1 :axle-snap-in-back)))
              (:chassis1 (ecase connect-to-object-name
                           (:chassis-holder1 :chassis-on-holder)))
              (:camaro-body1 (ecase connect-to-object-name
                               (:chassis1 :chassis-snap-in-connection)))
              (:seat1 (ecase connect-to-object-name
                        (:chassis1 :seat-snap-in-front)))
              (:seat2 (ecase connect-to-object-name
                        (:chassis1 :seat-snap-in-back))))))

     (let* ((put-pose (cram-tf:multiply-transform-stampeds
                       cram-tf:*robot-base-frame* cram-tf:*robot-left-tool-frame*
                       (cram-tf:multiply-transform-stampeds
                        cram-tf:*robot-base-frame* (cram->knowrob object-name)
                        on-object-transform ; bToo
                        ;; (get-object-placement-transform ; oToo aka gToo
                        ;;  on-object-name object-name "left_gripper" grasp)))) ; bToo * ooTg = bTg
                        (get-object-connection-transform ; ooTo
                         (get-connection-id object-name on-object-name)
                         on-object-name
                         object-name))  ; bToo * ooTo = bTo
                       (cram-tf:transform-stamped-inv
                        (get-object-manipulation-transform ; gTo
                         :grasp "left_gripper" object-name grasp))
                       :result-as-pose-or-transform :pose))) ; bTo * oTg = bTg
       (list (get-object-type-lift-pose object-type arm grasp put-pose)
             (get-object-type-2nd-lift-pose object-type arm grasp put-pose)
             put-pose
             (get-object-type-2nd-pregrasp-pose object-type arm grasp put-pose)
             (get-object-type-pregrasp-pose object-type arm grasp put-pose)))))
 )
