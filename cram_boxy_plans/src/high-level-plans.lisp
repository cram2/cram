;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :boxy-plans)

(defun move-arms-from-field-of-view ()
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e) ; ignore failures
         (roslisp:ros-warn (boxy-plans arm-from-field-of-view) "~a" e)
         (return)))
    (let ((?left-configuration kr-belief::*left-arm-out-of-field-of-view-state*))
      (exe:perform
       (desig:a motion
                (type moving-arm-joints)
                (left-configuration ?left-configuration))))))

(defun find-object (?object-type)
  ;; move arms from field of view
  (move-arms-from-field-of-view)
  ;; detect object with kinect
  (let ((?object
          (exe:perform (desig:an action
                                 (type detecting)
                                 (object (desig:an object (type ?object-type)))))))
    ;; move wrist with camera above object to look closer
    (exe:perform
     (desig:an action
               (type looking)
               (object ?object)
               (camera wrist)))
    ;; inspect object using camera wrist
    (desig:equate ?object
                  (exe:perform (desig:an action
                                 (type inspecting)
                                 (object (desig:an object (type ?object-type))))))
    ;; move arms from field of view
    ;; (move-arms-from-field-of-view)
    ;; return found object
    (desig:current-desig ?object)))








(defun test-stuff ()
  (let ((?obj (boxy-plans::detect (desig:an object (type chassis)))))
    (cram-process-modules:with-process-modules-running
        (boxy-pm:base-pm boxy-pm:neck-pm boxy-pm:grippers-pm boxy-pm:body-pm)
      (cpl:top-level
        (exe:perform
         (desig:an action
                   (type picking-up)
                   (object ?obj)
                   (arm left))))))
  (let ((?obj (boxy-plans::detect (desig:an object (type chassis-holder)))))
    (cram-process-modules:with-process-modules-running
        (boxy-pm:base-pm boxy-pm:neck-pm boxy-pm:grippers-pm boxy-pm:body-pm)
      (cpl:top-level
        (exe:perform
         (desig:an action
                   (type placing)
                   (object (desig:an object (type chassis)))
                   (on-object ?obj)
                   (arm left))))))
  (let ((?obj (boxy-plans::detect (desig:an object (type camaro-body)))))
    (cram-process-modules:with-process-modules-running
        (boxy-pm:base-pm boxy-pm:neck-pm boxy-pm:grippers-pm boxy-pm:body-pm)
      (cpl:top-level
        (exe:perform
         (desig:an action
                   (type looking)
                   (object ?obj)
                   (camera wrist))))))
  (let ((?pose (cl-transforms-stamped:transform-pose-stamped
                cram-tf:*transformer*
                :target-frame cram-tf:*robot-base-frame*
                :pose (cl-transforms-stamped:make-pose-stamped
                       cram-tf:*robot-left-tool-frame*
                       0.0
                       (cl-transforms:make-identity-vector)
                       (cl-transforms:make-identity-rotation)))))
    (cram-process-modules:with-process-modules-running
        (boxy-pm:base-pm boxy-pm:neck-pm boxy-pm:grippers-pm boxy-pm:body-pm)
      (cpl:top-level
        (cram-executive:perform
         (desig:a action
                  (type pushing)
                  (left-poses (?pose)))))))
  (let ((?obj (boxy-plans::detect (desig:an object (type axle))))
        (?with-obj (boxy-plans::detect (desig:an object (type chassis)))))
    (cram-process-modules:with-process-modules-running
        (boxy-pm:base-pm boxy-pm:neck-pm boxy-pm:grippers-pm boxy-pm:body-pm)
      (cpl:top-level
        (exe:perform
         (desig:an action
                   (type connecting)
                   (object ?obj)
                   (with-object ?with-obj)
                   (arm left)))))))


