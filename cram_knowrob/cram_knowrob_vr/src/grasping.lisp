;;;
;;; Copyright (c) 2018, Alina Hawkin <hawkin@cs.uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :kvr)

(defun human-to-robot-hand-transform ()
  "Defines the offset between the human hand from the virtual reality to the
robot standart gripper, which has been calculated manually.
RETURNS: a cl-transform."
  (let ((alpha 0)) ; (/ pi 4)
    (cl-transforms:make-transform
     (cl-transforms:make-3d-vector 0.0 -0.07 0.2)
     (cl-transforms:matrix->quaternion
      (make-array '(3 3)
                  :initial-contents
                  `((0                1 0)
                    (,(- (cos alpha)) 0 ,(- (sin alpha)))
                    (,(- (sin alpha)) 0 ,(cos alpha))))))))

(defun calculate-transform-object-type-T-gripper (object-type)
  (let ((prolog-object-type
          (roslisp-utilities:rosify-lisp-name (object-type-fixer object-type))))
    (cl-transforms:transform*
     (cl-transforms:transform-inv
      (car (query-object-location-by-object-type prolog-object-type "Start")))
     (car (query-hand-location-by-object-type prolog-object-type "Start"))
     (human-to-robot-hand-transform))))


(defmethod get-object-type-to-gripper-transform (object-type
                                                 object-name
                                                 arm
                                                 (grasp (eql :human-grasp)))

  (let* ((transform
           (calculate-transform-object-type-T-gripper object-type))
         (transform-stamped
           (cl-tf:transform->transform-stamped
            (roslisp-utilities:rosify-underscores-lisp-name object-name)
            (ecase arm
              (:left cram-tf:*robot-left-tool-frame*)
              (:right cram-tf:*robot-right-tool-frame*))
            0.0
            transform)))
    transform-stamped))

(defmethod get-object-type-to-gripper-pregrasp-transform (object-type
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped
   grasp-transform
   :x-offset (- objects::*cereal-pregrasp-xy-offset*)
   :z-offset objects::*lift-z-offset*))

(defmethod get-object-type-to-gripper-2nd-pregrasp-transform (object-type
                                                              object-name
                                                              arm
                                                              (grasp (eql :human-grasp))
                                                              grasp-transform)
  (cram-tf:translate-transform-stamped
   grasp-transform
   :x-offset (- objects::*cereal-pregrasp-xy-offset*)))

(defmethod get-object-type-to-gripper-lift-transform (object-type
                                                      object-name
                                                      arm
                                                      (grasp (eql :human-grasp))
                                                      grasp-transform)
  (cram-tf:translate-transform-stamped
   grasp-transform
   :z-offset objects::*lift-z-offset*))

(defmethod get-object-type-to-gripper-2nd-lift-transform (object-type
                                                          object-name
                                                          arm
                                                          (grasp (eql :human-grasp))
                                                          grasp-transform)
  (cram-tf:translate-transform-stamped
   grasp-transform
   :x-offset 0.0))
