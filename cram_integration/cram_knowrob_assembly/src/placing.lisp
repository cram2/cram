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

(defvar *known-attachment-types* nil
  "A list of symbols representing all known attachment types")

(defgeneric get-object-type-in-other-object-transform (object-type object-name
                                                       other-object-type other-object-name
                                                       attachment))

(defun get-object-placement-transform (object-name object-type object-transform
                                       other-object-name other-object-type other-object-transform
                                       attachment-type)
  (declare (ignore object-transform))
  "Returns a transform in robot base frame where the object named `object-name' should go"
  (let* ((base-frame
           cram-tf:*robot-base-frame*)
         (object-frame
           (roslisp-utilities:rosify-underscores-lisp-name object-name))
         (base-to-object-transform  ; bTo = bToo * ooTo
           (cram-tf:multiply-transform-stampeds
            base-frame
            object-frame
            other-object-transform
            (get-object-type-in-other-object-transform ; ooTo
             object-type object-name other-object-type other-object-name attachment-type))))
    base-to-object-transform))



(defmacro def-object-type-in-other-object-transform (object-type other-object-type attachment-type
                                                     &key
                                                       (attachment-translation ''(0.0 0.0 0.0))
                                                       (attachment-rot-matrix ''((1.0 0.0 0.0)
                                                                                 (0.0 1.0 0.0)
                                                                                 (0.0 0.0 1.0))))
  `(let ((evaled-object-type ,object-type)
         (evaled-other-object-type ,other-object-type)
         (evaled-attachment-type ,attachment-type)
         (evaled-attachment-translation ,attachment-translation)
         (evaled-attachment-rot-matrix ,attachment-rot-matrix))
     (let ((object-list
             (if (listp evaled-object-type)
                 evaled-object-type
                 (list evaled-object-type)))
           (other-object-list
             (if (listp evaled-other-object-type)
                 evaled-other-object-type
                 (list evaled-other-object-type))))
       (mapcar (lambda (object)
                 (mapcar (lambda (other-object)
                           (let ((transform
                                   (cl-transforms-stamped:make-transform-stamped
                                    (roslisp-utilities:rosify-underscores-lisp-name other-object)
                                    (roslisp-utilities:rosify-underscores-lisp-name object)
                                    0.0
                                    (cl-transforms:make-3d-vector
                                     (first evaled-attachment-translation)
                                     (second evaled-attachment-translation)
                                     (third evaled-attachment-translation))
                                    (cl-transforms:matrix->quaternion
                                     (make-array '(3 3)
                                                 :initial-contents
                                                 evaled-attachment-rot-matrix)))))

                             (pushnew evaled-attachment-type *known-attachment-types*)

   (defmethod get-object-type-in-other-object-transform ((object-type (eql object))
                                                         object-name
                                                         (other-object-type (eql other-object))
                                                         other-object-name
                                                         (attachment (eql evaled-attachment-type)))
         (let ((attachment-transform transform))
           (if attachment-transform
               (cram-tf:copy-transform-stamped
                attachment-transform
                :frame-id (roslisp-utilities:rosify-underscores-lisp-name other-object-name)
                :child-frame-id (roslisp-utilities:rosify-underscores-lisp-name object-name))
               (error "Attachment transform not defined for ~a with ~a attached with ~a~%"
                      object other-object attachment))))

                             ))
                         other-object-list))
               object-list))))


(defparameter *rotation-around-z-90-matrix*
  '(( 0  1  0)
    (-1  0  0)
    ( 0  0  1)))

(defparameter *rotation-around-z+90-matrix*
  '((0 -1  0)
    (1  0  0)
    (0  0  1)))

(defparameter *identity-matrix*
  '((1 0 0)
    (0 1 0)
    (0 0 1)))

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
