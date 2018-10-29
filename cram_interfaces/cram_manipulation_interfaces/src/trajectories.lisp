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

(in-package :cram-manipulation-interfaces)

(defvar *known-grasp-types* nil
  "A list of symbols representing all known grasp types")


(defgeneric get-object-type-to-gripper-transform (object-type object-name arm grasp)
  (:documentation "Returns a pose stamped.
Gripper is defined by a convention where Z is pointing towards the object."))

(defgeneric get-object-type-to-gripper-pregrasp-transform (object-type object-name
                                                           arm grasp grasp-transform)
  (:documentation "Returns a transform stamped"))

(defgeneric get-object-type-to-gripper-2nd-pregrasp-transform (object-type object-name
                                                               arm grasp grasp-transform)
  (:documentation "Returns a transform stamped. Default value is NIL."))

(defgeneric get-object-type-to-gripper-lift-transform (object-type object-name
                                                       arm grasp grasp-transform)
  (:documentation "Returns a transform stamped"))

(defgeneric get-object-type-to-gripper-2nd-lift-transform (object-type object-name
                                                           arm grasp grasp-transform)
  (:documentation "Returns a transform stamped"))


(defmacro def-object-type-to-gripper-transforms (object-type arm grasp-type
                                                 &key
                                                   (grasp-translation ''(0.0 0.0 0.0))
                                                   (grasp-rot-matrix ''((1.0 0.0 0.0)
                                                                        (0.0 1.0 0.0)
                                                                        (0.0 0.0 1.0)))
                                                   (pregrasp-offsets ''(0.0 0.0 0.0))
                                                   (2nd-pregrasp-offsets ''(0.0 0.0 0.0))
                                                   (lift-offsets ''(0.0 0.0 0.0))
                                                   (2nd-lift-offsets ''(0.0 0.0 0.0)))
  `(let ((evaled-object-type ,object-type)
         (evaled-arm ,arm)
         (evaled-grasp-type ,grasp-type)
         (evaled-grasp-translation ,grasp-translation)
         (evaled-grasp-rot-matrix ,grasp-rot-matrix)
         (evaled-pregrasp-offsets ,pregrasp-offsets)
         (evaled-2nd-pregrasp-offsets ,2nd-pregrasp-offsets)
         (evaled-lift-offsets ,lift-offsets)
         (evaled-2nd-lift-offsets ,2nd-lift-offsets))
     (let ((object-list
             (if (listp evaled-object-type)
                 evaled-object-type
                 (list evaled-object-type)))
           (arm-list
             (if (listp evaled-arm)
                 evaled-arm
                 (list evaled-arm))))
       (mapcar (lambda (object)
                 (mapcar (lambda (arm)
                           (let ((transform
                                   (cl-transforms-stamped:make-transform-stamped
                                    (roslisp-utilities:rosify-underscores-lisp-name object)
                                    (roslisp-utilities:rosify-underscores-lisp-name arm)
                                    0.0
                                    (cl-transforms:make-3d-vector
                                     (first evaled-grasp-translation)
                                     (second evaled-grasp-translation)
                                     (third evaled-grasp-translation))
                                    (cl-transforms:matrix->quaternion
                                     (make-array '(3 3)
                                                 :initial-contents evaled-grasp-rot-matrix)))))

                             (pushnew evaled-grasp-type *known-grasp-types*)

  (defmethod get-object-type-to-gripper-transform ((object-type (eql object))
                                                   object-name
                                                   (arm (eql arm))
                                                   (grasp (eql evaled-grasp-type)))
    (let ((grasp-transform transform))
      (if grasp-transform
          (cram-tf:copy-transform-stamped
           grasp-transform
           :frame-id (roslisp-utilities:rosify-underscores-lisp-name object-name)
           :child-frame-id (ecase arm
                             (:left cram-tf:*robot-left-tool-frame*)
                             (:right cram-tf:*robot-right-tool-frame*)))
          (error "Grasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

  (defmethod get-object-type-to-gripper-pregrasp-transform ((object-type (eql object))
                                                            object-name
                                                            (arm (eql arm))
                                                            (grasp (eql evaled-grasp-type))
                                                            grasp-transform)
    (let ((pregrasp-offsets evaled-pregrasp-offsets))
      (if pregrasp-offsets
          (destructuring-bind (x y z) pregrasp-offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "Pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

  (defmethod get-object-type-to-gripper-2nd-pregrasp-transform ((object-type (eql object))
                                                                object-name
                                                                (arm (eql arm))
                                                                (grasp (eql evaled-grasp-type))
                                                                grasp-transform)
    (let ((offsets evaled-2nd-pregrasp-offsets))
      (if offsets
          (destructuring-bind (x y z) offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "Pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

  (defmethod get-object-type-to-gripper-lift-transform ((object-type (eql object))
                                                        object-name
                                                        (arm (eql arm))
                                                        (grasp (eql evaled-grasp-type))
                                                        grasp-transform)
    (let ((offsets evaled-lift-offsets))
      (if offsets
          (destructuring-bind (x y z) offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "Pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

  (defmethod get-object-type-to-gripper-2nd-lift-transform ((object-type (eql object))
                                                            object-name
                                                            (arm (eql arm))
                                                            (grasp (eql evaled-grasp-type))
                                                            grasp-transform)
    (let ((offsets evaled-2nd-lift-offsets))
      (if offsets
          (destructuring-bind (x y z) offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "Pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp))))

                             ))
                         arm-list))
               object-list))))


(defgeneric get-object-grasping-poses (object-name object-type arm grasp object-transform)
  (:documentation "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)")
  (:method (object-name object-type arm grasp object-transform)
    (declare (type symbol object-name object-type arm grasp)
             (type cl-transforms-stamped:transform-stamped object-transform))

    ;; First correct the object transform such that rotationally-symmetric objects
    ;; would not be grasped in an awkward way with weird orientations
    (when (prolog `(object-rotationally-symmetric ,object-type))
      (setf object-transform
            (cram-tf:copy-transform-stamped
             object-transform
             :rotation (cl-transforms:make-identity-rotation))))

    (let* ((gripper-tool-frame
             (ecase arm
               (:left cram-tf:*robot-left-tool-frame*)
               (:right cram-tf:*robot-right-tool-frame*)))
           (object-to-standard-gripper-transform ; oTg'
             (get-object-type-to-gripper-transform object-type object-name arm grasp))
           (object-to-standard-gripper-pregrasp-transform ; oTg'
             (get-object-type-to-gripper-pregrasp-transform
              object-type object-name arm grasp
              object-to-standard-gripper-transform))
           (object-to-standard-gripper-2nd-pregrasp-transform ; oTg'
             (get-object-type-to-gripper-2nd-pregrasp-transform
              object-type object-name arm grasp
              object-to-standard-gripper-transform))
           (object-to-standard-gripper-lift-transform ; oTg'
             (get-object-type-to-gripper-lift-transform
              object-type object-name arm grasp
              object-to-standard-gripper-transform))
           (object-to-standard-gripper-2nd-lift-transform ; oTg'
             (get-object-type-to-gripper-2nd-lift-transform
              object-type object-name arm grasp
              object-to-standard-gripper-transform))
           (standard-to-particular-gripper-transform ; g'Tg
             (cl-transforms-stamped:transform->transform-stamped
              gripper-tool-frame
              gripper-tool-frame
              0.0
              (cut:var-value
               '?transform
               (car (prolog:prolog
                     `(and (cram-robot-interfaces:robot ?robot)
                           (cram-robot-interfaces:standard-to-particular-gripper-transform
                            ?robot ?transform))))))))
      (when (and object-to-standard-gripper-transform standard-to-particular-gripper-transform)

        (flet ((object-to-standard-gripper->base-to-particular-gripper (object-to-standard-gripper)
                 (when object-to-standard-gripper
                   (let ((base-to-standard-gripper-transform
                           (cram-tf:multiply-transform-stampeds
                            cram-tf:*robot-base-frame* gripper-tool-frame
                            object-transform          ; bTo
                            object-to-standard-gripper ; oTg'
                            :result-as-pose-or-transform :transform))) ; bTo * oTg' = bTg'
                     (cram-tf:multiply-transform-stampeds ; bTg' * g'Tg = bTg
                      cram-tf:*robot-base-frame* gripper-tool-frame
                      base-to-standard-gripper-transform      ; bTg'
                      standard-to-particular-gripper-transform ; g'Tg
                      :result-as-pose-or-transform :pose)))))

          (mapcar #'object-to-standard-gripper->base-to-particular-gripper
                  (list object-to-standard-gripper-pregrasp-transform
                        object-to-standard-gripper-2nd-pregrasp-transform
                        object-to-standard-gripper-transform
                        object-to-standard-gripper-lift-transform
                        object-to-standard-gripper-2nd-lift-transform)))))))
