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

(in-package :cram-object-interfaces)

(defvar *grasps* (make-hash-table :test #'equal)
  "Hash table of all known grasps, i.e. object to gripper transforms.
E.g.:
 (spoon left top)  -> xxx
 (spoon right top) -> xxx (same)
 (spoon left side) -> yyy
 (bottle left front) -> zzz")
(defvar *pregrasps* (make-hash-table :test #'equal)
  "Hash table of all known pregrasp offsets, structured similarly to *grasps*")
(defvar *2nd-pregrasps* (make-hash-table :test #'equal)
  "Hash table of all known 2nd-pregrasp offsets, structured similarly to *grasps*")
(defvar *lifts* (make-hash-table :test #'equal)
  "Hash table of all known lift offsets, structured similarly to *grasps*")
(defvar *2nd-lifts* (make-hash-table :test #'equal)
  "Hash table of all known 2nd-lift offsets, structured similarly to *grasps*")

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
                             (setf (gethash (list object arm evaled-grasp-type) *grasps*)
                                   transform))
                           (setf (gethash (list object arm evaled-grasp-type) *pregrasps*)
                                 evaled-pregrasp-offsets)
                           (setf (gethash (list object arm evaled-grasp-type) *2nd-pregrasps*)
                                 evaled-2nd-pregrasp-offsets)
                           (setf (gethash (list object arm evaled-grasp-type) *lifts*)
                                 evaled-lift-offsets)
                           (setf (gethash (list object arm evaled-grasp-type) *2nd-lifts*)
                                 evaled-2nd-lift-offsets))
                         arm-list))
               object-list))))

(defun remove-object-type-to-gripper-transforms (object-type arm grasp-type)
  (remhash (list object-type arm grasp-type) *grasps*)
  (remhash (list object-type arm grasp-type) *pregrasps*)
  (remhash (list object-type arm grasp-type) *2nd-pregrasps*)
  (remhash (list object-type arm grasp-type) *lifts*)
  (remhash (list object-type arm grasp-type) *2nd-lifts*))

(defgeneric get-object-type-to-gripper-transform (object-type object-name arm grasp)
  (:documentation "Returns a pose stamped.
Gripper is defined by a convention where Z is pointing towards the object.")
  (:method (object-type object-name arm grasp)
    (let ((grasp-transform (gethash (list object-type arm grasp) *grasps*)))
      (if grasp-transform
          (cram-tf:copy-transform-stamped
           grasp-transform
           :frame-id (roslisp-utilities:rosify-underscores-lisp-name object-name)
           :child-frame-id (ecase arm
                             (:left cram-tf:*robot-left-tool-frame*)
                             (:right cram-tf:*robot-right-tool-frame*)))
          (error "Grasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp)))))

(defgeneric get-object-type-to-gripper-pregrasp-transform (object-type object-name arm grasp
                                                           grasp-transform)
  (:documentation "Returns a transform stamped")
  (:method (object-type object-name arm grasp grasp-transform)
    (let ((pregrasp-offsets (gethash (list object-type arm grasp) *pregrasps*)))
      (if pregrasp-offsets
          (destructuring-bind (x y z) pregrasp-offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "Pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp)))))

(defgeneric get-object-type-to-gripper-2nd-pregrasp-transform (object-type object-name
                                                               arm grasp grasp-transform)
  (:documentation "Returns a transform stamped. Default value is NIL.")
  (:method (object-type object-name arm grasp grasp-transform)
    (let ((offsets (gethash (list object-type arm grasp) *2nd-pregrasps*)))
      (if offsets
          (destructuring-bind (x y z) offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "Pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp)))))

(defgeneric get-object-type-to-gripper-lift-transform (object-type object-name
                                                       arm grasp grasp-pose)
  (:documentation "Returns a transform stamped")
  (:method (object-type object-name arm grasp grasp-transform)
    (let ((offsets (gethash (list object-type arm grasp) *lifts*)))
      (if offsets
          (destructuring-bind (x y z) offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "Pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp)))))

(defgeneric get-object-type-to-gripper-2nd-lift-transform (object-type object-name
                                                           arm grasp grasp-pose)
  (:documentation "Returns a pose stamped")
  (:method (object-type object-name arm grasp grasp-transform)
    (let ((offsets (gethash (list object-type arm grasp) *2nd-lifts*)))
      (if offsets
          (destructuring-bind (x y z) offsets
            (cram-tf:translate-transform-stamped
             grasp-transform
             :x-offset x :y-offset y :z-offset z))
          (error "Pregrasp transform not defined for object type ~a with arm ~a and grasp ~a~%"
                 object-type arm grasp)))))


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
             (get-object-type-to-gripper-transform
              (find-most-specific-type-for-generic
               #'get-object-type-to-gripper-transform object-type)
              object-name arm grasp))
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
             (cl-tf:transform->transform-stamped
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

(defgeneric get-object-type-gripping-effort (object-type)
  (:documentation "Returns effort in Nm, e.g. 50."))

(defgeneric get-object-type-gripper-opening (object-type)
  (:documentation "How wide to open the gripper before grasping, in m."))


(def-fact-group object-knowledge (object-rotationally-symmetric orientation-matters object-type-grasp)

  (<- (object-rotationally-symmetric ?object-type)
    (fail))

  ;; The predicate ORIENTATION-MATTERS holds for all objects where the
  ;; orientation really matters when putting down the object. E.g. for
  ;; knives, forks, etc, the orientation is important while for plates
  ;; the orientation doesn't matter at all.
  (<- (orientation-matters ?object-type-symbol)
    (fail))

  (<- (object-type-grasp ?object-type ?grasp)
    (fail)))

;; todo: object-type-grasp should be calculated based on *grasps*
;; it should also have an additional argument ARM

(defun make-specifier-list (generic object-type)
  (cons
   `(eql ,object-type)
   (make-list
    (- (length (sb-pcl:generic-function-lambda-list generic)) 1)
    :initial-element t)))

(defun probe (generic object-type)
  (find-method generic
               '()
               (make-specifier-list generic object-type)
               nil))

(defun get-direct-supertypes (object-type)
  (mapcar
   (lambda (x) (cut:with-vars-bound (?super) x ?super))
   (cut:force-ll
    (prolog:prolog `(object-type-direct-subtype ?super ,object-type)))))

(defun find-most-specific-type-for-generic (generic object-type)
  (if (probe generic object-type)
      object-type
      (car (mapcar
            (alexandria:curry #'find-most-specific-type-for-generic generic)
            (get-direct-supertypes object-type)))))
