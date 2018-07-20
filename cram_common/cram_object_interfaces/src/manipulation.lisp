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

(defgeneric get-object-type-gripping-effort (object-type)
  (:documentation "Returns effort in Nm, e.g. 50."))

(defgeneric get-object-type-gripper-opening (object-type)
  (:documentation "How wide to open the gripper before grasping, in m."))

(defparameter *index->axis*
  '((0 . :x)
    (1 . :y)
    (2 . :z)))

(defparameter *axis-index->face*
  '(((:x +1) . :front)
    ((:x -1) . :back)
    ((:y +1) . :left-side)
    ((:y -1) . :right-side)
    ((:z +1) . :top)
    ((:z -1) . :bottom)))

(defun calculate-vector-face (vector)
  (let* ((axis-index-list
           (loop for x in vector
                 for i = 0 then (1+ i)
                 with max-value = most-negative-single-float
                 with max-index
                 with max-sign
                 do (when (> (abs x) max-value)
                      (setf max-value (abs x))
                      (setf max-index i)
                      (setf max-sign (if (= x 0) +1 (truncate (/ x (abs x))))))
                 finally (return (list (cdr (assoc max-index *index->axis*))
                                       max-sign)))))
    (cdr (assoc axis-index-list *axis-index->face* :test #'equal))))

(defun calculate-object-faces (robot-to-object-transform)
  (let* ((object-to-robot-transform
           (cram-tf:transform-stamped-inv robot-to-object-transform))
         (matrix
           (cl-transforms:quaternion->matrix
            (cl-transforms:rotation object-to-robot-transform)))
         (robot-negative-x-vector
           (list (- (aref matrix 0 0)) (- (aref matrix 1 0)) (- (aref matrix 2 0))))
         (robot-negative-z-vector
           (list (- (aref matrix 0 2)) (- (aref matrix 1 2)) (- (aref matrix 2 2))))
         (facing-robot-face
           (calculate-vector-face robot-negative-x-vector))
         (bottom-face
           (calculate-vector-face robot-negative-z-vector)))
    (list facing-robot-face bottom-face)))

(defun object-type-grasp->robot-grasp (robot-to-object-transform object-type-grasp)
  (destructuring-bind (grasp-axis grasp-axis-sign)
      (car (rassoc object-type-grasp *axis-index->face*))
    (let* ((grasp-axis-index
             (car (rassoc grasp-axis *index->axis*)))
           (matrix
             (cl-transforms:quaternion->matrix
              (cl-transforms:rotation robot-to-object-transform)))
           (object-grasp-vector
             (list (* grasp-axis-sign (aref matrix 0 grasp-axis-index))
                   (* grasp-axis-sign (aref matrix 1 grasp-axis-index))
                   (* grasp-axis-sign (aref matrix 2 grasp-axis-index))))
           (robot-grasp-face
             (calculate-vector-face object-grasp-vector)))
      ;; grasping object's front size would mean a back grasp for robot
      ;; otherwise all other grasps correspond to object sides
      (if (eq robot-grasp-face :front)
          :back
          (if (eq robot-grasp-face :back)
              :front
              robot-grasp-face)))))

(defun robot-grasp->object-type-grasp (robot-to-object-transform robot-grasp)
  (destructuring-bind (grasp-robot-axis grasp-robot-axis-sign)
      (car (rassoc robot-grasp *axis-index->face*))
    (when (eq grasp-robot-axis :x)
        (setf grasp-robot-axis-sign (* grasp-robot-axis-sign -1.0)))
    (let* ((grasp-robot-axis-index
             (car (rassoc grasp-robot-axis *index->axis*)))
           (matrix
             (cl-transforms:quaternion->matrix
              (cl-transforms:rotation
               (cram-tf:transform-stamped-inv robot-to-object-transform))))
           (robot-grasp-vector
             (list (* grasp-robot-axis-sign (aref matrix 0 grasp-robot-axis-index))
                   (* grasp-robot-axis-sign (aref matrix 1 grasp-robot-axis-index))
                   (* grasp-robot-axis-sign (aref matrix 2 grasp-robot-axis-index)))))
      (calculate-vector-face robot-grasp-vector))))

(defgeneric get-object-type-grasps (object-type facing-robot-face bottom-face
                                    rotationally-symmetric-p arm)
  (:documentation "Returns a lazy list of all possible grasps for `object-type'")
  (:method (object-type facing-robot-face bottom-face rotationally-symmetric-p arm)
    ;; (cut:lazy-list ((i 0))
    ;;   (when (< i 10) (cut:cont i (1+ i))))
    (cut:lazy-mapcar
     (lambda (bindings)
       (cut:var-value '?grasp bindings))
     (prolog:prolog `(obj-int:object-type-grasp ,object-type ?grasp)))))



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

