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

(defgeneric get-object-type-grasps (object-type arm)
  (:documentation "Returns a lazy list of all possible grasps for `object-type' with given `arm'")
  (:method (object-type arm)
    #-solution-using-sbcl-API
    (remove-if
     #'null
     (mapcar (lambda (grasp-type)
               (when (find-method #'get-object-type-to-gripper-transform
                                  '()
                                  `((eql ,object-type) T (eql ,arm) (eql ,grasp-type))
                                  nil)
                 grasp-type))
             #+sbcl
             (remove-if
              #'null
              (remove-duplicates
               (mapcar
                (lambda (generic-method)
                  (let ((grasp-specializer (fourth (sb-pcl:method-specializers generic-method))))
                    (when (typep grasp-specializer 'sb-mop:eql-specializer)
                      (sb-mop:eql-specializer-object grasp-specializer))))
                (sb-pcl:generic-function-methods
                 #'get-object-type-to-gripper-transform))))
             #-sbcl
             (error "CRAM object manipulation code only works under SBCL...")
             #+solution-using-grasp-defining-macros-doesnt-work-for-custom-methods-like-env-manip
             *known-grasp-types*))

    #+solution-using-lazy-lists-and-Prolog
    ;; (cut:lazy-list ((i 0))
    ;;   (when (< i 10) (cut:cont i (1+ i))))
    (cut:lazy-mapcar
     (lambda (bindings)
       (cut:var-value '?grasp bindings))
     (prolog:prolog `(object-type-grasp ,object-type ?grasp)))))
