;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :cljlo-utils)

(defgeneric pose->jlo (pose))
(defgeneric jlo->pose (jlo))

(defmethod pose->jlo ((p cl-transforms:pose))
  (let ((p-matrix (cl-transforms:transform->matrix p)))
    (jlo:make-jlo :parent (jlo:make-jlo :name "/map")
                  :pose (make-array (array-total-size p-matrix)
                                    :displaced-to p-matrix
                                    :element-type (array-element-type p-matrix)))))

(defmethod pose->jlo ((p cl-transforms:pose))
  (pose->jlo (cl-transforms:make-transform
              (cl-transforms:origin p)
              (cl-transforms:orientation p))))

(defmethod pose->jlo ((p cl-transforms:transform))
  (let ((p-matrix (cl-transforms:transform->matrix p)))
    (jlo:make-jlo :parent (jlo:make-jlo :name "/map")
                  :pose (make-array (array-total-size p-matrix)
                                    :displaced-to p-matrix
                                    :element-type (array-element-type p-matrix)))))

(defmethod pose->jlo ((p cl-tf:stamped-transform))
  (let ((p-matrix (cl-transforms:transform->matrix p)))
    (jlo:make-jlo :parent (jlo:make-jlo :name (cl-tf:frame-id p))
                  :name (cl-tf:child-frame-id p)
                  :pose (make-array (array-total-size p-matrix)
                                    :displaced-to p-matrix
                                    :element-type (array-element-type p-matrix)))))

(defmethod pose->jlo ((p cl-tf:pose-stamped))
  (let ((p-matrix (cl-transforms:pose->matrix p)))
    (jlo:make-jlo :parent (jlo:make-jlo :name (cl-tf:frame-id p))
                  :pose (make-array (array-total-size p-matrix)
                                    :displaced-to p-matrix
                                    :element-type (array-element-type p-matrix)))))

(defmethod jlo->pose (jlo)
  (cl-transforms:matrix->transform
   (make-array '(4 4) :displaced-to (vision_msgs-msg:pose-val
                                     (jlo:partial-lo (jlo:frame-query
                                                      (jlo:make-jlo :id 1)
                                                      jlo))))))
(defun gaussian->jlo (name mean cov)
  (declare (type symbol name)
           (type cl-transforms:3d-vector
                 mean)
           (type (simple-array * (3 3)) cov))
  (flet ((pose-cov->jlo-cov (cov)
           (let ((result (make-array 36 :initial-element 0.0d0)))
             (dotimes (y 3)
               (dotimes (x 3)
                 (setf (aref result (+ (* y 6) x)) (aref cov y x))))
             result)))
    (let ((result
           (jlo:make-jlo :name (symbol-name name)
                         :pose (make-array
                                16 :initial-contents
                                `(1 0 0 ,(cl-transforms:x mean)
                                    0 1 0 ,(cl-transforms:y mean)
                                    0 0 1 ,(cl-transforms:z mean)
                                    0 0 0 1))
                         :cov (let ((cov (pose-cov->jlo-cov cov)))
                                (dotimes (i 2)
                                  (setf (aref cov (+ (* i 6) 2)) 0.0d0)
                                  (setf (aref cov (+ (* 2 6) i)) 0.0d0))
                                (setf (aref cov 14) 0.1d0)
                                cov)
                         :parent (typecase mean
                                   (cl-tf:stamped-transform
                                      (jlo:make-jlo :name (cl-tf:frame-id mean)))                            
                                   (cl-transforms:transform
                                      (jlo:make-jlo :name "/map"))))))
      (roslisp:ros-info (gaussian-to-jlo cop-perception) "Search space id: ~a" (jlo:id result))
      result)))
