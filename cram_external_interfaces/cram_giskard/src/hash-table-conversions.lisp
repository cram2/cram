;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :giskard)

(defun alist->json-string (alist)
  (let ((stream (make-string-output-stream)))
    (yason:encode (cut:recursive-alist-hash-table alist :test #'equal)
                  stream)
    (get-output-stream-string stream)))

(defun hash-table->json-string (hash-table)
  (let ((stream (make-string-output-stream)))
    (yason:encode hash-table
                  stream)
    (get-output-stream-string stream)))


(defgeneric to-hash-table (object)
  (:documentation "Converts the object into a (nested) hash table."))

(defun make-point-hash-table (x y z)
  (alexandria:alist-hash-table
   `(("x" . ,x)
     ("y" . ,y)
     ("z" . ,z))
   :test #'equal))

(defun make-quaternion-hash-table (x y z w)
  (alexandria:alist-hash-table
   `(("x" . ,x)
     ("y" . ,y)
     ("z" . ,z)
     ("w" . ,w))
   :test #'equal))

(defun make-header-hash-table (stamp frame-id)
  (cut:recursive-alist-hash-table
   `(("stamp"
      . (("secs" . ,(floor stamp))
         ("nsecs" . ,(* (nth-value 1 (floor stamp)) 1.0d6))))
     ("frame_id" . ,frame-id)
     ("seq" . 0))
   :test #'equal))

(defmethod to-hash-table ((object cl-transforms:3d-vector))
  (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z))
      object
    (make-point-hash-table x y z)))

(defmethod to-hash-table ((object cl-transforms:quaternion))
  (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z)
               (w cl-transforms:w))
      object
    (make-quaternion-hash-table x y z w)))

(defmethod to-hash-table ((object cl-transforms:pose))
  (with-slots ((origin cl-transforms:origin)
               (orientation cl-transforms:orientation))
      object
    (cut:recursive-alist-hash-table
     `(("origin"
        . ,(to-hash-table origin))
       ("orientation"
        . ,(to-hash-table orientation)))
     :test #'equal)))

(defmethod to-hash-table ((object cl-transforms:transform))
  (with-slots ((origin cl-transforms:translation)
               (orientation cl-transforms:rotation))
      object
    (cut:recursive-alist-hash-table
     `(("translation"
        . ,(to-hash-table origin))
       ("rotation"
        . ,(to-hash-table orientation)))
     :test #'equal)))

(defmethod to-hash-table ((object cl-transforms-stamped:stamped))
  (with-slots ((stamp cl-transforms-stamped:stamp)
               (frame-id cl-transforms-stamped:frame-id))
      object
    (cut:recursive-alist-hash-table
     `(("header"
        . ,(make-header-hash-table stamp frame-id))))))

(defmethod to-hash-table ((object cl-transforms-stamped:vector-stamped))
  (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z)
               (stamp cl-transforms-stamped:stamp)
               (frame-id cl-transforms-stamped:frame-id))
      object
    (cut:recursive-alist-hash-table
     `(("header"
        . ,(make-header-hash-table stamp frame-id))
       ("vector"
        . ,(make-point-hash-table x y z)))
     :test #'equal)))

(defmethod to-hash-table ((object cl-transforms-stamped:point-stamped))
  (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z)
               (stamp cl-transforms-stamped:stamp)
               (frame-id cl-transforms-stamped:frame-id))
      object
    (cut:recursive-alist-hash-table
     `(("header"
        . ,(make-header-hash-table stamp frame-id))
       ("point"
        . ,(make-point-hash-table x y z)))
     :test #'equal)))

(defmethod to-hash-table ((object cl-transforms-stamped:pose-stamped))
  (with-slots ((origin cl-transforms:origin)
               (orientation cl-transforms:orientation)
               (stamp cl-transforms-stamped:stamp)
               (frame-id cl-transforms-stamped:frame-id))
      object
    (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z))
        origin
      (with-slots ((q1 cl-transforms:x) (q2 cl-transforms:y)
                   (q3 cl-transforms:z) (w cl-transforms:w))
          orientation
        (cut:recursive-alist-hash-table
         `(("header"
            . ,(make-header-hash-table stamp frame-id))
           ("pose"
            . (("position"
                . ,(make-point-hash-table x y z))
               ("orientation"
                . ,(make-quaternion-hash-table q1 q2 q3 w)))))
         :test #'equal)))))

(defmethod to-hash-table ((object cl-transforms-stamped:transform-stamped))
  (with-slots ((origin cl-transforms:translation)
               (orientation cl-transforms:rotation)
               (stamp cl-transforms-stamped:stamp)
               (frame-id cl-transforms-stamped:frame-id))
      object
    (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z))
        origin
      (with-slots ((q1 cl-transforms:x) (q2 cl-transforms:y)
                   (q3 cl-transforms:z) (w cl-transforms:w))
          orientation
        (cut:recursive-alist-hash-table
         `(("header"
            . ,(make-header-hash-table stamp frame-id))
           ("transform"
            . (("translation"
                . ,(make-point-hash-table x y z))
               ("rotation"
                . ,(make-quaternion-hash-table q1 q2 q3 w)))))
         :test #'equal)))))
