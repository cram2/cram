;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :pr2-reachability-costmap)

(defgeneric store (stream object))
(defgeneric restore (stream type))
(defgeneric storage-type (object))

(defmethod store (stream (object integer))
  (dotimes (i 8)
    (write-byte (ldb (byte 8 (* i 8)) object) stream)))

(defmethod restore (stream (type (eql 'integer)))
  (let ((result 0))
    (dotimes (i 8 result)
      (setf (ldb (byte 8 (* i 8)) result) (read-byte stream)))
    (if (< result #x8000000000000000)
        result
        (- result #x10000000000000000))))

(defmethod storage-type ((object integer))
  'integer)

(defmethod store (stream (object string))
  (store stream (length object))
  (dotimes (i (length object))
    (write-byte (char-code (aref object i)) stream)))

(defmethod restore (stream (type (eql 'string)))
  (let* ((length (restore stream 'integer))
         (result (make-array length :element-type 'unsigned-byte)))
    (read-sequence result stream)
    (map 'string #'code-char result)))

(defmethod storage-type ((object string))
  'string)

(defmethod store (stream (object symbol))
  (store stream (package-name (symbol-package object)))
  (store stream (symbol-name object)))

(defmethod restore (stream (type (eql 'symbol)))
  (let ((package-name (restore stream 'string))
        (symbol-name (restore stream 'string)))
    (intern symbol-name (find-package package-name))))

(defmethod storage-type ((object symbol))
  'symbol)

(defmethod store (stream (object single-float))
  (store stream (float object 0.0d0)))

(defmethod restore (stream (type (eql 'single-float)))
  (float (restore stream 'double-float) 0.0))

(defmethod storage-type ((object single-float))
  'single-float)

(defmethod store (stream (object double-float))
  (multiple-value-bind (base exponent sign)
      (integer-decode-float object)
    (store stream (* base sign))
    (store stream exponent)))

(defmethod restore (stream (type (eql 'double-float)))
  (let ((base (restore stream 'integer))
        (exponent (restore stream 'integer)))
    (float 
     (if (eql base 0) 0 (* base (expt 2 exponent)))
     0.0d0)))

(defmethod storage-type ((object double-float))
  'double-float)

(defmethod store (stream (object cl-transforms:3d-vector))
  (store stream (cl-transforms:x object))
  (store stream (cl-transforms:y object))
  (store stream (cl-transforms:z object)))

(defmethod restore (stream (type (eql 'cl-transforms:3d-vector)))
  (cl-transforms:make-3d-vector
   (restore stream 'double-float)
   (restore stream 'double-float)
   (restore stream 'double-float)))

(defmethod storage-type ((object cl-transforms:3d-vector))
  'cl-transforms:3d-vector)

(defmethod store (stream (object cl-transforms:quaternion))
  (store stream (cl-transforms:x object))
  (store stream (cl-transforms:y object))
  (store stream (cl-transforms:z object))
  (store stream (cl-transforms:w object)))

(defmethod restore (stream (type (eql 'cl-transforms:quaternion)))
  (cl-transforms:make-quaternion
   (restore stream 'double-float)
   (restore stream 'double-float)
   (restore stream 'double-float)
   (restore stream 'double-float)))

(defmethod storage-type ((object cl-transforms:quaternion))
  'cl-transforms:quaternion)

(defmethod store (stream (object list))
  (store stream (list-length object))
  (dolist (element object)
    (store stream (storage-type element))
    (store stream element)))

(defmethod restore (stream (type (eql 'list)))
  (let ((length (restore stream 'integer)))
    (loop repeat length
          collecting (let ((type (restore stream 'symbol)))
                       (restore stream type)))))

(defmethod storage-type ((object list))
  'list)

(defmethod store (stream (data array))
  (let ((number-of-dimensions (list-length (array-dimensions data))))
    (store stream number-of-dimensions)
    (dotimes (i number-of-dimensions)
      (store stream (array-dimension data i)))
    (store-array
     stream data (array-element-type data))))

(defgeneric store-array (stream data element-type)
  (:method (stream data element-type)
    (let* ((flat-array (make-array
                        (apply #'* (array-dimensions data))
                        :displaced-to data
                        :element-type element-type))
           (array-element-type (cond ((> (array-dimension flat-array 0) 0)
                                      (storage-type (aref flat-array 0)))
                                     (t element-type))))
      (store stream array-element-type)
      (loop for element across flat-array do
        (store stream element))))
  
  (:method (stream data (element-type (eql 'bit)))
    (let* ((bits (car (last (array-dimensions data))))
           (bytes (multiple-value-bind (result rest) (truncate bits 8)
                    (if (eql rest 0) result (+ result 1))))
           (element-dimension (apply #'* (butlast (array-dimensions data))))
           (flat-array (make-array
                        (cons (if (eql element-dimension 0) 1 element-dimension)
                              (last (array-dimensions data)))
                        :element-type 'bit
                        :displaced-to data)))
      (store stream element-type)
      (dotimes (i (array-dimension flat-array 0))
        (let ((value 0))
          (dotimes (bit bits)
            (setf value (logior value (ash (aref flat-array i bit) bit))))
          (dotimes (byte bytes)
            (write-byte (ldb (byte 8 (* 8 byte)) value) stream)))))))

(defmethod restore (stream (data (eql 'array)))
  (let* ((number-of-dimensions (restore stream 'integer))
         (dimensions (loop repeat number-of-dimensions
                           collecting (restore stream 'integer)))
         (element-type (restore stream 'symbol))
         (data (make-array dimensions :element-type element-type)))
    (restore-array stream data element-type)
    data))

(defgeneric restore-array (stream data element-type)
  (:method (stream data element-type)
    (let ((flat-array (make-array
                       (apply #'* (array-dimensions data))
                       :displaced-to data
                       :element-type element-type)))
      (loop for i from 0 below (array-dimension flat-array 0) do
        (setf (aref flat-array i) (restore stream element-type)))))

  (:method (stream data (element-type (eql 'bit)))
    (let* ((bits (car (last (array-dimensions data))))
           (bytes (multiple-value-bind (result rest) (truncate bits 8)
                    (if (eql rest 0) result (+ result 1))))
           (element-dimension (apply #'* (butlast (array-dimensions data))))
           (flat-array (make-array
                        (cons (if (eql element-dimension 0) 1 element-dimension)
                              (last (array-dimensions data)))
                        :element-type 'bit
                        :displaced-to data)))
      (dotimes (i (array-dimension flat-array 0))
        (let ((value 0))
          (dotimes (byte-index bytes)
            (let ((byte (read-byte stream)))
              (setf (ldb (byte 8 (* 8 byte-index)) value) byte)))
          (dotimes (bit bits)
            (if (eql (logand value (ash 1 bit)) 0)
                (setf (aref flat-array i bit) 0)
                (setf (aref flat-array i bit) 1))))))))
