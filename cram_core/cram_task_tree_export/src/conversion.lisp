;;;
;;; Copyright (c) 2022, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
;;;
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

(in-package :tt-export)

(defparameter *add-type* NIL
  "Adds the TYPE to each entry in the formatted tree.")

(defmethod format-desig ((atom_ T))
  atom_)

(defmethod format-desig ((symbol_ symbol))
  symbol_)

(defmethod format-desig ((string_ string))
  string_)

(defmethod format-desig ((number_ number))
  number_)

(defmethod format-desig ((list_ cons))
  (mapcar #'format-desig list_))

(defmethod format-desig ((desig cram-designators:designator))
  (let ((desig-desc (cram-designators:description desig)))
    (mapcar (lambda (key-value)
              `(,@(when *add-type*
                    (type-of (second key-value)))
                (,(first key-value)
                 ,(format-desig (second key-value)))))
            desig-desc)))

;;;;;;;;;;;;;;;;;;;;;;;;;
;; BEGIN TF formatting ;;
(defun format-header (stamp frame-id)
  `(("stamp"
      . (("secs" . ,(floor stamp))
         ("nsecs" . ,(* (nth-value 1 (floor stamp)) 1.0d6))))
     ("frame_id" . ,frame-id)
    ("seq" . 0)))

(defun format-point (x y z)
  `(("x" . ,x)
    ("y" . ,y)
    ("z" . ,z)))

(defun format-quaternion (x y z w)
  `(("x" . ,x)
    ("y" . ,y)
    ("z" . ,z)
    ("w" . ,w)))

(defmethod format-desig ((object cl-transforms:pose))
  (with-slots ((origin cl-transforms:origin)
               (orientation cl-transforms:orientation))
      object
    `(("origin"
       . ,(format-desig origin))
      ("orientation"
       . ,(format-desig orientation)))))

(defmethod format-desig ((object cl-transforms:3d-vector))
  (with-slots ((x cl-transforms:x)
               (y cl-transforms:y)
               (z cl-transforms:z))
      object
    `(("x" . ,x)
      ("y" . ,y)
      ("z" . ,z))))

(defmethod format-desig ((object cl-transforms:quaternion))
  (with-slots ((x cl-transforms:x)
               (y cl-transforms:y)
               (z cl-transforms:z)
               (w cl-transforms:w))
      object
    `(("x" . ,x)
      ("y" . ,y)
      ("z" . ,z)
      ("w" . ,w))))

(defmethod format-desig ((object cl-transforms:transform))
  (with-slots ((origin cl-transforms:translation)
               (orientation cl-transforms:rotation))
      object
    `(("translation"
       . ,(format-desig origin))
      ("rotation"
       . ,(format-desig orientation)))))



(defmethod format-desig ((object cl-transforms-stamped:stamped))
  (with-slots ((stamp cl-transforms-stamped:stamp)
               (frame-id cl-transforms-stamped:frame-id))
      object
    `(("header"
       . ,(format-header stamp frame-id)))))

(defmethod format-desig ((object cl-transforms-stamped:vector-stamped))
  (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z)
               (stamp cl-transforms-stamped:stamp)
               (frame-id cl-transforms-stamped:frame-id))
      object
     `(("header"
        . ,(format-header stamp frame-id))
       ("vector"
        . ,(format-point x y z)))))

(defmethod format-desig ((object cl-transforms-stamped:point-stamped))
  (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z)
               (stamp cl-transforms-stamped:stamp)
               (frame-id cl-transforms-stamped:frame-id))
      object
    `(("header"
       . ,(format-header stamp frame-id))
      ("point"
       . ,(format-point x y z)))))

(defmethod format-desig ((object cl-transforms-stamped:pose-stamped))
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
        `(("header"
           . ,(format-header stamp frame-id))
          ("pose"
           . (("position"
               . ,(format-point x y z))
              ("orientation"
               . ,(format-quaternion q1 q2 q3 w)))))))))

(defmethod format-desig ((object cl-transforms-stamped:transform-stamped))
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
        `(("header"
           . ,(format-header stamp frame-id))
          ("transform"
           . (("translation"
               . ,(format-point x y z))
              ("rotation"
               . ,(format-quaternion q1 q2 q3 w)))))))))
;; END TF formatting ;;
;;;;;;;;;;;;;;;;;;;;;;;
