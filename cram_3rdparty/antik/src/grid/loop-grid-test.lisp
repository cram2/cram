;; Testing setting and getting grid elemnts
;; Sumant Oemrawsingh 2012-03-31 15:37:41EDT loop-grid-test.lisp
;; Time-stamp: <2012-04-28 22:12:19EDT loop-grid-test.lisp>

(in-package :grid)

;;;;****************************************************************************
;;;; Get 
;;;;****************************************************************************

(defun read-grid-sum (grid)
  "Read all the elements of the grid and add them up."
  (iter:iterate (iter:for i vector-element-index grid)
    (iter:summing (grid:aref grid i))))

;;; Declare vector-double-float
(defun read-grid-sum-declared-vdf (grid)
  "Read all the elements of the grid and add them up."
  (declare (type vector-double-float grid))
  (iter:iterate (iter:for i vector-element-index grid)
    (iter:summing (grid:aref grid i))))

;;; Declare vector-complex-double-float
(defun read-grid-sum-declared-vcdf (grid)
  "Read all the elements of the grid and add them up."
  (declare (type vector-complex-double-float grid))
  (iter:iterate (iter:for i vector-element-index grid)
    (iter:summing (grid:aref grid i))))

;;;;****************************************************************************
;;;; Set 
;;;;****************************************************************************

;;;; loop-setf-test.lisp

(defun make-lists-list (rows columns)
 (loop for row below rows
    collect (loop for column below columns
               collect (* row column 1.0d0))))

(defun aref-loop-setf (rows columns)
 (let ((array (make-array (list rows columns) :element-type 'double-float)))
   (loop for row below rows
      do (loop for column below columns
            do (setf (aref array row column) (* row column 1.0d0))))
   array))

(defun grid-aref-loop-setf (rows columns)
 (let ((array (make-array (list rows columns) :element-type 'double-float)))
   (loop for row below rows
      do (loop for column below columns
            do (setf (grid:aref array row column) (* row column 1.0d0))))
   array))

(defun grid-aref-loop-setf-foreign (rows columns)
 (let ((array (grid:make-foreign-array 'double-float :dimensions (list rows columns))))
   (loop for row below rows
      do (loop for column below columns
            do (setf (grid:aref array row column) (* row column 1.0d0))))
   array))

#|

Results on my laptop are as follows.

CL-USER> (time (progn (aref-loop-setf 1024 10240) (values)))
Evaluation took:
 0.234 seconds of real time
 0.233963 seconds of total run time (0.192970 user, 0.040993 system)
 [ Run times consist of 0.057 seconds GC time, and 0.177 seconds non-GC time. ]
 100.00% CPU
 654,633,933 processor cycles
 251,658,256 bytes consed

; No value
CL-USER> (time (progn (grid-aref-loop-setf 1024 10240) (values)))
Evaluation took:
 3.094 seconds of real time
 3.084532 seconds of total run time (3.054536 user, 0.029996 system)
 [ Run times consist of 0.103 seconds GC time, and 2.982 seconds non-GC time. ]
 99.71% CPU
 8,632,811,366 processor cycles
 922,738,912 bytes consed

; No value
CL-USER> (time (progn (grid-aref-loop-setf-foreign 1024 10240) (values)))
Evaluation took:
 7.608 seconds of real time
 7.591846 seconds of total run time (7.552852 user, 0.038994 system)
 [ Run times consist of 0.506 seconds GC time, and 7.086 seconds non-GC time. ]
 99.79% CPU
 21,235,314,846 processor cycles
 4,026,564,512 bytes consed

; No value
CL-USER> (progn (defvar *initial-contents* (make-lists-list 1024 1024)) (values))

; No value
CL-USER> (time (progn (make-array (list 1024 1024) :element-type 'double-float :initial-contents *initial-contents*) (values)))
Evaluation took:
 0.000 seconds of real time
 0.000000 seconds of total run time (0.000000 user, 0.000000 system)
 100.00% CPU
 1,456 processor cycles
 0 bytes consed

; No value
CL-USER> (progn (defvar *initial-contents* (make-lists-list 1024 1024)) (values))

; No value
CL-USER> (time (progn (make-array (list 1024 1024) :element-type 'double-float :initial-contents *initial-contents*) (values)))
Evaluation took:
 0.000 seconds of real time
 0.000000 seconds of total run time (0.000000 user, 0.000000 system)
 100.00% CPU
 1,002 processor cycles
 0 bytes consed

; No value
CL-USER> (time (progn (grid:make-foreign-array 'double-float :dimensions (list 1024 1024) :initial-contents *initial-contents*) (values)))
Evaluation took:
 2934.652 seconds of real time
 2919.775127 seconds of total run time (2919.301199 user, 0.473928 system)
 [ Run times consist of 0.199 seconds GC time, and 2919.577 seconds non-GC time. ]
 99.49% CPU
 8,190,419,850,119 processor cycles
 368,726,592 bytes consed

; No value
CL-USER>

|#
