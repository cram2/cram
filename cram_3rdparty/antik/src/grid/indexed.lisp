;; Read an indexed set of data
;; Liam Healy 2011-12-30 10:10:58EST indexed.lisp
;; Time-stamp: <2013-12-25 19:37:12EST indexed.lisp>
;;
;; Copyright 2011, 2012, 2013 Liam M. Healy
;; Distributed under the terms of the GNU General Public License
;;
;; This program is free software: you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

(in-package :grid)

(export 'read-indexed-data)

;;; Data is presumed to be of the form: index-parameter-0 .... index-parameter-N datum,
;;; where index-parameter-i are integers related to the indices, or the indices themselves.
;;; 381   8     48      4.830

;;; Possible future improvements:
;;;  - accept make-simple-grid keyword arguments
;;;  - accept an existing grid, and fill it, rather than always creating a new one
;;;  - do not require maximum-index-parameters (hard unless input is read twice or saved when read)
;;;  - maybe make the input (file, stream, string) a kind of grid without random access, unless stream can be rewound; then you could just copy the grid you wanted.

(defun read-indexed-data
    (input maximum-index-parameters &rest args &key index-algorithm (default-value 0) (initial-element 0))
  "Read the stream of indexed data; each line contains one record.  The last column is the datum and the preceeding columns are either the indices, or inputs to index-algorithm, a function that should return the indices.  The grid is sized from maximum-indices, which are index parameters if index-algorithm is non-nil.  If a value read is not of the appropriate type (grid:*default-element-type*), then default-value is used instead, which should be of the appropriate type."
  (etypecase input
    (stream
     (flet ((mapip (ip)
	      (if index-algorithm
		  (funcall index-algorithm ip)
		  ip)))
       (let* ((dimensions (mapcar '1+ (mapip maximum-index-parameters)))
	      (grid (make-simple-grid :dimensions dimensions :initial-element initial-element))
	      (defval (coerce-value default-value (grid:element-type grid))))
	 (iter:iter (iter:for line :in-stream input :using #'read-line)
	   (let* ((ip-val
		    (with-input-from-string (lstr line)
		      (iter:iter (iter:for ip :in-stream lstr :using #'read)
			(iter:collecting ip))))
		  (ip-nocheck (butlast ip-val))
		  (indices (when (every #'<= ip-nocheck maximum-index-parameters)
			     (mapip ip-nocheck))))
	     (when indices
	       (setf (apply #'aref grid indices)
		     (let ((val (alexandria:lastcar ip-val)))
		       (if (typep val (grid:element-type grid))
			   val
			   defval))))))
	 grid)))
    (pathname
     (with-open-file (stream input)
       (apply 'read-indexed-data stream maximum-index-parameters args)))
    (string
     (with-input-from-string (stream input)
       (apply 'read-indexed-data stream maximum-index-parameters args)))))

#|
(defparameter *sample-indexed-data*
  "0     0     9.895
1     0     8.846
2     0     9.863
3     0     9.344
4     0     5.194
5     0     4.885
6     0    10.664
7     0     9.345
8     0     2.458
9     0     6.505
0     1     9.547
1     1     9.434
2     1     8.042
3     1     9.763
4     1     4.995
5     1     7.230
6     1    10.135
7     1     8.985
8     1     2.780
9     1     6.986")

(read-indexed-data *sample-indexed-data* '(9 1))
(read-indexed-data
 *sample-indexed-data*
 '(9 1)
 :index-algorithm
 (lambda (args) (list (first args) (* 2 (second args)))))
|#
