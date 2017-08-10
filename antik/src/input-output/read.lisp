;; Read from a table
;; Liam Healy 2010-07-06 12:32:55EDT read.lisp
;; Time-stamp: <2014-11-27 16:40:23EST read.lisp>
;;
;; Copyright 2010, 2011 Liam M. Healy
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


;; This is a rudimentary beginning of an informat text file data read.
;; Specify the separator and read data from a file or stream.
;; Requires cl-ppcre and split-sequence.

;; Eventually it would be nice to read multi-line records, among other things.

(in-package :grid)

(export '(read-data-file read-data-stream read-vector-from-string))

(defun read-maybe-from-string (string)
  "Read from the string if possible."
  (ignore-errors (read-from-string string)))

(defun read-data-line (line &optional (separator :whitespace))
  "Read the data line and split into fields separated by either a
   separator or white space, designated with :whitespace."
  (let ((*read-default-float-format* 'double-float))
    (mapcar 'read-maybe-from-string
	    (if (eq separator :whitespace)
		(split-sequence:split-sequence-if
		 'cl-ppcre::whitespacep line :remove-empty-subseqs t)
		(split-sequence:split-sequence
		 separator line :remove-empty-subseqs t)))))

(defun read-data-stream
    (&key (separator :whitespace) include-count (stream *standard-input*))
  "Read the data from the stream and collect a list of lists of values
   from columns separated by whitespace.  If include-count is a
   number, line count starting at that number will be prepended as the
   first field."
  (loop for line = (read-line stream nil nil)
     for count from (or include-count 0)
     while line 
     collect (let ((ans (read-data-line line separator)))
	       (if include-count (cons count ans) ans))))

(defun read-data-file
    (filename &key (separator :whitespace) include-count discard-header)
  "Read the data in the file into a list.  If include-count is true, one column will be a serial count of the line of input data.  If discard-header is not nil, discard header line(s); if a number greater than 1, discard that many lines."
  (with-open-file (stream filename :direction :input)
    (when discard-header
      (dotimes (i (if (numberp discard-header) discard-header 1))
	;;(declare (ignore i))
	(read-line stream)))
    (read-data-stream :separator separator :include-count include-count :stream stream)))

;;; It should be possible to remove the count argument here.
(defun read-vector-from-string (string count &optional (start 0) end)
  "Read the count element vector from a string."
  (with-input-from-string (stream string :start start :end end)
    (make-simple-grid
     :initial-contents (loop repeat count collect (read stream)))))

#|
;;;;; New approach, based on Fare-CSV reader

(defun read-grid-csv (string)
    (grid:make-simple-grid
     :initial-contents
     (antik::map-leaf
      'read-from-string
      (with-input-from-string (s string) (fare-csv:read-csv-stream s)))))

(read-grid-csv
"1.0, 1.0
2.0, 1.414213562373095
3.0, 1.732050807568877
4.0, 2.0
5.0, 2.2360679774997
")
|#
