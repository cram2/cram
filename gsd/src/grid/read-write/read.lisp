;; Read from a table
;; Liam Healy 2010-07-06 12:32:55EDT read.lisp
;; Time-stamp: <2010-07-09 16:15:35EDT read.lisp>
;;
;; Copyright 2010 Liam M. Healy
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

;; This requires cl-ppcre and split-sequence.
;; This is the very, very beginning of the necessary definitions to
;; make a robust file parser.

;; Eventually it would be nice to read multi-line records.

(in-package :grid)

(export '(read-data-file))

(defun read-maybe-from-string (string)
  "Read from the string if possible."
  (ignore-errors (read-from-string string)))

(defun read-data-line (line &optional (separator :whitespace))
  "Read the data line and split into fields separated by either a
   separator or white space, designated with :whitespace."
  (let ((*read-default-float-format* 'double-float))
    (mapcar 'read-maybe-from-string
	    (if (eq separator :whitespace)
		(cl-utilities:split-sequence-if
		 'cl-ppcre::whitespacep line :remove-empty-subseqs t)
		(cl-utilities:split-sequence
		 separator line :remove-empty-subseqs t)))))

(defun read-data-stream
    (&optional (separator :whitespace) include-count (stream *standard-input*))
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
    (filename &optional (separator :whitespace) include-count discard-header)
  (with-open-file (stream filename :direction :input)
    (when discard-header (read-line stream))
    (read-data-stream separator include-count stream)))

