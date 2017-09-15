;; Input and output from/to Matlab/Octave
;; Liam Healy 2010-12-24 16:48:47EST matlab.lisp
;; Time-stamp: <2010-12-24 16:49:38EST matlab.lisp>

;; Copyright 2011 Liam M. Healy
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

(in-package :antik)

(export '(format-matlab-list))

(defun format-matlab-list
    (list &optional (stream *standard-output*) (number-per-line 3))
  "Format a matrix row to be read by Matlab."
  ;; "format long" in Matlab for full precision
  (loop for (this . rest) on list
     for count from 0
     do
     (format stream "~22e" this)
     (if rest
	 (if (and (plusp count)
		  (= (mod count number-per-line) (1- number-per-line)))
	     (format stream ", ... ~%")
	     (format stream ", "))
	 (format stream "; ... ~%"))))
