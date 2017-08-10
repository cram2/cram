;; GSL library version
;; Liam Healy 2016-08-06 10:37:52EDT gsl-version.lisp
;; Time-stamp: <2016-11-20 16:42:06CST gsl-version.lisp>
;;
;; Copyright 2016 Liam M. Healy
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

(in-package :gsl)

(eval-when (:compile-toplevel :load-toplevel :execute)

  (cffi:defcvar ("gsl_version" *gsl-version* :read-only t) :string
    "The version of the GSL library being used.")
  (export '*gsl-version*)

  (defun have-at-least-gsl-version (version-wanted)
    "The GSL version currently running is at least the version wanted, specified as (major minor)."
    (or (null version-wanted)
	(let ((version-loaded
		(mapcar 'read-from-string (split-sequence:split-sequence #\. *gsl-version*))))
	  ;; subminor version number ignored
	  (or (> (first version-loaded) (first version-wanted))
	      (and
	       (= (first version-loaded) (first version-wanted))
	       (>= (second version-loaded) (second version-wanted)))))))

  (when (have-at-least-gsl-version '(2 0))
    (pushnew :gsl2 *features*))
  )
