;; Define a new readtable
;; Liam Healy 2014-09-28 22:27:28EDT readtable.lisp
;; Time-stamp: <2015-12-05 23:19:30EST readtable.lisp>

;; Copyright 2014, 2015 Liam M. Healy
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
(export '(set-reader set-reader-in-file))

(named-readtables:defreadtable :antik
  (:merge :standard)
  #-ccl
  (:macro-char #\# :dispatch))

(defun set-reader (&optional (readtable t) (default-float 'double-float))
  "If readtable is true, set the reader to enable the three Antik reader macros ``#_`` (physical dimension quantity), ``#d`` (date/time), ``#m`` (grid). Set the reader to interpret float literals without an exponent marker to be the type specified. If ``NIL``, leave as-is."
  (when readtable (named-readtables:in-readtable :antik))
  (when default-float
    (setf *read-default-float-format* default-float)
    #+allegro (tpl:setq-default *read-default-float-format* default-float)))

(defmacro set-reader-in-file (&optional (readtable t) (default-float 'double-float))
  "If readtable is true, set the reader to enable the three Antik reader macros ``#_`` (physical dimension quantity), ``#d`` (date/time), ``#m`` (grid). Set the reader to interpret float literals without an exponent marker to be the type specified. If ``NIL``, leave as-is. Use this form in Lisp source files."
  `(eval-when (:compile-toplevel :load-toplevel :execute)
     ,@(when readtable `((named-readtables:in-readtable :antik)))
     ,@(when default-float
	 `((setf *read-default-float-format* ',default-float)
	  #+allegro (tpl:setq-default *read-default-float-format* ,default-float)))))
