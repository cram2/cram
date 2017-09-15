;; Define package and reader for Antik
;; Liam Healy 2010-12-24 09:12:15EST package.lisp
;; Time-stamp: <2015-11-19 22:41:32EST package.lisp>

;; Copyright 2010, 2011, 2015 Liam M. Healy
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

(in-package :cl-user)

(defpackage #:antik
  ;; (:nicknames)
  (:shadow #:+ #:- #:* #:/
	   #:sin #:cos #:tan #:asin #:acos #:atan
	   #:sinh #:cosh #:tanh
	   #:sqrt #:expt #:log #:exp #:abs
	   #:plusp #:minusp #:zerop #:min #:max
	   #:=  #:>= #:> #:<= #:<
	   #:coerce #:floor #:round #:signum
	   #:incf #:decf
	   #:length #:time
	   ;; Iterate clause redefinitions
	   #:for #:sum #:summing #:multiply #:multiplying
	   #:minimize #:minimizing #:maximize #:maximizing
	   #+ccl #:@)
  (:export #:make-user-package #:physical-quantity #:physical-quantity-p #:@
	   #:for #:sum #:summing #:multiply #:multiplying
	   #:minimize #:minimizing #:maximize #:maximizing)
  (:use #:common-lisp #:iterate))

;; As a consolation for 'antik:* shadowing 'cl:* for things like
;; most recent primary value or declaration components:
(define-symbol-macro antik:@ cl:*)

(export (package-shadowing-symbols :antik) :antik)

(defpackage #:grid
    (:shadow #:aref)
    (:use #:common-lisp))

(defvar antik::*antik-user-use-packages* '(#:common-lisp #:iterate #:antik #:grid)
  "Packages to be used by default in antik-user packages.")

(defvar antik::*antik-user-shadow-symbols*
  (append (package-shadowing-symbols :antik)
	  (package-shadowing-symbols :grid)
	  #+ccl (list '#:terminate))	; CCL has conflict between iterate and ccl packages
  "Symbols to be shadowed by default in antik-user packages.")

(eval-when (:compile-toplevel :load-toplevel :execute)
  (defun antik:make-user-package (name &optional nicknames)
    "Make a package in which it is convenient to use Antik and related systems. If the package already exists, the use-packages and shadowing symbols are updated."
    (unless (find-package name) (make-package name :nicknames nicknames))
    (shadowing-import antik::*antik-user-shadow-symbols* name)
    (use-package antik::*antik-user-use-packages* name)))

(antik:make-user-package :antik-user)
