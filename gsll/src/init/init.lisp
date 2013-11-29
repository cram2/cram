;; Load GSL
;; Liam Healy Sat Mar  4 2006 - 18:53
;; Time-stamp: <2010-07-18 22:53:17EDT init.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2010 Liam M. Healy
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

(defpackage gsll
  (:nicknames :gsl)
  (:use :common-lisp :cffi)
  (:import-from
   :grid
   #:cl-array #:dimensions #:element-type
   #:foreign-array #:matrix #:dim0 #:dim1 #:^
   #:copy)
  (:shadowing-import-from :grid #:foreign-pointer)
  (:export
   #:cl-array #:dimensions #:element-type #:dim0 #:dim1
   #:copy))

(eval-when (:compile-toplevel :load-toplevel :execute)
  (defun gsl-config (arg)
    "A wrapper for tool `gsl-config'."
    (with-input-from-string
        (s (with-output-to-string (asdf::*verbose-out*)
             (asdf:run-shell-command "gsl-config ~s" arg)))
      (read-line s)
      (read-line s))))

(cffi:define-foreign-library libgslcblas
  (:unix (:or "libgslcblas.so.0" "libgslcblas.so"))
  (:darwin "libgslcblas.dylib")
  (:cygwin "cyggslcblas-0.dll")
  (t (:default "libgslcblas")))
   
(cffi:use-foreign-library libgslcblas)

;; When calling libgsl from emacs built for windows and slime, and
;; using clisp built for cygwin, we have to load lapack/cygblas.dll
;; before loading cyggsl-0.dll
#+(and clisp cygwin)
(cffi:load-foreign-library "/lib/lapack/cygblas.dll")

(cffi:define-foreign-library libgsl
  (:unix (:or "libgsl.so.0" "libgsl.so"))
  (:darwin "libgsl.dylib")
  (:cygwin "cyggsl-0.dll")
  (t (:default "libgsl")))
   
(cffi:use-foreign-library libgsl)
