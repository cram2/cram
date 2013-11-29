;; System definition
;; Liam Healy 2009-12-27 09:22:50EST foreign-array.asd
;; Time-stamp: <2010-07-07 09:58:54EDT foreign-array.asd>
;;
;; Copyright 2009, 2010 Liam M. Healy
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

(when (asdf:find-system :fsbv nil)
  (pushnew :fsbv *features*))

#+(or allegro ccl ecl lispworks sbcl)
(when (asdf:find-system :static-vectors nil)
  (pushnew :static-vectors *features*))

(asdf:defsystem foreign-array
  :name "foreign-array"
  :description "Foreign arrays as grid structured data"
  :author "Liam M. Healy"
  :license "GPL v3"
  :depends-on (grid cffi trivial-garbage alexandria
		    split-sequence
		    #+static-vectors static-vectors
		    #+fsbv fsbv)
  :components
  ((:file "types")
   (:file "complex-types" :depends-on ("types"))
   (:file "element-types")
   (:file "symbol-type")
   (:file "number-conversion"
	  :depends-on ("complex-types" "symbol-type"))
   (:file "foreign-array" :depends-on ("types" "element-types"))
   (:file "methods")
   (:file "subclass" :depends-on ("element-types"))
   (:file "vector-matrix" :depends-on ("element-types" "subclass"))))
