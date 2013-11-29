;; System grid-tests
;; Liam Healy 2009-11-28 17:22:21EST grid-tests.asd
;; Time-stamp: <2010-06-20 10:57:02EDT grid-tests.asd>
;;
;; Copyright 2009 Liam M. Healy
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

(asdf:defsystem grid-tests
  :description "Tests of grids"
  :author "Liam M. Healy"
  :license "GPL"
  ;;  :serial t
  :depends-on (:grid :lisp-unit)
  :components
  ((:module tests
	    :components
	    ((:file "augment")
	     (:file "compose")
	     (:file "element-functions")
	     (:file "map")
	     (:file "slice")
	     (:file "higher")))))
