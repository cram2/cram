;; System grid-tests
;; Liam Healy 2009-11-28 17:22:21EST grid-tests.asd
;; Time-stamp: <2010-06-20 19:04:25EDT foreign-array-tests.asd>
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

(asdf:defsystem foreign-array-tests
  :description "Tests of foreign arrays"
  :author "Liam M. Healy"
  :license "GPL"
  ;;  :serial t
  :depends-on (:grid-tests :foreign-array :lisp-unit)
  :components
  ((:module tests
	    :components
	    ((:file "compose")))))
