;; System definition
;; Liam Healy 2009-12-27 09:21:04EST grid.asd
;; Time-stamp: <2010-07-20 16:52:06EDT grid.asd>
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

(defpackage #:affi-asd
  (:use :cl :asdf))

(in-package :affi-asd)

(defsystem grid
  :description "Grid structured data or generalized arrays"
  :author "Liam M. Healy, Tamas K Papp"
  :license "GPL"
  ;;  :serial t
  :depends-on (alexandria)
  :components
  ;; composition, creation and mapping of arrays.
  ((:file "package")
   (:file "util" :depends-on ("package"))
   (:module affi			; Affine indexing
	    :components
	    ((:file "package")
	     (:file "utility" :depends-on ("package"))
	     (:file "affi" :depends-on ("utility"))
	     (:file "transformers" :depends-on ("affi"))))
   (:file "functions" :depends-on ("package" "util" affi))
   (:file "specification" :depends-on ("package" "util" affi))
   (:file "array" :depends-on ("package" "functions" affi))
   (:file "map" :depends-on ("package" affi))
   (:file "compose" :depends-on ("package" affi))
   (:file "slice" :depends-on ("package" affi))
   (:file "higher" :depends-on ("package" affi))
   (:file "copy" :depends-on ("package"))
   (:module tests
	    :components
	    ((:file "grids")))))

#+asdf-system-connections 
(asdf:defsystem-connection grid-iterate-extension
  :requires (grid iterate alexandria)
  :components ((:module
		"grid-iterate"
		:pathname ""
		:components ((:file "iterate")))))
