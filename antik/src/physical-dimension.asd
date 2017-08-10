;; Physical dimension system definition
;; Liam Healy 2012-02-20 10:14:49EST physical-dimension.asd
;; Time-stamp: <2015-11-25 13:45:08EST physical-dimension.asd>

;; Copyright 2012, 2013, 2014, 2015 Liam M. Healy
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

(asdf:defsystem #:physical-dimension
  :name "Physical Dimension"
  :description "A library to provide computations with physical dimension, i.e. units."
  :author "Liam M. Healy"
  :license "GPL v3"
  :serial t
  :depends-on (#:foreign-array #:fare-utils #:trivial-utf-8)
  :components
  ((:module init
    :serial t
    :components
	    ((:file "pd-shadow-symbols")))
   (:module physical-quantities
    :serial t
    :components
	    ((:file "format-output")
	     (:file "units")
	     (:file "scalar")
	     (:file "funcall")
	     (:file "unit-definitions")
	     (:file "degree-symbol")
	     (:file "sysunit-definitions")
	     (:file "physical-quantities")
	     (:file "undimension")
	     (:file "arithmetic")
	     (:file "angle")
	     (:file "angle-component")
	     (:file "state")
	     (:file "grid")))
   (:module cartesian
    :serial t
    :components
	    ((:file "cartesian")
	     (:file "polar")
	     (:file "rotation")))
   (:module date-time
    :serial t
    :components
	    ((:file "time-interval")
	     (:file "relative-time")
	     (:file "linear-timepoint")	
	     (:file "dtmath")
	     (:file "ut1")
	     (:file "time-interval-tests")))
   (:module tests
    :serial t
    :components
	    ((:file "physical-quantities")
	     (:file "physical-quantities-grid")
	     (:file "format-grid")))))
