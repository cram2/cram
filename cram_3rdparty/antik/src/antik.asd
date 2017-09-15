;; 
;; Liam Healy 2012-02-20 10:14:49EST physical-dimension.asd
;; Time-stamp: <2015-11-15 15:05:52EST antik.asd>

;; Copyright 2015 Liam M. Healy
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
;; 

(in-package :cl-user)

(asdf:defsystem #:antik
  :name "Antik"
  :description "A library of numerical mathematics."
  :author "Liam M. Healy"
  :license "GPL v3"
  :serial t
  :depends-on (#:physical-dimension #:gsll)
  :components
  ((:module mathematics
    :serial t
    :components
    ((:file "integers")
     (:file "trigonometry")))
   (:module optimize
    :components
    ((:file "one-dim")
     (:file "one-dim-pq")
     (:file "least-squares")))
   (:module linear-algebra
    :components
    ((:file "linear-algebra")
     (:file "linear-algebra-tests")))
   (:module sample
    :components
    ((:file "low-discrepancy-sequence"))))
  :perform (asdf:test-op 
	    (o c)
	    (uiop:symbol-call :lisp-unit '#:run-tests :all :antik)
	    (uiop:symbol-call :lisp-unit '#:run-tests :all :grid)))
