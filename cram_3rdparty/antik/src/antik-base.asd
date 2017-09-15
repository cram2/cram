;; Antik system definition
;; Liam Healy 2010-12-24 09:43:28EST antik.asd
;; Time-stamp: <2015-12-25 11:33:09EST antik-base.asd>

;; Copyright 2011, 2012, 2013, 2014, 2015 Liam M. Healy
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

;;; Antik is a system for doing scientific, engineering, and computational mathematics.  

(in-package :cl-user)

(asdf:defsystem #:antik-base
  :name "Antik"
  :description "Basic definitions providing a foundation for computational mathematics, science, and engineering."
  :author "Liam M. Healy"
  :license "GPL v3"
  :serial t
  :depends-on (#:iterate #:alexandria #:metabang-bind #:named-readtables #:cl-ppcre #:split-sequence #:lisp-unit)
  :components
  ((:module init
    :serial t
    :components
    ((:file "package")
     (:file "utility")
     (:file "conditions")
     (:file "object")
     (:file "iterate")
     (:file "intermediate")		; temporary
     (:file "generic")))
   (:module input-output
    :serial t
    :components
    ((:file "readtable")
     (:file "parameters")
     (:file "format-output")
     (:file "float")
     (:file "matlab")
     (:file "org-mode")
     (:file "write")
     (:file "read")))
   (:module date-time
    :serial t
    :components
    ((:file "format-output")
     (:file "iso8601")
     (:file "dtspec")
     (:file "formats")	
     (:file "timepoint")
     (:file "read-time")
     (:file "convert-timescale")))
   (:module grid
    :components
    ((:file "util")
     (:module affi			; Affine indexing
      :components
      ((:file "package")
       (:file "utility" :depends-on ("package"))
       (:file "affi" :depends-on ("utility"))
       (:file "transformers" :depends-on ("affi"))))
     (:file "functions" :depends-on ("util" affi))
     (:file "specification" :depends-on ("util" "functions" affi))
     (:file "array" :depends-on ("functions" affi))
     (:file "map" :depends-on (affi "functions"))
     (:file "compose" :depends-on ("functions" affi))
     (:file "slice" :depends-on (affi))
     (:file "norm-vector-product" :depends-on (affi "functions"))
     (:file "copy")
     (:file "iterate" :depends-on ("compose"))
     (:file "mathematics" :depends-on ("map"))
     (:file "index-functions")
     (:file "indexed" :depends-on ("specification"))
     (:file "format-output")
     (:file "parameters")
     (:module tests
      :components
      ((:file "augment")
       (:file "sequence")
       (:file "compose")
       (:file "map")
       (:file "slice")
       (:file "norm-vector-product")))))
   (:module tests
    :components
    ((:file "numbers")))))
