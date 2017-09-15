;; Scientific data, mostly retrieved from the net
;; Liam Healy 2015-03-08 22:39:45EDT science-data.asd
;; Time-stamp: <2015-03-14 10:36:56EDT science-data.asd>

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

(in-package :cl-user)

(asdf:defsystem #:science-data
  :name "Science data"
  :description "Numerical data science and engineering data."
  :author "Liam M. Healy"
  :license "GPL v3"
  :serial t
  :depends-on (#:physical-dimension #:drakma)
  :components
  ((:module date-time
    :components
    ((:file "eop")))))
