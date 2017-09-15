;; Parameters for nf related to physical quanitites.
;; Liam Healy 2013-03-24 18:24:56EDT format-output.lisp
;; Time-stamp: <2014-01-13 22:57:23EST format-output.lisp>

;; Copyright 2013, 2014 Liam M. Healy
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

(in-package :antik)

(define-parameter nf :system-of-units
  :value nil :type list
  :documentation
  "The system of units used for output. If NIL, don't print units. Standard systems of units may be altered with #'make-sysunits.")

(define-parameter nf :degrees
  :value :decimal :type (member :decimal :dms)
  :documentation
  "Whether to format degrees as decimal degees or degrees, minutes, seconds.")


