;; Parameters for nf related to date-time.
;; Liam Healy 2013-03-24 18:24:56EDT format-output.lisp
;; Time-stamp: <2014-01-13 21:47:14EST format-output.lisp>

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

(define-parameter nf :timepoint-linear
  :value nil :type (or symbol list)
  :documentation
  "Convert timepoints to a linear scale if
  specified as a list of epoch time and unit.  For example
  \computer{'(*midnight-2000* :year)} will present time points as a
  real number of calendar years, including fractions, such as
  3.3223.  If the value is a symbol representing a unit, like :year,
  the epoch is taken as 0.")

(define-parameter nf :ignore-day-only
  :value nil :type t
  :documentation "Datime specified day-only will still format with time.")

(define-parameter nf :date-time-separator
  :value nil :type (or null character (member t))
  ;; Interpretation of nil, t determined by #'iso8601-string.
  :documentation
  "Character to place between date and time in ISO8601 datime output.
   If nil, use a space, if T, use #\T.")


