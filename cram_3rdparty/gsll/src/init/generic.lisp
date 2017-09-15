;; Define generic functions used across chapters
;; Liam Healy 2016-06-12 15:37:54EDT generic.lisp
;; Time-stamp: <2016-06-12 16:38:57EDT generic.lisp>

;; Copyright 2016 Liam M. Healy
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

(in-package :gsl)

(export '(parameter))

(defgeneric parameter (object parameter)
  (:documentation "Get the value of the GSL parameter from the GSL object."))

(defgeneric (setf parameter) (value object parameter)
  (:documentation "Set the value of the GSL parameter from the GSL object."))
