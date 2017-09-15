;; Dilogarithm
;; Liam Healy, Fri Mar 17 2006 - 18:44
;; Time-stamp: <2011-10-29 23:35:35EDT dilogarithm.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009, 2010, 2011 Liam M. Healy
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

;;; dilog merge complex and real
(export 'dilogarithm)
(defgeneric dilogarithm (x)
  (:documentation			; FDL
   "The dilogarithm."))

(defmfun dilogarithm ((x float))
  "gsl_sf_dilog_e" ((x :double) (ret (:pointer (:struct sf-result))))
  :definition :method)

(defmfun dilogarithm ((x complex))
  "gsl_sf_complex_dilog_e"
  (((abs x) :double)
   ((phase x) :double)
   (re (:pointer (:struct sf-result)))
   (im (:pointer (:struct sf-result))))
  :definition :method
  :return ((complex-with-error re im)))

(save-test dilogarithm
	   (dilogarithm 1.0d0)
	   (dilogarithm #c(0.0d0 1.0d0)))


