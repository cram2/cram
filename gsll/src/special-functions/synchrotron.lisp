;; Synchrotron functions
;; Liam Healy, Mon May  1 2006 - 22:29
;; Time-stamp: <2009-12-27 10:09:59EST synchrotron.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009 Liam M. Healy
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

(defmfun synchrotron-1 (x)
  "gsl_sf_synchrotron_1_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The first synchrotron function x \int_x^\infty dt K_{5/3}(t)} for x >= 0.")

(defmfun synchrotron-2 (x)
  "gsl_sf_synchrotron_2_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The second synchrotron function x K_{2/3}(x)} for x >= 0.")

;;;;****************************************************************************
;;;; Examples and unit test
;;;;****************************************************************************

(save-test synchrotron
  (synchrotron-1 4.0d0)
  (synchrotron-2 4.0d0))

