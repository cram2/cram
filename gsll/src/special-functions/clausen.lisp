;; Clausen function
;; Liam Healy, Sat Mar 18 2006 - 23:18
;; Time-stamp: <2009-12-27 10:10:07EST clausen.lisp>
;;
;; Copyright 2006, 2007, 2008 Liam M. Healy
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

(defmfun clausen (x)
  "gsl_sf_clausen_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The Clausen integral Cl_2(x).")

(save-test clausen (clausen 2.5d0))
