;; Regression test LINEAR-LEAST-SQUARES for GSLL, automatically generated
;;
;; Copyright 2009 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST LINEAR-LEAST-SQUARES
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST -106.59999999999998d0 0.05999999999999999d0
                              39601.99999999999d0 -19.9d0
                              0.009999999999999998d0 0.8d0)
                        (MULTIPLE-VALUE-LIST
                         (LINEAR-LEAST-SQUARES-UNIVARIATE-EXAMPLE NIL)))
                       (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
                        (LIST 1.1824632487186013d0 0.1845715900137661d0
                              1.3031038153096723d0 16.006137036426168d0)
                        (MULTIPLE-VALUE-LIST
                         (LINEAR-LEAST-SQUARES-MULTIVARIATE-EXAMPLE
                          (MV-LINEAR-LEAST-SQUARES-DATA) NIL))))

