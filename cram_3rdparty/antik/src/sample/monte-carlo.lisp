;; Monte Carlo techniques
;; Liam Healy 2013-09-25 12:29:24EDT monte-carlo.lisp
;; Time-stamp: <2013-09-25 12:37:57EDT monte-carlo.lisp>

;; Copyright 2013 Liam M. Healy
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
(export 'sample-acceptance-rejection)

(defun sample-acceptance-rejection (pdf number-of-samples lower-bound upper-bound maximum-density rng)
  "Sample from the univariate PDF using the acceptance-rejection method.  Supremum distribution is interval-flat over domain, with value max.  Returns the list of samples and the total number of candidates."
  ;; See 1990 Flury, SLAC slides  id:e48523a5-a0d5-4aa2-899b-14f8bd86bb32
  (iter (with number-found = 0)
    (for count from 0)
    (let* ((y (gsl:sample rng :flat :a lower-bound :b upper-bound)) ; sample from constant supremum distribution "g(x)"
	   (u (gsl:sample rng :flat :a 0.0 :b maximum-density))
	   (pdf-val (funcall pdf y)))
      (when (<= u pdf-val)
	(incf number-found)
	(collect y into samples))
      (until (>= number-found number-of-samples))
      (finally (return (values samples count))))))
