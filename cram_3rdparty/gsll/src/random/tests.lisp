;; Emulate the GSL tests for random distributions.
;; Liam Healy 2010-04-17 09:44:49EDT tests.lisp
;; Time-stamp: <2010-06-02 10:05:02EDT tests.lisp>
;;
;; Copyright 2010 Liam M. Healy
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

;; randist/test.c

(defconstant +gslt-BINS+ 100)
(defconstant +gslt-lower-limit+ -5.0d0)	; a
(defconstant +gslt-upper-limit+ 5.0d0)	; b
(defconstant +gslt-bin-size+		; dx
  (/ (- +gslt-upper-limit+ +gslt-lower-limit+) +gslt-BINS+))
(defconstant +initial-number-of-samples+ 100000) ; N

(defun distribution-bin-integral (pdf)
  "Compute the CDF for a bin from the PDF."
  (let ((cdf (make-array +gslt-BINS+ :element-type 'double-float)))
    (dotimes (i +gslt-BINS+ cdf)
      (let* ((xtrial (+ +gslt-lower-limit+ (* i +gslt-bin-size+)))
	     (x (if (< (abs xtrial) 1.0d-10) 0.0d0 xtrial)))
	(setf (aref cdf i)
	      (integration-qags
	       pdf x (+ x +gslt-bin-size+) 1.0d-16 1.0d-4 1000))))))

(defun bin-samples (count edge number-of-samples distribution &rest distribution-parameters)
  "Update the fixnum vectors count and edge with sample random values from the distribution,
   and return the mean."
  (let ((rng (make-random-number-generator +default-type+))
	(total 0.0d0))
    (dotimes (i number-of-samples)
      (let ((r (apply 'sample rng distribution distribution-parameters)))
	(incf total r)
	(when (< +gslt-lower-limit+ r +gslt-upper-limit+)
	  (multiple-value-bind (bin frac)
	      (floor (- r +gslt-lower-limit+) +gslt-bin-size+)
	    (if (zerop frac)
		(incf (aref edge bin))
		(incf (aref count bin)))))))
    ;; If the bin above is empty, its lower edge hits belong in the lower bin.
    (dotimes (i +gslt-BINS+)
      (when (and (< (1+ i) +gslt-BINS+) (zerop (aref count i)))
	(incf (aref count i) (aref edge (1+ i)))
	(setf (aref edge (1+ i)) 0)))
    (/ total number-of-samples)))

(defun limits-check (count number-of-samples cdf retry)
  "Returns nil if finite and within limits."
  (let ((retval nil))
    (dotimes (i +gslt-BINS+ retval)
      (setf retval
	    (if (not (finitep (aref cdf i)))
		t
		(if (not (zerop (aref cdf i)))
		    (let ((d (- (aref count i) (* number-of-samples (aref cdf i)))))
		      (and (> (/ d (sqrt (* number-of-samples (aref cdf i)))) 5)
			   (> d 2)))
		    (zerop (aref count i)))))
      (if (and retry retval) (return retval))
      ;; Printout out of warning will need
      ;; (x (+ +gslt-lower-limit+ (* i +gslt-bin-size+)))
      (if retval (warn "observed vs. expected")))))

(defvar *pdf-number-of-tries* 50)
(defun testpdf (pdf-function &rest distribution)
  "Test the probability density function in the same way that GSL does.
   If everything is functioning correctly, this will return T."
  (let ((count (make-array +gslt-BINS+ :element-type 'fixnum :initial-element 0))
	(edge (make-array +gslt-BINS+ :element-type 'fixnum :initial-element 0))
	(number-of-samples +initial-number-of-samples+)
	(cdf (distribution-bin-integral pdf-function)))
    (dotimes (i *pdf-number-of-tries*)
      (let ((mean (apply 'bin-samples count edge number-of-samples distribution)))
	(unless (finitep mean) (error "mean ~a is not finite" mean))
	(if (limits-check count number-of-samples cdf (< (1+ i) *pdf-number-of-tries*))
	    (if (plusp i) (setf number-of-samples (* 2 number-of-samples)))
	    (return t))))))

