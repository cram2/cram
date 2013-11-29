;; Tdist distribution
;; Liam Healy, Sat Oct  7 2006 - 16:13
;; Time-stamp: <2010-05-24 20:01:35EDT tdist.lisp>
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

;;; /usr/include/gsl/gsl_randist.h

(defmfun sample
    ((generator random-number-generator) (type (eql :tdist)) &key nu)
  "gsl_ran_tdist"
  (((mpointer generator) :pointer) (nu :double))
  :definition :method
  :c-return :double
  :documentation			; FDL
  "A random variate from the Student t-distribution.  The
   distribution function is,
   p(x) dx = {\Gamma((\nu + 1)/2) \over \sqrt{\pi \nu} \Gamma(\nu/2)}
   (1 + x^2/\nu)^{-(\nu + 1)/2} dx
   for -\infty < x < +\infty.")

(defmfun tdist-pdf (x nu)
  "gsl_ran_tdist_pdf" ((x :double) (nu :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x) at x
   for a t-distribution with nu degrees of freedom, using the formula
   given in #'sample :tdist.")

(defmfun tdist-P (x nu)
  "gsl_cdf_tdist_P" ((x :double) (nu :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
  P(x) for the tdist distribution with nu degrees of freedom.")

(defmfun tdist-Q (x nu)
  "gsl_cdf_tdist_Q" ((x :double) (nu :double))
  :c-return :double
  :documentation			; FDL
  "The cumulative distribution functions
   Q(x) for the tdist distribution with nu degrees of freedom.")

(defmfun tdist-Pinv (P nu)
  "gsl_cdf_tdist_Pinv" ((P :double) (nu :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
   P(x) for the tdist distribution with nu degrees of freedom.")

(defmfun tdist-Qinv (Q nu)
  "gsl_cdf_tdist_Qinv" ((Q :double) (nu :double))
  :c-return :double
  :documentation			; FDL
  "The inverse cumulative distribution functions
   Q(x) for the tdist distribution with nu degrees of freedom.")

;;; Examples and unit test
(save-test tdist
	   (testpdf (lambda (r) (tdist-pdf r 1.75d0)) :tdist :nu 1.75d0)
	   (testpdf (lambda (r) (tdist-pdf r 12.75d0)) :tdist :nu 12.75d0)

	   ;; From test_tdist in cdf/test.c 
	   (tdist-P 0.0d0 1.0d0)
	   (tdist-P 1d-100 1.0d0)
	   (tdist-P 0.001d0 1.0d0)
	   (tdist-P 0.01d0 1.0d0)
	   (tdist-P 0.1d0 1.0d0)
	   (tdist-P 0.325d0 1.0d0)
	   (tdist-P 1.0d0 1.0d0)
	   (tdist-P 1.5d0 1.0d0)
	   (tdist-P 2.0d0 1.0d0)
	   (tdist-P 10.0d0 1.0d0)
	   (tdist-P 20.0d0 1.0d0)
	   (tdist-P 100.0d0 1.0d0)
	   (tdist-P 1000.0d0 1.0d0)
	   (tdist-P 10000.0d0 1.0d0)

	   (tdist-Q 0.0d0 1.0d0)
	   (tdist-Q 1d-100 1.0d0)
	   (tdist-Q 0.001d0 1.0d0)
	   (tdist-Q 0.01d0 1.0d0)
	   (tdist-Q 0.1d0 1.0d0)
	   (tdist-Q 0.325d0 1.0d0)
	   (tdist-Q 1.0d0 1.0d0)
	   (tdist-Q 1.5d0 1.0d0)
	   (tdist-Q 2.0d0 1.0d0)
	   (tdist-Q 10.0d0 1.0d0)
	   (tdist-Q 20.0d0 1.0d0)
	   (tdist-Q 100.0d0 1.0d0)
	   (tdist-Q 1000.0d0 1.0d0)
	   (tdist-Q 10000.0d0 1.0d0)

	   (tdist-P -1d-100 1.0d0)
	   (tdist-P -0.001d0 1.0d0)
	   (tdist-P -0.01d0 1.0d0)
	   (tdist-P -0.1d0 1.0d0)
	   (tdist-P -0.325d0 1.0d0)
	   (tdist-P -1.0d0 1.0d0)
	   (tdist-P -1.5d0 1.0d0)
	   (tdist-P -2.0d0 1.0d0)
	   (tdist-P -10.0d0 1.0d0)
	   (tdist-P -20.0d0 1.0d0)
	   (tdist-P -100.0d0 1.0d0)
	   (tdist-P -1000.0d0 1.0d0)
	   (tdist-P -10000.0d0 1.0d0)

	   (tdist-Q -1d-100 1.0d0)
	   (tdist-Q -0.001d0 1.0d0)
	   (tdist-Q -0.01d0 1.0d0)
	   (tdist-Q -0.1d0 1.0d0)
	   (tdist-Q -0.325d0 1.0d0)
	   (tdist-Q -1.0d0 1.0d0)
	   (tdist-Q -1.5d0 1.0d0)
	   (tdist-Q -2.0d0 1.0d0)
	   (tdist-Q -10.0d0 1.0d0)
	   (tdist-Q -20.0d0 1.0d0)
	   (tdist-Q -100.0d0 1.0d0)
	   (tdist-Q -1000.0d0 1.0d0)
	   (tdist-Q -10000.0d0 1.0d0)

	   (tdist-P 0.0d0 2.0d0)
	   (tdist-P 1d-100 2.0d0)
	   (tdist-P 0.001d0 2.0d0)
	   (tdist-P 0.01d0 2.0d0)
	   (tdist-P 0.1d0 2.0d0)
	   (tdist-P 0.325d0 2.0d0)
	   (tdist-P 1.0d0 2.0d0)
	   (tdist-P 1.5d0 2.0d0)
	   (tdist-P 2.0d0 2.0d0)
	   (tdist-P 10.0d0 2.0d0)
	   (tdist-P 20.0d0 2.0d0)
	   (tdist-P 100.0d0 2.0d0)
	   (tdist-P 1000.0d0 2.0d0)
	   (tdist-P 10000.0d0 2.0d0)

	   (tdist-Q 0.0d0 2.0d0)
	   (tdist-Q 1d-100 2.0d0)
	   (tdist-Q 0.001d0 2.0d0)
	   (tdist-Q 0.01d0 2.0d0)
	   (tdist-Q 0.1d0 2.0d0)
	   (tdist-Q 0.325d0 2.0d0)
	   (tdist-Q 1.0d0 2.0d0)
	   (tdist-Q 1.5d0 2.0d0)
	   (tdist-Q 2.0d0 2.0d0)
	   (tdist-Q 10.0d0 2.0d0)
	   (tdist-Q 20.0d0 2.0d0)
	   (tdist-Q 100.0d0 2.0d0)
	   (tdist-Q 1000.0d0 2.0d0)
	   (tdist-Q 10000.0d0 2.0d0)

	   (tdist-P -1.0d-100 2.0d0)
	   (tdist-P -0.001d0 2.0d0)
	   (tdist-P -0.01d0 2.0d0)
	   (tdist-P -0.1d0 2.0d0)
	   (tdist-P -0.325d0 2.0d0)
	   (tdist-P -1.0d0 2.0d0)
	   (tdist-P -1.5d0 2.0d0)
	   (tdist-P -2.0d0 2.0d0)
	   (tdist-P -10.0d0 2.0d0)
	   (tdist-P -20.0d0 2.0d0)
	   (tdist-P -100.0d0 2.0d0)
	   (tdist-P -1000.0d0 2.0d0)
	   (tdist-P -10000.0d0 2.0d0)

	   (tdist-Q -1.0d-100 2.0d0)
	   (tdist-Q -0.001d0 2.0d0)
	   (tdist-Q -0.01d0 2.0d0)
	   (tdist-Q -0.1d0 2.0d0)
	   (tdist-Q -0.325d0 2.0d0)
	   (tdist-Q -1.0d0 2.0d0)
	   (tdist-Q -1.5d0 2.0d0)
	   (tdist-Q -2.0d0 2.0d0)
	   (tdist-Q -10.0d0 2.0d0)
	   (tdist-Q -20.0d0 2.0d0)
	   (tdist-Q -100.0d0 2.0d0)
	   (tdist-Q -1000.0d0 2.0d0)
	   (tdist-Q -10000.0d0 2.0d0)

	   (tdist-P 0.0d0 300.0d0)
	   (tdist-P 1d-100 300.0d0)
	   (tdist-P 0.001d0 300.0d0)
	   (tdist-P 0.01d0 300.0d0)
	   (tdist-P 0.1d0 300.0d0)
	   (tdist-P 0.325d0 300.0d0)
	   (tdist-P 1.0d0 300.0d0)
	   (tdist-P 1.5d0 300.0d0)
	   (tdist-P 2.0d0 300.0d0)
	   (tdist-P 10.0d0 300.0d0)
	   (tdist-P 20.0d0 300.0d0)
	   (tdist-P 100.0d0 300.0d0)
	   (tdist-P 1000.0d0 300.0d0)
	   (tdist-P 10000.0d0 300.0d0)

	   (tdist-Q 0.0d0 300.0d0)
	   (tdist-Q 1d-100 300.0d0)
	   (tdist-Q 0.001d0 300.0d0)
	   (tdist-Q 0.01d0 300.0d0)
	   (tdist-Q 0.1d0 300.0d0)
	   (tdist-Q 0.325d0 300.0d0)
	   (tdist-Q 1.0d0 300.0d0)
	   (tdist-Q 1.5d0 300.0d0)
	   (tdist-Q 2.0d0 300.0d0)
	   (tdist-Q 10.0d0 300.0d0)
	   (tdist-Q 20.0d0 300.0d0)
	   (tdist-Q 100.0d0 300.0d0)
	   (tdist-Q 1000.0d0 300.0d0)
	   (tdist-Q 10000.0d0 300.0d0)

	   (tdist-P -1.0d-100 300.0d0)
	   (tdist-P -0.001d0 300.0d0)
	   (tdist-P -0.01d0 300.0d0)
	   (tdist-P -0.1d0 300.0d0)
	   (tdist-P -0.325d0 300.0d0)
	   (tdist-P -1.0d0 300.0d0)
	   (tdist-P -1.5d0 300.0d0)
	   (tdist-P -2.0d0 300.0d0)
	   (tdist-P -10.0d0 300.0d0)
	   (tdist-P -20.0d0 300.0d0)
	   (tdist-P -100.0d0 300.0d0)
	   (tdist-P -1000.0d0 300.0d0)
	   (tdist-P -10000.0d0 300.0d0)

	   (tdist-Q -1.0d-100 300.0d0)
	   (tdist-Q -0.001d0 300.0d0)
	   (tdist-Q -0.01d0 300.0d0)
	   (tdist-Q -0.1d0 300.0d0)
	   (tdist-Q -0.325d0 300.0d0)
	   (tdist-Q -1.0d0 300.0d0)
	   (tdist-Q -1.5d0 300.0d0)
	   (tdist-Q -2.0d0 300.0d0)
	   (tdist-Q -10.0d0 300.0d0)
	   (tdist-Q -20.0d0 300.0d0)
	   (tdist-Q -100.0d0 300.0d0)
	   (tdist-Q -1000.0d0 300.0d0)
	   (tdist-Q -10000.0d0 300.0d0)

	   (tdist-Pinv 0.5d0 1.0d0)
	   (tdist-Pinv 5.00318309780080559d-1 1.0d0) 
	   (tdist-Pinv 5.03182992764908255d-1 1.0d0) 
	   (tdist-Pinv 5.31725517430553569d-1 1.0d0) 
	   (tdist-Pinv 6.00023120032852123d-1 1.0d0) 
	   (tdist-Pinv 0.75000000000000000d0 1.0d0)
	   (tdist-Pinv 8.12832958189001183d-1 1.0d0)
	   (tdist-Pinv 8.52416382349566726d-1 1.0d0) 
	   (tdist-Pinv 9.68274482569446430d-1 1.0d0) 
	   (tdist-Pinv 9.84097748743823625d-1 1.0d0) 
	   (tdist-Pinv 9.96817007235091745d-1 1.0d0) 
	   (tdist-Pinv 9.99681690219919441d-1 1.0d0) 
	   (tdist-Pinv 9.99968169011487724d-1 1.0d0)

	   (tdist-Qinv 0.5d0 1.0d0)
	   (tdist-Qinv 4.99681690219919441d-1 1.0d0) 
	   (tdist-Qinv 4.96817007235091745d-1 1.0d0) 
	   (tdist-Qinv 4.68274482569446430d-1 1.0d0) 
	   (tdist-Qinv 3.99976879967147876d-1 1.0d0) 
	   (tdist-Qinv 2.5d-1 1.0d0)
	   (tdist-Qinv 1.87167041810998816d-1 1.0d0)
	   (tdist-Qinv 1.47583617650433274d-1 1.0d0) 
	   (tdist-Qinv 3.17255174305535695d-2 1.0d0) 
	   (tdist-Qinv 1.59022512561763752d-2 1.0d0) 
	   (tdist-Qinv 3.18299276490825515d-3 1.0d0) 
	   (tdist-Qinv 3.18309780080558939d-4 1.0d0) 
	   (tdist-Qinv 3.18309885122757724d-5 1.0d0) 

	   (tdist-Pinv 4.99681690219919441d-1 1.0d0) 
	   (tdist-Pinv 4.96817007235091744d-1 1.0d0) 
	   (tdist-Pinv 4.68274482569446430d-1 1.0d0) 
	   (tdist-Pinv 3.99976879967147876d-1 1.0d0) 
	   (tdist-Pinv 0.25d0 1.0d0)
	   (tdist-Pinv 1.87167041810998816d-1 1.0d0) 
	   (tdist-Pinv 1.47583617650433274d-1 1.0d0) 
	   (tdist-Pinv 3.17255174305535695d-2 1.0d0) 
	   (tdist-Pinv 1.59022512561763751d-2 1.0d0) 
	   (tdist-Pinv 3.18299276490825514d-3 1.0d0) 
	   (tdist-Pinv 3.18309780080558938d-4 1.0d0) 
	   (tdist-Pinv 3.18309885122757724d-5 1.0d0) 

	   (tdist-Qinv 5.00318309780080559d-1 1.0d0) 
	   (tdist-Qinv 5.03182992764908255d-1 1.0d0) 
	   (tdist-Qinv 5.31725517430553570d-1 1.0d0) 
	   (tdist-Qinv 6.00023120032852124d-1 1.0d0) 
	   (tdist-Qinv 7.5d-1 1.0d0)
	   (tdist-Qinv 8.12832958189001184d-1 1.0d0)
	   (tdist-Qinv 8.52416382349566726d-1 1.0d0) 
	   (tdist-Qinv 9.68274482569446430d-1 1.0d0) 
	   (tdist-Qinv 9.84097748743823625d-1 1.0d0) 
	   (tdist-Qinv 9.96817007235091745d-1 1.0d0) 
	   (tdist-Qinv 9.99681690219919441d-1 1.0d0) 
	   (tdist-Qinv 9.99968169011487724d-1 1.0d0) 

	   (tdist-Pinv 4.99646446697795041d-01 2.0d0)
	   (tdist-Pinv 4.96464554479100486d-01 2.0d0)
	   (tdist-Pinv 4.64732719207070087d-01 2.0d0)
	   (tdist-Pinv 3.88014227253126233d-01 2.0d0)
	   (tdist-Pinv 2.11324865405187118d-01 2.0d0)
	   (tdist-Pinv 1.36196562445500540d-01 2.0d0)
	   (tdist-Pinv 9.17517095361369836d-02 2.0d0)
	   (tdist-Pinv 4.92622851166284542d-03 2.0d0)
	   (tdist-Pinv 1.24533194618354849d-03 2.0d0)
	   (tdist-Pinv 4.99925012497812894d-05 2.0d0)
	   (tdist-Pinv 4.99999250001249998d-07 2.0d0)
	   (tdist-Pinv 4.99999992500000125d-09 2.0d0)                               

	   (tdist-Qinv 5.00353553302204959d-1 2.0d0) 
	   (tdist-Qinv 5.03535445520899514d-1 2.0d0) 
	   (tdist-Qinv 5.35267280792929913d-1 2.0d0) 
	   (tdist-Qinv 6.11985772746873767d-1 2.0d0) 
	   (tdist-Qinv 7.88675134594812882d-1 2.0d0) 
	   (tdist-Qinv 8.63803437554499460d-1 2.0d0) 
	   (tdist-Qinv 9.08248290463863016d-1 2.0d0) 
	   (tdist-Qinv 9.95073771488337155d-1 2.0d0) 
	   (tdist-Qinv 9.98754668053816452d-1 2.0d0) 
	   (tdist-Qinv 9.99950007498750219d-1 2.0d0) 
	   (tdist-Qinv 9.99999500000749999d-1 2.0d0) 
	   (tdist-Qinv 9.99999995000000075d-1 2.0d0) 
                               
	   (tdist-Pinv 5.00000000000000000d-01 300.0d0)
	   (tdist-Pinv 5.00398609900942949d-01 300.0d0)
	   (tdist-Pinv 5.03986033020559088d-01 300.0d0)
	   (tdist-Pinv 5.39794441177768194d-01 300.0d0)
	   (tdist-Pinv 6.27296201542523812d-01 300.0d0)
	   (tdist-Pinv 8.40941797784686861d-01 300.0d0)
	   (tdist-Pinv 9.32666983425369137d-01 300.0d0)
	   (tdist-Pinv 9.76799239508425455d-01 300.0d0)
	   (tdist-Pinv 1.00000000000000000d0 300.0d0)
                               
	   (tdist-Qinv 5.00000000000000000d-01 300.0d0)
	   (tdist-Qinv 4.99601390099057051d-1 300.0d0)
	   (tdist-Qinv 4.96013966979440912d-1 300.0d0)
	   (tdist-Qinv 4.60205558822231806d-1 300.0d0)
	   (tdist-Qinv 3.72703798457476188d-1 300.0d0)
	   (tdist-Qinv 1.59058202215313138d-1 300.0d0)
	   (tdist-Qinv 6.73330165746308628d-2 300.0d0)
	   (tdist-Qinv 2.32007604915745452d-2 300.0d0)
	   (tdist-Qinv 8.279313677d-21 300.0d0)
	   (tdist-Qinv 1.93159812815803978d-57 300.0d0)
	   (tdist-Qinv 1.02557519997736154d-232 300.0d0)
	   (tdist-Qinv 0.00000000000000000d0 300.0d0)

	   (tdist-Pinv 4.99601390099057051d-01 300.0d0)
	   (tdist-Pinv 4.96013966979440912d-01 300.0d0)
	   (tdist-Pinv 4.60205558822231806d-01 300.0d0)
	   (tdist-Pinv 3.72703798457476188d-01 300.0d0)
	   (tdist-Pinv 1.59058202215313138d-01 300.0d0)
	   (tdist-Pinv 6.73330165746308628d-02 300.0d0)
	   (tdist-Pinv 2.32007604915745452d-02 300.0d0)
	   (tdist-Pinv 8.279313675556272534d-21 300.0d0)
	   (tdist-Pinv 1.93159812815803978d-57 300.0d0)
	   (tdist-Pinv 1.02557519997736154d-232 300.0d0)
	   (tdist-Pinv 0.0 300.0d0)
                               
	   (tdist-Qinv 5.00398609900942949d-1 300.0d0)
	   (tdist-Qinv 5.03986033020559088d-1 300.0d0)
	   (tdist-Qinv 5.39794441177768194d-1 300.0d0)
	   (tdist-Qinv 6.27296201542523812d-1 300.0d0)
	   (tdist-Qinv 8.40941797784686862d-1 300.0d0)
	   (tdist-Qinv 9.32666983425369137d-1 300.0d0)
	   (tdist-Qinv 9.76799239508425455d-1 300.0d0)
	   (tdist-Qinv 1.000000000000000000d0 300.0d0))
