;; Regression test RAYLEIGH-TAIL for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST RAYLEIGH-TAIL
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST 1.0255032370386696d0 19.0764679267351d0
	  15.928966102255199d0 3.4422048899106383d0
	  17.131838333441106d0 12.071957529361999d0
	  3.112992916690818d0 7.749889301203328d0
	  11.145450138119857d0 7.825198187316554d0
	  7.476774681552917d0))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 10 COLLECT
	   (sample rng :rayleigh-tail :a 1.0d0 :sigma 10.0d0)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.10224317624874313d0)
   (MULTIPLE-VALUE-LIST
    (RAYLEIGH-TAIL-PDF 0.25d0 -2.0d0 2.0d0))))

