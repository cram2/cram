;; Regression test SPHERICAL-VECTOR for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST SPHERICAL-VECTOR
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST -0.617745613497854d0 -0.7863779988047479d0
	  0.993748310886084d0 0.1116436053298841d0
	  -0.9458104280982743d0 0.3247193158722761d0
	  0.45726622946182216d0 0.8893298574734622d0
	  -0.46325616159849964d0 -0.8862244234622655d0))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 4 APPEND
	   (MULTIPLE-VALUE-LIST (sample rng :direction-2d))))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST 0.9999986835208556d0 -0.0016226387631051197d0
	  0.5203010106077766d0 0.8539829379797504d0
	  -0.2035120531038584d0 0.9790724407527016d0
	  0.9454753227485545d0 -0.3256937427607672d0
	  0.11500033916619544d0 0.9933654523848008d0))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 4 APPEND
	   (MULTIPLE-VALUE-LIST (sample rng :direction-2d-trig-method))))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST -0.09129925750445994d0 0.18782185357162273d0
	  0.977950610665004d0 -0.9051182961559773d0
	  -0.050683764485791594d0 -0.4221279734645046d0
	  0.13993766535985133d0 0.8385462620524484d0
	  -0.526552576872909d0))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 2 APPEND
	   (MULTIPLE-VALUE-LIST (sample rng :direction-3d)))))))

