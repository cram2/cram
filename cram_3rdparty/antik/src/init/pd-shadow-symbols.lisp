;; Symbols defined in physical-dimension and GSLL that would conflict in Antik user packages
;; Liam Healy 2013-11-24 19:04:51EST pd-shadow-symbols.lisp
;; Time-stamp: <2013-11-24 19:22:58EST pd-shadow-symbols.lisp>

(in-package :antik)

(setf
 antik::*antik-user-shadow-symbols*
 (append antik::*antik-user-shadow-symbols*
	 ;; Where there is a symbol conflict between GSLL and other packages,
	 '(antik::polar-to-rectangular	; GSLL's doesn't use vectors
	   antik::rectangular-to-polar	; GSLL's doesn't use vectors
	   ;; antik:acceleration refers to the time derivative of velocity vs. object 'gsl:acceleration.
	   antik::acceleration
	   ;; No actual conflict due to different usage of symbols:
	   ;; antik:psi means "pounds per square inch" vs. function #'gsl:psi
	   ;; antik:knots means "nautical miles per hour" vs. function #'gsl:knots
	   antik::psi
	   antik::knots)))

(antik:make-user-package :antik-user)	; Add the new use package and shadow symbols to :antik-user
