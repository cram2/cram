;; Physical constants in the CGSM system
;; Liam Healy 2009-05-25 17:01:32EDT cgsm.lisp
;; Time-stamp: <2010-05-23 11:35:35EDT cgsm.lisp>
;;
;; Copyright 2009, 2010 Liam M. Healy
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

#+linux
(define "_GNU_SOURCE")

;;; When installed through Mac Ports, GSL .h files will be found
;;; in /opt/local/include.
#+darwin
(cc-flags #.(gsl-config "--cflags"))

(include "gsl/gsl_const_cgsm.h")

(constant (+cgsm-speed-of-light+ "GSL_CONST_CGSM_SPEED_OF_LIGHT") :type double-float)
(constant (+cgsm-gravitational-constant+ "GSL_CONST_CGSM_GRAVITATIONAL_CONSTANT")
	  :type double-float)
(constant (+cgsm-plancks-constant-h+ GSL_CONST_CGSM_PLANCKS_CONSTANT_H)
	  :type double-float)
(constant (+cgsm-plancks-constant-hbar+ GSL_CONST_CGSM_PLANCKS_CONSTANT_HBAR)
	  :type double-float)
(constant (+cgsm-astronomical-unit+ GSL_CONST_CGSM_ASTRONOMICAL_UNIT) :type double-float)
(constant (+cgsm-light-year+ GSL_CONST_CGSM_LIGHT_YEAR) :type double-float)
(constant (+cgsm-parsec+ GSL_CONST_CGSM_PARSEC) :type double-float)
(constant (+cgsm-grav-accel+ GSL_CONST_CGSM_GRAV_ACCEL) :type double-float)
(constant (+cgsm-electron-volt+ GSL_CONST_CGSM_ELECTRON_VOLT) :type double-float)
(constant (+cgsm-mass-electron+ GSL_CONST_CGSM_MASS_ELECTRON) :type double-float)
(constant (+cgsm-mass-muon+ GSL_CONST_CGSM_MASS_MUON) :type double-float)
(constant (+cgsm-mass-proton+ GSL_CONST_CGSM_MASS_PROTON) :type double-float)
(constant (+cgsm-mass-neutron+ GSL_CONST_CGSM_MASS_NEUTRON) :type double-float)
(constant (+cgsm-rydberg+ GSL_CONST_CGSM_RYDBERG) :type double-float)
(constant (+cgsm-boltzmann+ GSL_CONST_CGSM_BOLTZMANN) :type double-float)
(constant (+cgsm-bohr-magneton+ GSL_CONST_CGSM_BOHR_MAGNETON) :type double-float)
(constant (+cgsm-nuclear-magneton+ GSL_CONST_CGSM_NUCLEAR_MAGNETON) :type double-float)
(constant (+cgsm-electron-magnetic-moment+ GSL_CONST_CGSM_ELECTRON_MAGNETIC_MOMENT)
	  :type double-float)
(constant (+cgsm-proton-magnetic-moment+ GSL_CONST_CGSM_PROTON_MAGNETIC_MOMENT)
	  :type double-float)
(constant (+cgsm-molar-gas+ GSL_CONST_CGSM_MOLAR_GAS) :type double-float)
(constant (+cgsm-standard-gas-volume+ GSL_CONST_CGSM_STANDARD_GAS_VOLUME) :type double-float)
(constant (+cgsm-minute+ GSL_CONST_CGSM_MINUTE) :type double-float)
(constant (+cgsm-hour+ GSL_CONST_CGSM_HOUR) :type double-float)
(constant (+cgsm-day+ GSL_CONST_CGSM_DAY) :type double-float)
(constant (+cgsm-week+ GSL_CONST_CGSM_WEEK) :type double-float)
(constant (+cgsm-inch+ GSL_CONST_CGSM_INCH) :type double-float)
(constant (+cgsm-foot+ GSL_CONST_CGSM_FOOT) :type double-float)
(constant (+cgsm-yard+ GSL_CONST_CGSM_YARD) :type double-float)
(constant (+cgsm-mile+ GSL_CONST_CGSM_MILE) :type double-float)
(constant (+cgsm-nautical-mile+ GSL_CONST_CGSM_NAUTICAL_MILE) :type double-float)
(constant (+cgsm-fathom+ GSL_CONST_CGSM_FATHOM) :type double-float)
(constant (+cgsm-mil+ GSL_CONST_CGSM_MIL) :type double-float)
(constant (+cgsm-point+ GSL_CONST_CGSM_POINT) :type double-float)
(constant (+cgsm-texpoint+ GSL_CONST_CGSM_TEXPOINT) :type double-float)
(constant (+cgsm-micron+ GSL_CONST_CGSM_MICRON) :type double-float)
(constant (+cgsm-angstrom+ GSL_CONST_CGSM_ANGSTROM) :type double-float)
(constant (+cgsm-hectare+ GSL_CONST_CGSM_HECTARE) :type double-float)
(constant (+cgsm-acre+ GSL_CONST_CGSM_ACRE) :type double-float)
(constant (+cgsm-barn+ GSL_CONST_CGSM_BARN) :type double-float)
(constant (+cgsm-liter+ GSL_CONST_CGSM_LITER) :type double-float)
(constant (+cgsm-us-gallon+ GSL_CONST_CGSM_US_GALLON) :type double-float)
(constant (+cgsm-quart+ GSL_CONST_CGSM_QUART) :type double-float)
(constant (+cgsm-pint+ GSL_CONST_CGSM_PINT) :type double-float)
(constant (+cgsm-cup+ GSL_CONST_CGSM_CUP) :type double-float)
(constant (+cgsm-fluid-ounce+ GSL_CONST_CGSM_FLUID_OUNCE) :type double-float)
(constant (+cgsm-tablespoon+ GSL_CONST_CGSM_TABLESPOON) :type double-float)
(constant (+cgsm-teaspoon+ GSL_CONST_CGSM_TEASPOON) :type double-float)
(constant (+cgsm-canadian-gallon+ GSL_CONST_CGSM_CANADIAN_GALLON) :type double-float)
(constant (+cgsm-uk-gallon+ GSL_CONST_CGSM_UK_GALLON) :type double-float)
(constant (+cgsm-miles-per-hour+ GSL_CONST_CGSM_MILES_PER_HOUR) :type double-float)
(constant (+cgsm-kilometers-per-hour+ GSL_CONST_CGSM_KILOMETERS_PER_HOUR)
	  :type double-float)
(constant (+cgsm-knot+ GSL_CONST_CGSM_KNOT) :type double-float)
(constant (+cgsm-pound-mass+ GSL_CONST_CGSM_POUND_MASS) :type double-float)
(constant (+cgsm-ounce-mass+ GSL_CONST_CGSM_OUNCE_MASS) :type double-float)
(constant (+cgsm-ton+ GSL_CONST_CGSM_TON) :type double-float)
(constant (+cgsm-metric-ton+ GSL_CONST_CGSM_METRIC_TON) :type double-float)
(constant (+cgsm-uk-ton+ GSL_CONST_CGSM_UK_TON) :type double-float)
(constant (+cgsm-troy-ounce+ GSL_CONST_CGSM_TROY_OUNCE) :type double-float)
(constant (+cgsm-carat+ GSL_CONST_CGSM_CARAT) :type double-float)
(constant (+cgsm-unified-atomic-mass+ GSL_CONST_CGSM_UNIFIED_ATOMIC_MASS)
	  :type double-float)
(constant (+cgsm-gram-force+ GSL_CONST_CGSM_GRAM_FORCE) :type double-float)
(constant (+cgsm-pound-force+ GSL_CONST_CGSM_POUND_FORCE) :type double-float)
(constant (+cgsm-kilopound-force+ GSL_CONST_CGSM_KILOPOUND_FORCE) :type double-float)
(constant (+cgsm-poundal+ GSL_CONST_CGSM_POUNDAL) :type double-float)
(constant (+cgsm-calorie+ GSL_CONST_CGSM_CALORIE) :type double-float)
(constant (+cgsm-btu+ GSL_CONST_CGSM_BTU) :type double-float)
(constant (+cgsm-therm+ GSL_CONST_CGSM_THERM) :type double-float)
(constant (+cgsm-horsepower+ GSL_CONST_CGSM_HORSEPOWER) :type double-float)
(constant (+cgsm-bar+ GSL_CONST_CGSM_BAR) :type double-float)
(constant (+cgsm-std-atmosphere+ GSL_CONST_CGSM_STD_ATMOSPHERE) :type double-float)
(constant (+cgsm-torr+ GSL_CONST_CGSM_TORR) :type double-float)
(constant (+cgsm-meter-of-mercury+ GSL_CONST_CGSM_METER_OF_MERCURY)
	  :type double-float)
(constant (+cgsm-inch-of-mercury+ GSL_CONST_CGSM_INCH_OF_MERCURY) :type double-float)
(constant (+cgsm-inch-of-water+ GSL_CONST_CGSM_INCH_OF_WATER) :type double-float)
(constant (+cgsm-psi+ GSL_CONST_CGSM_PSI) :type double-float)
(constant (+cgsm-poise+ GSL_CONST_CGSM_POISE) :type double-float)
(constant (+cgsm-stokes+ GSL_CONST_CGSM_STOKES) :type double-float)
(constant (+cgsm-faraday+ GSL_CONST_CGSM_FARADAY) :type double-float)
(constant (+cgsm-electron-charge+ GSL_CONST_CGSM_ELECTRON_CHARGE) :type double-float)
;;;(constant (+cgsm-gauss+ GSL_CONST_CGSM_GAUSS) :type double-float)
(constant (+cgsm-stilb+ GSL_CONST_CGSM_STILB) :type double-float)
(constant (+cgsm-lumen+ GSL_CONST_CGSM_LUMEN) :type double-float)
(constant (+cgsm-lux+ GSL_CONST_CGSM_LUX) :type double-float)
(constant (+cgsm-phot+ GSL_CONST_CGSM_PHOT) :type double-float)
(constant (+cgsm-footcandle+ GSL_CONST_CGSM_FOOTCANDLE) :type double-float)
(constant (+cgsm-lambert+ GSL_CONST_CGSM_LAMBERT) :type double-float)
(constant (+cgsm-footlambert+ GSL_CONST_CGSM_FOOTLAMBERT) :type double-float)
(constant (+cgsm-curie+ GSL_CONST_CGSM_CURIE) :type double-float)
(constant (+cgsm-roentgen+ GSL_CONST_CGSM_ROENTGEN) :type double-float)
(constant (+cgsm-rad+ GSL_CONST_CGSM_RAD) :type double-float)
(constant (+cgsm-solar-mass+ GSL_CONST_CGSM_SOLAR_MASS) :type double-float)
(constant (+cgsm-bohr-radius+ GSL_CONST_CGSM_BOHR_RADIUS) :type double-float)
(constant (+cgsm-newton+ GSL_CONST_CGSM_NEWTON) :type double-float)
(constant (+cgsm-dyne+ GSL_CONST_CGSM_DYNE) :type double-float)
(constant (+cgsm-joule+ GSL_CONST_CGSM_JOULE) :type double-float)
(constant (+cgsm-erg+ GSL_CONST_CGSM_ERG) :type double-float)
(constant (+cgsm-stefan-boltzmann-constant+
	   GSL_CONST_CGSM_STEFAN_BOLTZMANN_CONSTANT)
	  :type double-float)
(constant (+cgsm-thomson-cross-section+ GSL_CONST_CGSM_THOMSON_CROSS_SECTION)
	  :type double-float)
