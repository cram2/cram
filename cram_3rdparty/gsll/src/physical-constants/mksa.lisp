;; Physical constants in the MKSA system
;; Liam Healy 2009-05-25 17:01:32EDT mksa.lisp
;; Time-stamp: <2010-05-23 11:35:28EDT mksa.lisp>
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

(include "gsl/gsl_const_mksa.h")

(constant (+mksa-speed-of-light+ "GSL_CONST_MKSA_SPEED_OF_LIGHT") :type double-float)
(constant (+mksa-gravitational-constant+ "GSL_CONST_MKSA_GRAVITATIONAL_CONSTANT")
	  :type double-float)
(constant (+mksa-plancks-constant-h+ GSL_CONST_MKSA_PLANCKS_CONSTANT_H)
	  :type double-float)
(constant (+mksa-plancks-constant-hbar+ GSL_CONST_MKSA_PLANCKS_CONSTANT_HBAR)
	  :type double-float)
(constant (+mksa-astronomical-unit+ GSL_CONST_MKSA_ASTRONOMICAL_UNIT) :type double-float)
(constant (+mksa-light-year+ GSL_CONST_MKSA_LIGHT_YEAR) :type double-float)
(constant (+mksa-parsec+ GSL_CONST_MKSA_PARSEC) :type double-float)
(constant (+mksa-grav-accel+ GSL_CONST_MKSA_GRAV_ACCEL) :type double-float)
(constant (+mksa-electron-volt+ GSL_CONST_MKSA_ELECTRON_VOLT) :type double-float)
(constant (+mksa-mass-electron+ GSL_CONST_MKSA_MASS_ELECTRON) :type double-float)
(constant (+mksa-mass-muon+ GSL_CONST_MKSA_MASS_MUON) :type double-float)
(constant (+mksa-mass-proton+ GSL_CONST_MKSA_MASS_PROTON) :type double-float)
(constant (+mksa-mass-neutron+ GSL_CONST_MKSA_MASS_NEUTRON) :type double-float)
(constant (+mksa-rydberg+ GSL_CONST_MKSA_RYDBERG) :type double-float)
(constant (+mksa-boltzmann+ GSL_CONST_MKSA_BOLTZMANN) :type double-float)
(constant (+mksa-bohr-magneton+ GSL_CONST_MKSA_BOHR_MAGNETON) :type double-float)
(constant (+mksa-nuclear-magneton+ GSL_CONST_MKSA_NUCLEAR_MAGNETON) :type double-float)
(constant (+mksa-electron-magnetic-moment+ GSL_CONST_MKSA_ELECTRON_MAGNETIC_MOMENT)
	  :type double-float)
(constant (+mksa-proton-magnetic-moment+ GSL_CONST_MKSA_PROTON_MAGNETIC_MOMENT)
	  :type double-float)
(constant (+mksa-molar-gas+ GSL_CONST_MKSA_MOLAR_GAS) :type double-float)
(constant (+mksa-standard-gas-volume+ GSL_CONST_MKSA_STANDARD_GAS_VOLUME) :type double-float)
(constant (+mksa-minute+ GSL_CONST_MKSA_MINUTE) :type double-float)
(constant (+mksa-hour+ GSL_CONST_MKSA_HOUR) :type double-float)
(constant (+mksa-day+ GSL_CONST_MKSA_DAY) :type double-float)
(constant (+mksa-week+ GSL_CONST_MKSA_WEEK) :type double-float)
(constant (+mksa-inch+ GSL_CONST_MKSA_INCH) :type double-float)
(constant (+mksa-foot+ GSL_CONST_MKSA_FOOT) :type double-float)
(constant (+mksa-yard+ GSL_CONST_MKSA_YARD) :type double-float)
(constant (+mksa-mile+ GSL_CONST_MKSA_MILE) :type double-float)
(constant (+mksa-nautical-mile+ GSL_CONST_MKSA_NAUTICAL_MILE) :type double-float)
(constant (+mksa-fathom+ GSL_CONST_MKSA_FATHOM) :type double-float)
(constant (+mksa-mil+ GSL_CONST_MKSA_MIL) :type double-float)
(constant (+mksa-point+ GSL_CONST_MKSA_POINT) :type double-float)
(constant (+mksa-texpoint+ GSL_CONST_MKSA_TEXPOINT) :type double-float)
(constant (+mksa-micron+ GSL_CONST_MKSA_MICRON) :type double-float)
(constant (+mksa-angstrom+ GSL_CONST_MKSA_ANGSTROM) :type double-float)
(constant (+mksa-hectare+ GSL_CONST_MKSA_HECTARE) :type double-float)
(constant (+mksa-acre+ GSL_CONST_MKSA_ACRE) :type double-float)
(constant (+mksa-barn+ GSL_CONST_MKSA_BARN) :type double-float)
(constant (+mksa-liter+ GSL_CONST_MKSA_LITER) :type double-float)
(constant (+mksa-us-gallon+ GSL_CONST_MKSA_US_GALLON) :type double-float)
(constant (+mksa-quart+ GSL_CONST_MKSA_QUART) :type double-float)
(constant (+mksa-pint+ GSL_CONST_MKSA_PINT) :type double-float)
(constant (+mksa-cup+ GSL_CONST_MKSA_CUP) :type double-float)
(constant (+mksa-fluid-ounce+ GSL_CONST_MKSA_FLUID_OUNCE) :type double-float)
(constant (+mksa-tablespoon+ GSL_CONST_MKSA_TABLESPOON) :type double-float)
(constant (+mksa-teaspoon+ GSL_CONST_MKSA_TEASPOON) :type double-float)
(constant (+mksa-canadian-gallon+ GSL_CONST_MKSA_CANADIAN_GALLON) :type double-float)
(constant (+mksa-uk-gallon+ GSL_CONST_MKSA_UK_GALLON) :type double-float)
(constant (+mksa-miles-per-hour+ GSL_CONST_MKSA_MILES_PER_HOUR) :type double-float)
(constant (+mksa-kilometers-per-hour+ GSL_CONST_MKSA_KILOMETERS_PER_HOUR)
	  :type double-float)
(constant (+mksa-knot+ GSL_CONST_MKSA_KNOT) :type double-float)
(constant (+mksa-pound-mass+ GSL_CONST_MKSA_POUND_MASS) :type double-float)
(constant (+mksa-ounce-mass+ GSL_CONST_MKSA_OUNCE_MASS) :type double-float)
(constant (+mksa-ton+ GSL_CONST_MKSA_TON) :type double-float)
(constant (+mksa-metric-ton+ GSL_CONST_MKSA_METRIC_TON) :type double-float)
(constant (+mksa-uk-ton+ GSL_CONST_MKSA_UK_TON) :type double-float)
(constant (+mksa-troy-ounce+ GSL_CONST_MKSA_TROY_OUNCE) :type double-float)
(constant (+mksa-carat+ GSL_CONST_MKSA_CARAT) :type double-float)
(constant (+mksa-unified-atomic-mass+ GSL_CONST_MKSA_UNIFIED_ATOMIC_MASS)
	  :type double-float)
(constant (+mksa-gram-force+ GSL_CONST_MKSA_GRAM_FORCE) :type double-float)
(constant (+mksa-pound-force+ GSL_CONST_MKSA_POUND_FORCE) :type double-float)
(constant (+mksa-kilopound-force+ GSL_CONST_MKSA_KILOPOUND_FORCE) :type double-float)
(constant (+mksa-poundal+ GSL_CONST_MKSA_POUNDAL) :type double-float)
(constant (+mksa-calorie+ GSL_CONST_MKSA_CALORIE) :type double-float)
(constant (+mksa-btu+ GSL_CONST_MKSA_BTU) :type double-float)
(constant (+mksa-therm+ GSL_CONST_MKSA_THERM) :type double-float)
(constant (+mksa-horsepower+ GSL_CONST_MKSA_HORSEPOWER) :type double-float)
(constant (+mksa-bar+ GSL_CONST_MKSA_BAR) :type double-float)
(constant (+mksa-std-atmosphere+ GSL_CONST_MKSA_STD_ATMOSPHERE) :type double-float)
(constant (+mksa-torr+ GSL_CONST_MKSA_TORR) :type double-float)
(constant (+mksa-meter-of-mercury+ GSL_CONST_MKSA_METER_OF_MERCURY)
	  :type double-float)
(constant (+mksa-inch-of-mercury+ GSL_CONST_MKSA_INCH_OF_MERCURY) :type double-float)
(constant (+mksa-inch-of-water+ GSL_CONST_MKSA_INCH_OF_WATER) :type double-float)
(constant (+mksa-psi+ GSL_CONST_MKSA_PSI) :type double-float)
(constant (+mksa-poise+ GSL_CONST_MKSA_POISE) :type double-float)
(constant (+mksa-stokes+ GSL_CONST_MKSA_STOKES) :type double-float)
(constant (+mksa-faraday+ GSL_CONST_MKSA_FARADAY) :type double-float)
(constant (+mksa-electron-charge+ GSL_CONST_MKSA_ELECTRON_CHARGE) :type double-float)
(constant (+mksa-gauss+ GSL_CONST_MKSA_GAUSS) :type double-float)
(constant (+mksa-stilb+ GSL_CONST_MKSA_STILB) :type double-float)
(constant (+mksa-lumen+ GSL_CONST_MKSA_LUMEN) :type double-float)
(constant (+mksa-lux+ GSL_CONST_MKSA_LUX) :type double-float)
(constant (+mksa-phot+ GSL_CONST_MKSA_PHOT) :type double-float)
(constant (+mksa-footcandle+ GSL_CONST_MKSA_FOOTCANDLE) :type double-float)
(constant (+mksa-lambert+ GSL_CONST_MKSA_LAMBERT) :type double-float)
(constant (+mksa-footlambert+ GSL_CONST_MKSA_FOOTLAMBERT) :type double-float)
(constant (+mksa-curie+ GSL_CONST_MKSA_CURIE) :type double-float)
(constant (+mksa-roentgen+ GSL_CONST_MKSA_ROENTGEN) :type double-float)
(constant (+mksa-rad+ GSL_CONST_MKSA_RAD) :type double-float)
(constant (+mksa-solar-mass+ GSL_CONST_MKSA_SOLAR_MASS) :type double-float)
(constant (+mksa-bohr-radius+ GSL_CONST_MKSA_BOHR_RADIUS) :type double-float)
(constant (+mksa-newton+ GSL_CONST_MKSA_NEWTON) :type double-float)
(constant (+mksa-dyne+ GSL_CONST_MKSA_DYNE) :type double-float)
(constant (+mksa-joule+ GSL_CONST_MKSA_JOULE) :type double-float)
(constant (+mksa-erg+ GSL_CONST_MKSA_ERG) :type double-float)
(constant (+mksa-stefan-boltzmann-constant+
	   GSL_CONST_MKSA_STEFAN_BOLTZMANN_CONSTANT)
	  :type double-float)
(constant (+mksa-thomson-cross-section+ GSL_CONST_MKSA_THOMSON_CROSS_SECTION)
	  :type double-float)
(constant (+mksa-vacuum-permittivity+ GSL_CONST_MKSA_VACUUM_PERMITTIVITY)
	  :type double-float)
(constant (+mksa-vacuum-permeability+ GSL_CONST_MKSA_VACUUM_PERMEABILITY)
	  :type double-float)
(constant (+mksa-debye+ GSL_CONST_MKSA_DEBYE) :type double-float)
