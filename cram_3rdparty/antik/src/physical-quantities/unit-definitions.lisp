;; Definitions of units and dimensions.
;; Liam Healy Fri Mar  1 2002 - 15:34
;; Time-stamp: <2015-04-20 12:45:08EDT unit-definitions.lisp>

;; Copyright 2011 Liam M. Healy
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

(export '(radian revolution half-revolution 2pi))

;;; Define particular dimensions and units; units.lisp must be loaded
;;; before this can be loaded.  See
;;; http://physics.nist.gov/cuu/Units/index.html for names,
;;; abbreviations, and definitions of units.
;;; Most of these definitions are taken from Novak's file.

;;;;****************************************************************************
;;;; Convenience definitions
;;;;****************************************************************************

(defvar *degree-symbol* "o" "The degree symbol.")
(unit-symbol-macro radian)
(unit-symbol-macro revolution)
(unit-symbol-macro revolution 1/2 half-)

(defconstant 2pi (* 2 pi))

;;;;****************************************************************************
;;;; Dimensions
;;;;****************************************************************************

(define-basis-dimensions)

(define-derived-dimensions
    '((force              (/ (* mass length) (* time time)))
      (area               (* length length))
      (volume             (* length length length))
      (power              (/ (* mass length length)
			   (* time time time)))
      (energy             (/ (* mass length length) (* time time)))
      (velocity              (/ length time))
      (momentum           (* mass velocity))
      (angular-momentum   (* mass length velocity))
      (acceleration       (/ length (* time time)))
      (pressure           (/ force area))
      (density            (/ mass volume))
      (charge             (* current time))
      (electric-potential (/ power current))
      (capacitance        (/ charge electric-potential))
      (resistance         (/ electric-potential current))
      (conductance        (/ current electric-potential))
      (magnetic-field     (/ mass (* current time time)))
      (magnetic-flux      (* magnetic-field area))
      (inductance         (/ magnetic-flux current))
      (frequency          (/ angle time))
      (dose               (/ (* length length) (* time time)))))

;;;;****************************************************************************
;;;; Basic units
;;;;****************************************************************************

(define-units 'dimensionless
    `((unity          1.0       ())
      (dozen          12.0      ())
      (gross          144.0     ())
      (ten            10.0      ())
      (hundred        100.0     ())
      (thousand       1000.0    ())
      (million        1.0e6     ())
      (billion        1.0e9     ())
      (trillion       1.0e12    ())
      (quadrillion    1.0e15    ())
      (quintillion    1.0e18    ())
      (percent        0.01      (\% percent))
      (tenth          0.1       ())
      (hundredth      0.01      ())
      (thousandth     0.001     ())
      (millionth      1.0e-6    ())
      (billionth      1.0e-9    ())
      (trillionth     1.0e-12   ())
      (quadrillionth  1.0e-15   ())
      (quintillionth  1.0e-18   ())
      (yotta          1.0e24    (yotta-))
      (zetta          1.0e21    (zetta-))
      (exa            1.0e18    (exa-))
      (peta           1.0e15    (peta-))
      (tera           1.0e12    (tera-))
      (giga           1.0e9     (giga-))
      (mega           1.0e6     (mega-))
      (kilo           1000.0    (kilo-))
      (hecto          100.0     (hecto-))
      (deka           10.0      (deca deka- deca-))
      (deci           0.1       (deci-))
      (centi          0.01      (centi-))
      (milli          0.001     (milli-))
      (micro          1.0e-6    (micro-))
      (nano           1.0e-9    (nano-))
      (pico           1.0e-12   (pico-))
      (femto          1.0e-15   (femto-))
      (atto           1.0e-18   (atto-))
      (zepto          1.0e-21   (zepto-))
      (yocto          1.0e-24   (yocto-))))

(define-units 'length
    '((meter         1.0       (m meters) "m")
      (foot          0.3048    (ft feet) "ft")
      (decimeter     0.1       (dm decimeters))
      (centimeter    0.01      (cm centimeters) "cm")
      (millimeter    0.001     (mm millimeters) "mm")
      (dekameter     10.0      (dam dekameters decameter
				decameters))
      (hectometer    100.0     (hm hectometers))
      (kilometer     1000.0    (km kilometers) "km")
      (micron        1.0e-6    (um micro-meter micrometer
				micrometers micro-meters microns))
      (nanometer     1.0e-9    (nm nanometers))
      (angstrom      1.0e-10   (a  angstroms))
      (inch          0.0254    (in inches) "in")
      (mile          1609.344  (mi miles) "mi")
      (nautical-mile 1852.0    (nmi nauticalmiles
				nauticalmile nautical-miles) "n.mi.")
      (astronomical-unit       1.49598e11 (au))
      (light-year    9.46e15    (ly light-years lightyear lightyears))
      (parsec        3.083e16   (parsecs))
      (fathom        1.8054     (fathoms))
      (yard          0.9144     (yd yards))
      (rod           5.0292     (rods))
      (mil           0.0000254  (mils))
      (furlong       201.168    (furlongs))))

(define-units 'time
    '((second      1.0        (s sec secs seconds) "s")
      (millisecond (* milli second)     (ms msec millisec milliseconds) "ms")
      (microsecond (* micro second)     (us usec microsec microseconds))
      (nanosecond  (* nano  second)     (ns nsec nanosec nanoseconds) "ns")
      (picosecond  (* pico  second)     (ps psec picosec picoseconds))
      (femtosecond (* femto second)     (femtoseconds femtosec))
      (attosecond  (* atto  second)     (attoseconds attosec))
      (minute      (* 60    second)     (min minutes))
      (hour        (* 3600  second)     (hr hours))
      (day         (* 86400 second)     (days))
      (week        (* 604800 second)    (wk weeks))
      (fortnight   (* 1209600 second)   (fortnights))
      (month       (* 2629800 second)   (mon months))
      (year        (* 31557600 second)  (yr years))
      (century     (* 3.15576e9 second) (centuries))))

(define-units 'mass
    '((kilogram         1.0           (kg kilograms) "kg")
      (hectogram        0.1           (hg hectograms))
      (dekagram         0.01          (dag dekagrams decagram decagrams))
      (gram             0.001         (g gm grams) "g")
      (decigram         0.0001        (dg decigrams))
      (centigram        0.00001       (cg centigrams))
      (milligram        1.0e-6        (mg milligrams))
      (microgram        1.0e-9        (ug micrograms))
      (metric-ton       1000.0        (metric-tons tonne tonnes))
      (pound            0.45359237    (lb lbs pounds)) ; exactly
      (slug             14.593902937  (slugs))
      ;; derived 02 Jun 95 based on pound, foot, and earth-gravity
      (atomic-mass-unit 1.6605402e-27 (amu atomic-mass-units))
      (earth-mass       5.98e24       ())
      (ounce            (/ pound 16)  (oz ounces))
      (ton              (* 2000 pound) (tons short-ton short-tons))
      (long-ton         (* 2240 pound) (long-ton long-tons))
      (hundredweight    (* 100 pound) (hundredweights))
      (dram             (/ ounce 16)  (drams))
      (grain            (/ dram 27.344) (grains))
      (troy-pound       (* 0.373 kilogram) (troy-pounds))
      (troy-ounce       (* 31.103 gram) (troy-ounces ounce-troy ounces-troy))
      (pennyweight      (* 1.555 gram) (pennyweights))
      (scruple          (* 1.296 gram) (scruples))))

(define-units 'temperature
    '((kelvin      1.0       (k kelvin kelvins) "K")
      (rankine     5/9       ()) ))

(define-units 'current
    '((ampere      1.0       (amp amps amperes)) 
      (milliampere (* milli ampere)
       (milliamp milliamps ma milliampere))
      (microampere (* micro ampere)
       (microamp microamps ua microamperes))
      (abampere    (* 10 ampere) (abamp abamperes))
      (statampere  (* 3.336e-10 ampere) (statamp statamperes))))

(define-units 'substance
    '((mole               1.0       (mol moles)) ))

(define-units 'luminosity
    '((candela            1.0       (cd candelas)) ))

(define-units 'money
    '((dollar             1.0       (dollars $)) ))

(define-units 'angle
    `((radian    1.0                (rad radians) "rad")
      (steradian 1.0                (sr steradians))
      (degree    (/ ,pi 180)
       (,(antik-symbol *degree-symbol*) deg degrees) (,*degree-symbol* "\\dgr") nil)
      (arcminute (/ degree 60)      (arcmin arcminutes arc-minute arc-minutes) nil)
      (arcsecond (/ arcminute 60)   (arcsec arcseconds arc-second arc-seconds) nil)
      (revolution (* 2 ,pi)         (rev orbit) nil)))

;;;;****************************************************************************
;;;; Derived units
;;;;****************************************************************************

(define-units 'area
    '((hectare (* 10000 meter meter)      (hectares))
      (are     (* 100 meter meter)        (ares))
      (acre    (* 43560 foot foot)        (acres))
      (barn    (* 1.0e-28 meter meter)    (barns))))

(define-units 'volume
    '((acre-foot        (* acre foot)       (acrefoot acre-feet acrefeet))
      (liter           (* 0.001 (expt meter 3))
       (l liters cubic-decimeter cubic-decimeters))
      (deciliter       (/ liter 10)       (dl deciliters))
      (centiliter      (/ liter 100) (cl centiliters))
      (dekaliter       (* liter 10) (dekaliters decaliter decaliters))
      (hectoliter      (* 100 liter) (hectoliters))
      (gallon          (* 3.785411784 liter) (gal gallons))
      (quart           (/ gallon 4) (qt quarts))
      (peck            (* 8 quart) (pecks))
      (bushel          (* 4 peck) (bushels))
      (fifth           (/ gallon 5) (fifths))
      (pint            (* 0.473 liter) (pt pints))
      (cup             (/ pint 2) (cups))
      (fluid-ounce     (* 0.029573 liter)
       (floz fluidounce fluidounces fluid-ounces))
      (gill            (* 4 fluid-ounce) (gills))
      (fluidram        (* 3.5516 (expt cm 3)) (fluidrams))
      (minim           (* 0.059194 (expt cm 3)) (minims))
      (tablespoon      (/ fluidounce 2) (tbsp tablespoons))
      (teaspoon        (/ tablespoon 3) (tsp teaspoons)) ) )

(define-units 'acceleration
    '((earth-gravity (* 9.80665 (/ meter (* second second))) (gs))))

(define-units 'force
    '((pound-force  (/ (* slug foot) (* second second)) (lbf))
      (ounce-force  (/ pound-force 16)        ())
      (newton (/ (* kilogram meter) (* second second)) (N nt newtons) "N")
      (dyne   (/ (* gram centimeter) (* second second)) (dynes))
      (kilogram-force  (* kilogram earth-gravity) (kgf kilogram-weight))
      (gram-weight     (* gram earth-gravity) (gram-force)) ))

(define-units 'power
    '((watt       (/ (* kilogram meter meter) (* second second second))
		  (w watts))
      (milliwatt  (* milli watt) (mlw milli-watt milli-watts))
      (microwatt  (* micro watt) (uw micro-watt micro-watts))
      (kilowatt   (* kilo watt) (kw kilowatts))
      (megawatt   (* mega watt) (mw megawatts mega-watt mega-watts))
      (gigawatt   (* giga watt) (gw gigawatts giga-watt giga-watts))
      (horsepower (* 550 (/ (* foot pound-force) second)) (hp))))

(define-units 'energy
    '((joule (/ (* kilogram meter meter) (* second second))
	     (j joules) "J")
      (foot-pound (* foot pound-force)
       (ftlb ft-lb footpound footpounds foot-pounds))
      (kilowatt-hour (* kilo watt hour)
       (kwh kilowatthour kilowatthours
	kilowatt-hours))
      (watt-hour (* watt hour) (watthour watthours watt-hours))
      (horsepower-hour (* horsepower hour) (hp-hour))
      (electron-volt (* 1.60217733e-19 joule) (ev electronvolt electronvolts
	electron-volts))
      (kev (* 1000 electronvolt) (kilo-electron-volts))
      (mev (* 1.60217733e-13 joule) (mega-electron-volts))
      (gev (* 1.60217733e-10 joule) (giga-electron-volts))
      (tev (* 1.60217733e-7 joule) (tera-electron-volts))
      (calorie (* 4.184 joule) (cal calorie calories))
      (kilocalorie (* 4184.0 joule) (kcal kilo-calorie kilo-calories))
      (british-thermal-unit (* 1055.056 joule)
       (btu btus britishthermalunit
	britishthermalunits
	british-thermal-units))
      (erg (* 1.0e-7 joule) (ergs)) ) )

(define-units 'charge
    '((coulomb     (* ampere second)     (coul coulombs))
      (abcoulomb   (* 10.0 coulomb)      (abcoul abcoulombs))
      (statcoulomb (* 3.336e-10 coulomb) (statcoul statcoulombs))
      (amperehour  (* 3600.0  coulomb)
       (amp-hour ampere-hour amperehours ampere-hours))))

(define-units 'pressure
    '((pounds-per-square-inch (/ (* 144 pound-force) (* foot foot)) (psi))
      (pascal     (/ newton (* meter meter)) (pa))
      (kilopascal (* 1000.0 pascal) (kilo-pascal kpa kilopascals))
      (bar        (* 1.0e5 pascal)  (bars))
      (millibar   (* milli bar)     (millibars))
      (torr       (* (/ 101325 760) pascal) ())
      (dynes-per-square-centimeter (/ dyne (* centimeter centimeter)))
      (atmosphere (* 101325 pascal) (atm))))

(define-units 'velocity
    '((knot              (/ nautical-mile hour) (knots))
      (speed-of-light    (* 299792458 (/ meter second)))))

(define-units 'dose			; of radiation
    '((gray    (/ joule kilogram)   (gy))
      (sievert (/ joule kilogram)   (sv))
      ;;(rad     (/ gray 100)         ()) ; already exists as 'rem
      (rem     (/ sievert 100)      ())))

(define-units 'frequency
    `((hertz     ,2pi (hz) "Hz")
      (becquerel ,2pi (bq))
      (kilohertz   (* kilo hertz)       (khz) "KHz")
      (megahertz   (* mega hertz)       (mhz) "MHz")
      (gigahertz   (* giga hertz)       (ghz) "GHz")
      (terahertz   (* tera hertz)       (thz) "THz")
      (curie       (* 3.7e10 becquerel) (curies))))

(define-units 'electric-potential
    '((volt      (/ (* kilogram meter meter)
		    (* ampere second second second))
		 (v volts))
      (millivolt (* milli volt)  (mv millivolts))
      (microvolt (* micro volt)  (uv microvolts))
      (abvolt    (* 1.0e-8 volt) (abvolts))
      (statvolt  (* 299.8 volt)  (statvolts)) ))

(define-units 'resistance
    '((ohm      (/ (* kilogram meter meter)
		   (* ampere ampere second second second))
		(ohms))
      (kilohm   (* kilo ohm)     (kilohms))
      (megohm   (* mega ohm)     (megohms))
      (abohm    (* nano ohm)     (abohms))
      (statohm  (* 8.987e11 ohm) (statohms)) ))

(define-units 'conductance
    '((siemens      (/ (* ampere ampere second second second)
		       (* kilogram meter meter)) () ) ))

(define-units 'capacitance
    '((farad   (/ (* ampere ampere second second second second)
		  (* kilogram meter meter))
	       (f farads))
      (microfarad (* micro farad)     (uf microfarads))
      (picofarad  (* pico farad)      (pf picofarads))
      (abfarad    (* giga farad)      (abfarads))
      (statfarad  (* 1.113e-12 farad) (statfarads)) ))

(define-units 'inductance
    '((henry      (/ (* kilogram meter meter)
		     (* ampere ampere second second))
		  (henrys))
      (millihenry (* milli henry)    (mh millihenrys))
      (microhenry (* micro henry)    (uh microhenrys))
      (abhenry    (* nano henry)     (abhenrys))
      (stathenry  (* 8.987e11 henry) (stathenrys)) ))

(define-units 'magnetic-flux
    '((weber      (/ (* kilogram meter meter)
		     (* ampere second second))
		  (wb webers))
      (maxwell    (* 1.0e-8 weber)  (maxwells)) ))

(define-units 'magnetic-field
    '((tesla      (/ kilogram (* ampere second second))
		  (teslas))
      (gauss      (* 1.0e-4 tesla) ())
      (milligauss (* milli gauss)  ()) ))


