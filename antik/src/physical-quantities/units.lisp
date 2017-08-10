;; Define physical dimensions and units of measure and systems of units.
;; Liam Healy Wed Feb 27 2002 - 13:18
;; Time-stamp: <2015-04-20 12:51:32EDT units.lisp>

;; Copyright 2011, 2013, 2015 Liam M. Healy
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

(export '(dimension
	  get-canonical-name print-name
	  long-print-name short-print-name tex-print-name
	  define-unit define-units
	  define-physical-constant
	  define-system-of-units
	  set-system-of-units
	  with-system-of-units
	  with-si-units
	  dimensionless))

;;; Handle physical dimensions and units of measure, for any
;;; units scaling type (simple ratio).
;;; Loosely adapted from Novak.

;;;;****************************************************************************
;;;; Build/access table of information about units and dimensions
;;;;****************************************************************************

;;; Each entry in the table is corresponds to a unit of measure (e.g. 'meter)
;;; or a physical dimension (e.g. 'length) indexed by the symbol.

(defvar *unit-information* (make-hash-table)
  "Properties of each unit of measure and physical dimension.")

(defmacro unit-property (unit property)
  "Find or set the particular property of the unit or dimension.
   Package of unit does not matter."
  `(getf (gethash (alexandria:ensure-symbol ,unit :antik)
		  *unit-information*)
	 ,property))

;;; The physical dimension is given as a list of exponents of the
;;; basis dimensions.  It is recorded for both unit names and dimension names. 
(defmacro dimension (unit-or-dimension)
  "The dimension as a dimel (list of exponents)
   from a unit name or dimension name."
  `(unit-property ,unit-or-dimension 'dimension))

(defmacro siconversion (unit)
  "Ratio of the given unit to the SI unit of the same dimension."
  `(unit-property ,unit 'siconversion))

;;; Synonyms are recorded in order of length, so the shortest one is
;;; used for printout.  They are recorded in the *unit-information* table
;;; with just the 'canonical-name property; all other information will be found
;;; by looking up the full unit name. 

(defmacro synonyms (unit)
  "The synonyms of the unit."
  `(unit-property ,unit 'synonyms))

(defmacro canonical-name (unit-or-syn)
  "The full unit name from a synonym."
  `(unit-property ,unit-or-syn 'canonical-name))

(defun get-canonical-name (symbol)
  (or (canonical-name symbol)
      (error "Unit/attribute ~a unknown" symbol)))

;;; By convention, print names are listed as:
;;;   long &optional tex short
;;; This should probably be formalized better with keywords.

(defmacro print-name (unit)
  "How a unit is printed."
  `(unit-property ,unit 'print-name))

(defun long-print-name (unit)
  "How the unit or attribute should be printed on the screen."
  (first (print-name unit)))

(defun short-print-name (unit)
  "How the unit or attribute should be printed on the screen."
  (or (third (print-name unit)) (long-print-name unit)))

(defun tex-print-name (unit)
  "How the unit or attribute should be printed in TeX."
  (or (second (print-name unit))
      (format nil "\\text{~a}" (short-print-name unit))))

;;;;****************************************************************************
;;;; Find units from a system of units
;;;;****************************************************************************

(defun sysunits-dimel (x) (first x))
(defun sysunits-unitname (x) (rest x))

(defun dimel-magnitude (dimel)
  "A metric on the dimel."
  (apply 'cl:+ (mapcar 'cl:abs dimel)))

(defun sysunits-metric (dimel-index-item)
  (dimel-magnitude (sysunits-dimel dimel-index-item)))

(defun flfl (int1 int2)
  "Flexible floor; if int2 is zero,
   returns nil if int1 is nonzero or t if it is."
  (if (zerop int2)
      (zerop int1)
    (floor int1 int2)))

;;; (vecfloor '(2 -4 2 0 0 0 0 0 ) '(1 -2 1 0 0 0 0 0 0))
;;; 2
;;; (0 0 0 0 0 0 0 0)
;;; (vecfloor '(2 -4 3 0 0 0 0 0 ) '(1 -2 1 0 0 0 0 0 0))
;;; 2
;;; (0 0 1 0 0 0 0 0)
;;; (vecfloor '(1 -2 1 0 0 0 0 0 1) '(1 -2 1 0 0 0 0 0 0))
;;; 1
;;; (0 0 0 0 0 0 0 0 1)
(defun vecfloor (vec1 vec2)
  "Find the lowest flexible floor of the two vectors of integers, and the
   remainder vec1-gcd*vec2."
  (let* ((fls (remove nil (remove t (mapcar #'flfl vec1 vec2)))) 
	 (divisor (if fls (apply 'cl:min fls) 0)))
    (values divisor (- vec1 (* divisor vec2)))))

;;; (find-units '(2 -4 3 0 0 0 0 0 0))
;;; (NEWTON 2 (0 0 1 0 0 0 0 0))
;;; (find-units '(1 -2 1 0 0 0 0 0 0))
;;; (NEWTON 1 (0 0 0 0 0 0 0 0 0))
;;; (find-units '(1 -2 1 0 0 0 0 0 1))
;;; (NEWTON 1 (0 0 0 0 0 0 0 0 1))
;;; (FIND-UNITS '(-2 1 1 0 0 0 0 0 0))
;;; (METER -2 (0 1 1 0 0 0 0 0 0))
;;; This does not handle mixed units predictably; see 2002-09-04.
(defun find-units (dimel)
  "Find a minimal combination of named units in the designated system of units
   that matches the dimel.  Returns a list (unit-name exponent remainder-dimel)."
  (if (equal (dimension 'dimensionless) dimel)
      (list 'unity 1 (dimension 'dimensionless))
    (let ((usable-units
	   (sort			; Sort in decreasing order of magnitude
	    ;; Throw away all named units whose magnitude is greater than
	    ;; the dimel we have.
	    (remove-if (lambda (n) (> n (dimel-magnitude dimel)))
		       (nf-option :system-of-units)
		       :key #'sysunits-metric)
	    'cl:>
	    :key  #'sysunits-metric))
	  ;; will accumulate the best physical unit divisor if there is no exact divisor
	  best-unit best-div (best-rem (dimension 'dimensionless))
	  (best-rem-mag most-positive-fixnum))
      (dolist (unit usable-units
		(list (sysunits-unitname best-unit) best-div best-rem))
	(multiple-value-bind (div rem)
	    (vecfloor dimel (sysunits-dimel unit))
	  (if (zerop rem)
	      ;; If exact integer multiple, that's a winner
	      (return (list (sysunits-unitname unit) div rem))
	    ;; Save the best (smallest remainder)
	    (when (< (dimel-magnitude rem) best-rem-mag)
	      (setf best-rem rem best-rem-mag (dimel-magnitude rem)
		    best-div div best-unit unit))))))))

;;;;****************************************************************************
;;;; Set system of units 
;;;;****************************************************************************

;; define-system-of-units
;; set-system-of-units	    ; set system of units
;; with-system-of-units	    ; new system of units within scope
;; with-si-units

(defun find-unit-dimel (symbol)
  (let ((cu (get-canonical-name symbol)))
    (cons (dimension cu) cu)))

(defmacro find-sysunits (sysunits)
  "Find the named system of units."
  `(unit-property ,sysunits 'system-of-units))

(defun make-sysunits (units &optional system-of-units)
  "Make a system of units by naming the units and optionally naming a system of units which serves as a base; if it is T, then use the current system-of-units."
  ;; Put an 'equal remove-duplicates on 'first to make a clean list of dimels.
  (append (mapcar 'find-unit-dimel units)
	  (if (eq system-of-units t)
	      (nf-option :system-of-units)
	      (when system-of-units
		(find-sysunits system-of-units)))))

(defmacro define-system-of-units (system-name units &optional augemented-system)
  "Define a system of units with the given name and units, possibly by augmenting the named system of units."
  `(setf (find-sysunits ',system-name) (make-sysunits ',units ',augemented-system)))

(defun set-system-of-units (system-of-units &rest units)
  "Set the default system of units to the specified system."
  (setf
   (nf-option :system-of-units)
   (make-sysunits units system-of-units))
  ;;(warn "Set default system of units.")
  (assert (nf-option :system-of-units) nil "System of units ~a is not defined." system-of-units)
  (values))

(defmacro with-system-of-units ((system-of-units &rest units) &body body)
  "When executing the body, change the system of units by naming the units. If system-of-units is non-nil, augment that system with the specified units; if it is T, augment the current system of units."
  ;; Eventually, it would be nice to be able to take systems too, but it's awkward with symbol-value and checking sysunitp etc.
  `(with-nf-options
       (:system-of-units (make-sysunits ',units ',system-of-units))
     ,@body))

(defmacro with-si-units (&body body)
  "Use the SI system of units in the body."
  `(with-system-of-units (si) ,@body))

;;;;****************************************************************************
;;;; Parsing and generating dimensions and units
;;;;****************************************************************************

;;; A physical dimension is recorded as "dimel", a list of exponents of the
;;; basis dimensions (length, mass, etc.).

(defparameter *basis-dimensions*
    '(length time mass temperature current substance luminosity money angle)
  "The basis dimensions, in order.")

(export *basis-dimensions*)

(defun make-new-dimel (&optional (initial-element 0))
  (make-list (length *basis-dimensions*) :initial-element initial-element))

(defun dimelp (dimel)
  "Argument is a scalar dimel."
  (and (listp dimel) (= (length dimel) (length *basis-dimensions*))
       (every 'cl:numberp dimel)))

(defmacro check-dimel (dimel)
  `(assert (dimelp ,dimel) (,dimel) "Not a dimel: ~a" ,dimel))

(defun dimel-basis-p (dimel)
  "Return the index if this dimel is a basis dimension/unit."
  (let ((pos (position 1 dimel)))
    (and pos
	 (every 'cl:zerop (subseq dimel 0 pos))
	 (every 'cl:zerop (subseq dimel (1+ pos)))
	 pos)))

;;; (make-dimel 's 1 'foot -1 'm -1 'kg 1)
;;; (-2 1 1 0 0 0 0 0 0)
;;; 3.280839895013123
(defun make-dimel (&rest name-expons)
  "Make a dimension exponent list (dimel) and SI conversion factor
   from name-expons of the form unitname1 expon1 unitname2 expon2...
   Symbols may be units or physical dimension names; the
   SI conversion value will be nil if a physical dimension is used."
  (let ((el (make-new-dimel)))
    (loop for (name expon) on name-expons by #'cddr
	  for cname = (if (null name)
			  'unity
			  (get-canonical-name name))
	  for pdim = (dimension cname)
	  with conv = 1
	  do
       (setf el (+ el (* expon pdim)))
       (if (and conv (siconversion cname))
	   (setf conv (* conv (expt (siconversion cname) expon)))
	   (setf conv nil))
	  finally (return (values el conv)))))

(defun op-dimension (dimension0 dimension1 &optional (operation '+))
  "Operate on the dimensions, either dimels or vectors of dimels."
  (if (and (dimelp dimension0) (dimelp dimension1))
      ;; Dimensions are dimels
      (funcall operation dimension0 dimension1)
      ;; At least one dimensions are vectors of dimels
      (if (dimelp dimension0)
	  (map 'vector (alexandria:curry operation dimension0) dimension1)
	  (if (dimelp dimension1)
	      (map 'vector (alexandria:curry operation dimension1) dimension0)
	      (map 'vector operation dimension0 dimension1)))))

(defun unit-remove-illegal-chars (unit)
  "Remove the characters that are illegal in unit and dimension
   names because they are used in unit math."
  (let ((str (remove #\/ (remove #\* (remove #\- (string unit))))))
    (if (symbolp unit)
	(intern str (symbol-package unit))
	str)))

;;; (parse-unit-sexp '(/ kg (* m (/ foot s))))
;;;   (-2 1 1 0 0 0 0 0 0)
;;;   3.280839895013123
;;; (parse-unit-sexp '(/ (* mass length) (* time time)))
;;;   (1 -2 1 0 0 0 0 0 0)
;;;   NIL
;;; (parse-unit-sexp '(/ furlong fortnight))
;;;   (1 -1 0 0 0 0 0 0 0)
;;;   1.6630952380952381e-4
(defun parse-unit-sexp (expression)
  "Find dimel from a unit expression; the symbols in this expression may
   be either unit names or dimension names.  Return the dimel and
   the conversion; if the expression contains a physical dimension (like 'time)
   as opposed to a unit of measure (like 'second), then the conversion
   is nil."
  ;; Novak's glunitdim
  (let ((nel nil) (number 1))
    (labels ((pux (expr expon)
	       "Return list of (symbol expon symbol expon ...)."
	       (cond ((numberp expr) (setf number (cl:* number (cl:expt expr expon))))
		     ((and expr (symbolp expr) (boundp expr)) ; global symbol
		      (pux (symbol-value expr) expon))
		     ((atom expr)
		      (let ((clean (unit-remove-illegal-chars expr)))
			(setf (getf nel clean)
			      (cl:+ (or (getf nel clean) 0) expon))))
		     ((member (first expr) '(* cl:*)) ; multiply
		      (mapcar (lambda (x) (pux x expon)) (rest expr)))
		     ((member (first expr) '(/ cl:/)) ; divide unit expressions
		      (if (cddr expr)
			  (pux (second expr) expon)
			  (pux (second expr) (cl:- expon))) ; one argument, invert
		      (pux (if (cdddr expr) ; multiple divisors
			       (cons '* (cddr expr))
			       (third expr))
			   (- expon)))
		     ((member (first expr) '(expt cl:expt)) ; exponentiate
		      (pux (second expr) (cl:* expon (third expr))))
		     (t (error "~A has bad unit operator.~%" expr)))))
      (pux expression 1)
      (multiple-value-bind (el conv)
	  (apply #'make-dimel nel)
	(values el (when conv (cl:* conv number)))))))

(defun rationalize-units (dimel)
  "Find a common denominator of the unit exponents and
   return the integer dimel with the common denominator."
  (let ((fracs (remove-if 'cl:integerp dimel)))
    (if fracs
	(let ((lcm (apply #'lcm (mapcar #'denominator fracs))))
	  (values (* lcm dimel) lcm))
      (values dimel 1))))

;;; (make-ue '(2 -4 3 0 0 0 0 0 0))
;;; (NEWTON 2 KILOGRAM 1)
;;; (make-ue '(1/2 0 0 0 0 0 0 0 0))
(defun make-ue (dimel)
  "From the dimel (list of dimension exponents),
   make an unit-exponent list."
  (check-dimel dimel)
  (multiple-value-bind (dimelint cd)
      (rationalize-units dimel)
    (destructuring-bind (unit expon remainder)
	(find-units dimelint)
      (if (eq unit 'unity)
	  nil
	  (if (equal remainder dimelint)
	      (error "Dimensions cannot be interpreted in this system of units.")
	      (append (list unit (/ expon cd))
		      (make-ue (/ remainder cd))))))))

;;; This is the inverse of parse-unit-sexp
;;; (make-unit-sexp '(2 -4 3 0 0 0 0 0 0) t)
;;; (* (EXPT NEWTON 2) KILOGRAM)
;;; (make-unit-sexp '(-2 1 1 0 0 0 0 0 0) t)
;;; (/ (* KILOGRAM SECOND) (EXPT METER 2))
;;; (make-unit-sexp '(1/2 0 0 0 0 0 0 0 0) t)
(defun make-unit-sexp (dimel scalar-dimensions)
  "From the dimel (list of dimension exponents),
   make an unit sexp."
  (if scalar-dimensions
      (flet ((prod (lst)
	       (if (null lst)
		   nil
		   (if (single lst) (first lst) (cons '* lst))))
	     (pwr (name expon) (if (= expon 1) name `(expt ,name ,expon))))
	(when dimel
	  (check-dimel dimel)
	  (loop for (unit expon) on (make-ue dimel) by #'cddr
		when (plusp expon)
		  collect (pwr unit expon) into num
		when (minusp expon)
		  collect (pwr unit (- expon)) into den
		finally
	     (return
	       (if (null den)
		   (prod num)
		   `(/ ,(if (null num) 1 (prod num)) ,(prod den)))))))
      (let ((fn (grid:elementwise (alexandria:rcurry #'make-unit-sexp t))))
	(funcall fn dimel))))

;;; (find-unit-expr 'speed)
;;; (/ JGM2-EARTH-RADIUS HERG)
(defun find-unit-expr (unit-expr)
  "Find the unit name or expression."
  (multiple-value-bind (dimel conv)
      (parse-unit-sexp unit-expr)
    (if conv
	unit-expr
      (make-unit-sexp dimel t))))

;;;;****************************************************************************
;;;; Formatting
;;;;****************************************************************************

;;; (make-pq-string '(/ (* M KG KG) (* S S)))
;;; "M-KG^2/S^2"
(defun make-pq-string (unit-sexp &optional style)
  "Make a string in a customary form for unit expressions from the sexp."
  (cond ((null unit-sexp) "")
	((symbolp unit-sexp)
	 ;; Use abbreviations; counts on first synonym being the preferred
	 ;; (usually shortes) abbreviation.
	 (or (if (print-name unit-sexp) ; if there's a print-name, use it
		 (if (listp (print-name unit-sexp))
		     (if style (second (print-name unit-sexp))
			 (first (print-name unit-sexp))) ; plain or TeX
		     (print-name unit-sexp)))
	     (string-downcase	   ; or down case
	      (or		   ; the first synonym (usu. shortest)
	       (first (synonyms (get-canonical-name unit-sexp)))
	       unit-sexp))))		; last resort symbol itself.
	((numberp unit-sexp) (princ-to-string unit-sexp))
	((and (listp unit-sexp) (member (first unit-sexp) '(/ cl:/)))
	 ;; quotient of units
	 (mkstr (if (eql (second unit-sexp) 1)
		    ""
		    (make-pq-string (second unit-sexp) style))
		"/"
		(make-pq-string (third unit-sexp) style)))
	((and (listp unit-sexp) (member (first unit-sexp) '(expt cl:expt)))
	 ;; exponent of units
	 ;; base of exponent should be a symbol, or string will not look right
	 (mkstr (make-pq-string (second unit-sexp) style)
		"^"
		(third unit-sexp)))
	(t
	 ;; multiplied units; run length encode
	 (let* ((prodlist		; list of unit without "*"
		 (if (member (first unit-sexp) '(* cl:*)) (rest unit-sexp) unit-sexp))
		(posn (position (first prodlist) prodlist :test-not #'eql))
		(first-expon (or posn (length prodlist))))
	   (mkstr
	    (format nil "~a~@[^~d~]~:[~;-~]"
		    (make-pq-string (first prodlist) style)
		    (unless (= first-expon 1) first-expon)
		    posn)
	    (make-pq-string (subseq prodlist first-expon)))))))

;;;;****************************************************************************
;;;; Defining dimensions
;;;;****************************************************************************

(defun def-dimensionp (symbol)
  "Define the predicate and type to check a particular dimension."
  ;; The generated predicate function symbols aren't exported because
  ;; the user is expected to use typep with the defined type.
  (let ((fnname (alexandria:symbolicate symbol "P")))
    (setf (fdefinition fnname)
	  (lambda (x) (check-dimension x symbol nil t)))
    (eval `(deftype ,symbol () '(satisfies ,fnname)))))

(defun define-basis-dimensions ()
  "Define the dimel for the dimensions in *basis-dimensions*."
  (setf (dimension 'dimensionless) (make-new-dimel)
	(canonical-name 'dimensionless) 'dimensionless)
  (dolist (dim *basis-dimensions*)
    (def-dimensionp dim)
    (setf (dimension dim)
      (let ((dimel (make-new-dimel)))
	(setf (elt dimel (position dim *basis-dimensions* :test #'string-equal))
	  1)
	dimel)
      (canonical-name dim) dim)))

(defun define-derived-dimensions (name-def-pairs)
  "Define the dimel given in the name-def-pairs, where the definition
   is in terms of any already-defined physical dimension."
  (dolist (pair name-def-pairs)
    (destructuring-bind (name def) pair
      (define-unit name def)
      (def-dimensionp name)
      (setf (siconversion name) nil))))

;;;;****************************************************************************
;;;; Defining units
;;;;****************************************************************************

(defun check-uname (new name)
  (when (canonical-name name)
    (unless (eql new (canonical-name name)) ; don't error out if we're re-evaluating the same form
      (cerror "Change definition."
	      "Attempting to redefine ~a to mean ~a, but it already means ~a."
	      name new (canonical-name name)))))

(defun check-unames (new names)
  (check-uname new new)
  (dolist (s names) (check-uname new s)))

(defun define-unit (unit definition &optional synonyms print-name dimension (rationalize t))
  "Define the unit.  If the definition is a number then the dimension
   information will be extracted from dimension."
  ;; Check if name/synonym is already defined.
  (export unit (symbol-package unit))
  (check-unames unit synonyms)
  (multiple-value-bind (derived-dim conversion)
      (if (and dimension (numberp definition))
	  (values (parse-unit-sexp dimension) definition)
	  (parse-unit-sexp definition))
    (setf (dimension unit)
	  ;; if dimension is given, use it; otherwise, derive from expression
	  (if dimension (parse-unit-sexp dimension) derived-dim)
	  (siconversion unit)
	  (if conversion
	      (if rationalize
		  (rationalize conversion)
		  conversion)
	      1)
	  (synonyms unit) (pushnew (unit-remove-illegal-chars unit) synonyms)
	  (print-name unit) print-name
	  (canonical-name unit) unit)
    (dolist (syn synonyms)
      (export syn (symbol-package syn))
      (setf (canonical-name syn) unit))))

(defun define-units (dimension unit-def-syn-pnm)
  "Define the units given.  The second argument is a list of lists, each
   consisting of a unit name, its definition, and a list of synonyms."
  (dolist (uda unit-def-syn-pnm)
    (destructuring-bind (units definition &optional synonyms print-name (rationalize t))
	uda
      (define-unit units definition synonyms print-name dimension rationalize))))

(defparameter *unit-symbol-macros* nil
  "Unit symbols that were defined with unit-symbol-macro.")

(defmacro unit-symbol-macro (unit &optional (multiplier 1) prefix)
  "Define a symbol macro to expand to the given unit."
  (let ((sm (symb (or prefix "") unit)))
    `(progn
      (define-symbol-macro
	  ,sm
	  (pqwu (make-pq ,multiplier ',(unit-remove-illegal-chars unit))))
      (pushnew ',sm *unit-symbol-macros*))))

(defmacro define-physical-constant
    (name value &optional synonyms print-name docstring)
  "Define the variable to be the physical dimension quantity given, and also make it a unit of measure."
  (declare (ignore docstring))
  `(progn
    (define-unit
	',name
	(multiple-value-bind (mag unit)
	    (pqval ,value)
	  `(* ,mag ,unit))
      ',synonyms ',print-name)
    (unit-symbol-macro ,name)))

#+example
(define-physical-constant err ; JGM2-earth-rotation-rate
    (make-pq 7.2921158553d-5 '(/ radians second)) (earthav)
  "JGM2 rotation of the earth in radians/solar second
   from back cover of Vallado.")
