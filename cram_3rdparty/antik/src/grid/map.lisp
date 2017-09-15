;; Map and old grid to a new grid
;; Liam Healy 2009-11-15 18:05:16EST map.lisp
;; Time-stamp: <2014-03-15 12:54:18EDT map.lisp>
;;
;; Copyright 2009, 2010, 2012, 2013, 2014 Liam M. Healy
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

;; Inspired by Tamas Papp's map-subarray.
;; For examples, see grid/tests/.

(in-package :grid)

(export '(map-grid map-n-grids elementwise))

(defun make-destination-affi (destination destination-specification source-affi)
  (if destination
      (affi destination)
      (if destination-specification
	  (affi:make-affi (specification-dimensions destination-specification))
	  source-affi)))

(defun make-destination (affi specification initial-element)
  "Make the grid based on the specification, modified with the
   affi given.  It will be created with initial-element if non-nil."
  (apply
   'make-grid
   (merge-specification specification nil (coerce (affi:get-domain affi) 'list))
   (when initial-element
     (list
      :initial-element
      (coerce-value initial-element (spec-element-type specification))))))

;;; Elementwise mapping over multiple grids.
(defun map-n-grids
    (&key sources
       ;; destination grid information
       destination destination-affi destination-specification
       initial-element			; default value
       ;; functions to apply
       (element-functions (make-list (length sources) :initial-element nil))
       combination-function
       (combine-destination (or initial-element destination)))
  "Map on multiple source grids.  The arguments are:
   sources:
     A list of (grid-or-function affi), each one representing
     either a grid or function from which data will be drawn.  
     If it is a grid, the grid values are used.  If it is a function
     of the appropriate number of non-negative integer arguments,
     values will be created by calling that function on the indices.
     If affi is not specified and grid-or-function is a grid, the affi
     will default to the AFFI of that grid.
   destination:
     A grid in which to put the results, or nil if one is
     to be created.
   destination-affi:
     An AFFI for the destinattion.
   destination-specification:
     A specification for the destination, if destination-affi
     is not given.
   initial-element:  Initial value to give to the destination grid.
   element-functions:
     A list of functions or nil, one for each source.  This function
     is applied to the source element before combination.
   combination-function:
     A function that combines each of the source values after
     being passed through the element-functions.  If nil,
     then just take the value from the first source.
   combine-destination:
     If true, pass as the first argument to the combination-function
     the prior value of the destination grid.  Defaults to T if
     :desination is specified; otherwise nil."
  (let ((sources		    ; Warning! sources being redefined
	  (loop for source in sources
		collect
		(cons
		 (if (and (first source) (not (gridp (first source))))
		     nil
		     (first source))
		 (rest source))))
	(index-functions ; Separate out the functions (if any) in the sources
	  (loop for source in sources
		collect
		(when (and (first source) (not (gridp (first source))))
		  (first source)))))
    (dolist (source sources)		; set all the source affis
      (unless (second source) (setf (second source) (affi (first source)))))
    (unless destination-affi
      (setf destination-affi
	    (make-destination-affi
	     destination destination-specification
	     (second (first sources)))))
    (unless destination
      (setf destination
	    (make-destination
	     destination-affi
	     (or destination-specification
		 (let ((source-objs (mapcar #'first sources)))
		   (alexandria:if-let
		       (source-grid-pos (position-if #'gridp source-objs))
		     ;; Find the first grid in the sources on which to model the destination grid
		     (specification (nth source-grid-pos source-objs))
		     (make-specification *default-grid-type* *default-dimensions* *default-element-type*))))
	     initial-element)))
    (let* ((source-walkers
	     (mapcar (lambda (s) (affi:make-walker (second s))) sources))
	   source-inds
	   (dest-walker (affi:make-walker destination-affi))
	   (destination-index nil))
      (block loop
	(loop
	  (setf source-inds
		(loop for w in source-walkers
		      for inds = (funcall w)
		      unless inds do (return-from loop)
			collect inds)
		destination-index
		(or (funcall dest-walker) (return-from loop))
		(grid:aref* destination destination-index)
		(let ((tsrcs
			(mapcar
			 (lambda (element-function index-function source source-index)
			   (funcall
			    (or element-function
				#'(lambda (elt)
				    (coerce-value elt (element-type destination))))
			    (if index-function
				(apply index-function
				       (affi::delinearize-index (second source) source-index))
				(grid:aref* (first source) source-index))))
			 element-functions
			 index-functions
			 sources
			 source-inds)))
		  (if combination-function ; must have combination-function if >1 source
		      (if combine-destination
			  (apply combination-function
				 (grid:aref* destination destination-index) tsrcs)
			  (apply combination-function tsrcs))
		      (first tsrcs)))))))
    destination))

(defun map-grid
    (&key source source-affi source-dims ; source grid information
     ;; destination grid information
     destination destination-affi destination-specification
     initial-element			    ; default value
     element-function combination-function) ; functions to apply
  "Make a new grid by mapping on an existing grid or on indices.
   :source            
      The source grid; if not a grid, it is taken as a
      function to apply to the grid indices to make the
      element of the destination; if this is supplied,
      source is ignored and element-function is only applied
      to the default value.
   :source-affi       
      The affi to be used for extraction; defaults to
      making an affi from source-dims, then destination-specification. 
   :source-dims       
      The dimensions of the source if source not supplied;
      if NIL, dimensions are taken from destination-specification.
   :destination       
      The destination grid, if not supplied, it will be made
      according to destination-affi and destination-specification.
   :destination-affi
      The affi for injection, defaults to (affi destination) 
      if destination is supplied, then
      makes an affi if destination-specification is supplied,
      otherwise source-affi.
   :destination-specification 
      The specification to use for the destination to be make,
      defaults to the specification of source.
   :initial-element     
      The default value to set in a newly-created destination.
   :element-function  
      The function to apply to each element of the source; defaults
      to coercing element to the element type of the destination.
   :combination-function
      A designator for a function of two arguments,
      or nil (default).  If a function, it will be funcalled on
      the destination element and the transformed source element.
      If nil, the destination element is overwritten."
  (map-n-grids :sources
	       (list (list
		      source
		      (or source-affi
			  (when (or source-dims destination-specification)
			    (affi:make-affi
			     (or source-dims
				 (specification-dimensions destination-specification)))))))
	       :destination destination
	       :destination-affi destination-affi
	       :destination-specification destination-specification
	       :initial-element initial-element
	       :element-functions (list element-function)
	       :combination-function combination-function))

(defun (setf linear-data) (value grid)
  "Set the contents of the grid according to the value, which
   is a sequence."
  (map-grid :source (coerce value 'vector)
	    :destination grid))

(defun linear-data (grid &optional (type *default-grid-type*))
  "The contents of the grid as vector."
  (map-grid :source grid
	    :destination-specification
	    (merge-specification
	     (specification grid) type (list (affi:size (affi grid))))))

(defun elementwise (function &optional toggle-physical-dimension)
  "Make a function on a grid as an elementwise map of a scalar function.  If the result has no physical dimension but the argument does, or vice versa, toggle-physical-dimension should be T.
@example
(funcall (grid:elementwise 'sqrt) #(1.0d0 2.0d0 3.0d0 4.0d0))
#(1.0 1.4142135623730951 1.7320508075688772 2.0)
@end example"
  ;; This could probably be generalized, but it's needed in this form
  ;; for #'normalize.
  (lambda (x)
    (map-grid
     :source x
     :destination-specification
     (when toggle-physical-dimension
       (if (antik:physical-quantity-p x)
	   (specification (antik::pq-magnitude x))
	   (pdq-grid-specification (specification x))))
     :element-function function)))
