;; Wavelet transforms.
;; Liam Healy, Mon Nov 26 2007 - 20:43
;; Time-stamp: <2010-07-07 14:24:54EDT wavelet.lisp>
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

;;; /usr/include/gsl/gsl_wavelet.h

;;; Examples do not agree with C results.

;;;;****************************************************************************
;;;; Allocation of wavelets
;;;;****************************************************************************

;;; Utility
(defun forward-backward (symb)
  (if (string-equal symb :backward) -1 1))

;;;;****************************************************************************
;;;; Allocation of wavelets
;;;;****************************************************************************

(defmobject wavelet "gsl_wavelet"
  ((type :pointer) (member sizet))
  "wavelet"
  :documentation			; FDL
  "Make and initialize a wavelet object of type 'type.  The
   parameter 'member selects the specific member of the wavelet
   family.  A memory-allocation-failure error indicates either
   lack of memory or an unsupported member requested.")

(defmpar +daubechies-wavelet+ "gsl_wavelet_daubechies"
  ;; FDL
  "The Daubechies wavelet family of maximum phase with member/2
   vanishing moments.  The implemented wavelets are 
   member=4, 6,..., 20, with member even.")

(defmpar +daubechies-wavelet-centered+ "gsl_wavelet_daubechies_centered"
  ;; FDL
  "The Daubechies wavelet family of maximum phase with member/2
   vanishing moments.  The implemented wavelets are 
   member=4, 6,..., 20, with member even.")

(defmpar +haar-wavelet+ "gsl_wavelet_haar"
  ;; FDL
  "The Haar wavelet.  The only valid choice for member for the
   Haar wavelet is member=2.")

(defmpar +haar-wavelet-centered+ "gsl_wavelet_haar_centered"
  ;; FDL
  "The Haar wavelet.  The only valid choice for member for the
   Haar wavelet is member=2.")

(defmpar +bspline-wavelet+ "gsl_wavelet_bspline"
  ;; FDL
  "The biorthogonal B-spline wavelet family of order (i,j).  
   The implemented values of member = 100*i + j are 103, 105, 202, 204,
   206, 208, 301, 303, 305 307, 309.")

(defmpar +bspline-wavelet-centered+ "gsl_wavelet_bspline_centered"
  ;; FDL
  "The biorthogonal B-spline wavelet family of order (i,j).  
   The implemented values of member = 100*i + j are 103, 105, 202, 204,
   206, 208, 301, 303, 305 307, 309.")

(defmfun name ((wavelet wavelet))
  "gsl_wavelet_name"
  (((mpointer wavelet) :pointer))
  :definition :method
  :c-return :string
  :documentation			; FDL
  "The name of the wavelet family.")

(defmobject wavelet-workspace
  "gsl_wavelet_workspace"
  ((size sizet))
  "wavelet workspace"
  :documentation			; FDL
  "Make a workspace for the discrete wavelet transform.
   To perform a one-dimensional transform on size elements, a workspace
   of size size must be provided.  For two-dimensional transforms of
   size-by-size matrices it is sufficient to allocate a workspace of
   size, since the transform operates on individual rows and
   columns.")

;;;;****************************************************************************
;;;; Wavelet transforms 1D
;;;;****************************************************************************

(defmfun wavelet-transform (wavelet data stride direction workspace)
  "gsl_wavelet_transform"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   (stride sizet) ((dim0 data) sizet) ((forward-backward direction) :int)
   ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute in-place forward and inverse discrete wavelet
   transforms on the array data. The length of the
   transform n is restricted to powers
   of two.  For the transform version of the function the argument
   dir can be either :forward or :backward.  A workspace
   of the same length as data must be provided.
   For the forward transform, the elements of the original array are 
   replaced by the discrete wavelet
   transform f_i -> w_{j,k}
   in a packed triangular storage layout, 
   where j is the index of the level j = 0 ... J-1
   and k is the index of the coefficient within each level,
   k = 0 ... (2^j)-1.  The total number of levels is J = \log_2(n).
   The output data has the following form,
   (s_{-1,0}, d_{0,0}, d_{1,0}, d_{1,1}, d_{2,0},\cdots, d_{j,k},\cdots, d_{J-1,2^{J-1} - 1}) 
   where the first element is the smoothing coefficient
   s_{-1,0}, followed by the detail coefficients d_{j,k} for each level j.
   The backward transform inverts these coefficients to obtain 
   the original data.")

(defmfun wavelet-transform-forward (wavelet data stride workspace)
  "gsl_wavelet_transform_forward"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   (stride sizet) ((dim0 data) sizet) ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute in-place forward and inverse discrete wavelet
   transforms on the array data.  The length of the transform
   is restricted to powers of two.
   A workspace of the same length as data must be provided.
   For the forward transform, the elements of the original array are 
   replaced by the discrete wavelet transform
   f_i -> w_{j,k} in a packed triangular storage layout, 
   where j is the index of the level j = 0 ... J-1
   and k is the index of the coefficient within each level,
   k = 0 ... (2^j)-1.  The total number of levels is J = \log_2(n).
   The output data has the following form,
   (s_{-1,0}, d_{0,0}, d_{1,0}, d_{1,1}, d_{2,0},\cdots, d_{j,k},\cdots, d_{J-1,2^{J-1} - 1}) 
   where the first element is the smoothing coefficient s_{-1,0},
   followed by the detail coefficients d_{j,k} for each level j.
   The backward transform inverts these coefficients to obtain 
   the original data.")

(defmfun wavelet-transform-inverse (wavelet data stride workspace)
  "gsl_wavelet_transform_inverse"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   (stride sizet) ((dim0 data) sizet) ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute in-place inverse discrete wavelet
   transforms on the array data.  The length of the transform
   is restricted to powers of two.
   A workspace of the same length as data must be provided.
   For the forward transform, the elements of the original array are 
   replaced by the discrete wavelet transform
   f_i -> w_{j,k} in a packed triangular storage layout, 
   where j is the index of the level j = 0 ... J-1
   and k is the index of the coefficient within each level,
   k = 0 ... (2^j)-1.  The total number of levels is J = \log_2(n).
   The output data has the following form,
   (s_{-1,0}, d_{0,0}, d_{1,0}, d_{1,1}, d_{2,0},\cdots, d_{j,k},\cdots, d_{J-1,2^{J-1} - 1}) 
   where the first element is the smoothing coefficient s_{-1,0},
   followed by the detail coefficients d_{j,k} for each level j.
   The backward transform inverts these coefficients to obtain 
   the original data.")

;;;;****************************************************************************
;;;; Wavelet transforms 2D
;;;;****************************************************************************

(defmfun wavelet-2d-transform (wavelet data tda direction workspace)
  "gsl_wavelet2d_transform"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   (tda sizet) ((dim0 data) sizet) ((dim1 data) sizet)
   ((forward-backward direction) :int) ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute in-place forward and inverse discrete wavelet transforms
  in standard and non-standard forms on the
  array data stored in row-major form with dimensions
  and physical row length tda.  The dimensions must
  be equal (square matrix) and are restricted to powers of two.  For the
  transform version of the function the argument direction can be
  either :forward or :backward.  A
  workspace of the appropriate size must be provided.  On exit,
  the appropriate elements of the array data are replaced by their
  two-dimensional wavelet transform.
  An error invalid-argument is signalled if the matrix is not square
  with dimension a power of 2, or if insufficient
  workspace is provided.")

(defmfun wavelet-2d-transform-forward (wavelet data tda workspace)
  "gsl_wavelet2d_transform_forward"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   (tda sizet) ((dim0 data) sizet) ((dim1 data) sizet)
   ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute two-dimensional in-place forward and inverse
  discrete wavelet transforms in standard and non-standard forms on the
  array data stored in row-major form with dimensions size1
  and size2 and physical row length tda.  The dimensions must
  be equal (square matrix) and are restricted to powers of two.  A
  workspace of the appropriate size must be provided.  On exit,
  the appropriate elements of the array data are replaced by their
  two-dimensional wavelet transform.
  An error invalid-argument is signalled if the matrix is not square
  with dimension a power of 2, or if insufficient
  workspace is provided.")

(defmfun wavelet-2d-transform-inverse (wavelet data tda workspace)
  "gsl_wavelet2d_transform_inverse"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   (tda sizet) ((dim0 data) sizet) ((dim1 data) sizet)
   ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute two-dimensional in-place forward and inverse discrete
   wavelet transforms in standard and non-standard forms on the array
   data stored in row-major form with dimensions size1 and size2 and
   physical row length tda.  The dimensions must be equal (square matrix)
   and are restricted to powers of two.  A workspace of the appropriate
   size must be provided.  On exit, the appropriate elements of the array
   data are replaced by their two-dimensional wavelet transform.  An
   error invalid-argument is signalled if the matrix is not square with dimension
   a power of 2, or if insufficient workspace is provided.")

(defmfun wavelet-2d-transform-matrix (wavelet data direction workspace)
  "gsl_wavelet2d_transform_matrix"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   ((forward-backward direction) :int) ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute the two-dimensional in-place wavelet transform on a matrix.")

(defmfun wavelet-2d-transform-matrix-forward (wavelet data workspace)
  "gsl_wavelet2d_transform_matrix_forward"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute the two-dimensional in-place wavelet transform on a matrix.")

(defmfun wavelet-2d-transform-matrix-inverse (wavelet data workspace)
  "gsl_wavelet2d_transform_matrix_inverse"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute the two-dimensional in-place wavelet transform on a matrix.")

(defmfun wavelet-2d-nonstandard-transform (wavelet data tda direction workspace)
  "gsl_wavelet2d_nstransform"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   (tda sizet) ((dim0 data) sizet) ((dim1 data) sizet) (direction :int)
   ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute the two-dimensional wavelet transform in non-standard form.")

(defmfun wavelet-2d-nonstandard-transform-forward (wavelet data tda workspace)
  "gsl_wavelet2d_nstransform_forward"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   (tda sizet) ((dim0 data) sizet) ((dim1 data) sizet)
   ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute the two-dimensional wavelet transform in non-standard form.")

(defmfun wavelet-2d-nonstandard-transform-inverse (wavelet data tda workspace)
  "gsl_wavelet2d_nstransform_inverse"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   (tda sizet) ((dim0 data) sizet) ((dim1 data) sizet)
   ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute the two-dimensional wavelet transform in non-standard form.")

(defmfun wavelet-2d-nonstandard-transform-matrix (wavelet data direction workspace)
  "gsl_wavelet2d_nstransform_matrix"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   (direction :int) ((mpointer workspace) :pointer))
  :documentation
  "Compute the non-standard form of the two-dimensional in-place wavelet
   transform on a matrix.")

(defmfun wavelet-2d-nonstandard-transform-matrix-forward (wavelet data workspace)
  "gsl_wavelet2d_nstransform_matrix_forward"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute the non-standard form of the two-dimensional in-place wavelet
   transform on a matrix.")

(defmfun wavelet-2d-nonstandard-transform-matrix-inverse (wavelet data workspace)
  "gsl_wavelet2d_nstransform_matrix_inverse"
  (((mpointer wavelet) :pointer) ((foreign-pointer data) :pointer)
   ((mpointer workspace) :pointer))
  :documentation			; FDL
  "Compute the non-standard form of the two-dimensional in-place wavelet
   transform on a matrix.")

;;;;****************************************************************************
;;;; Example
;;;;****************************************************************************

;;; See GSL manual Section 30.4.
(defparameter *wavelet-sample*
 #m(0.0462458471760794d0 0.0462458471760794d0 0.0512458471760794d0 0.0712458471760795d0
 0.0712458471760795d0 0.0662458471760795d0 0.0962458471760795d0 0.101245847176079d0
 0.116245847176079d0 0.121245847176079d0 0.116245847176079d0 0.106245847176079d0
 0.0912458471760794d0 0.101245847176079d0 0.0962458471760795d0 0.0962458471760795d0
 0.0962458471760795d0 0.0912458471760794d0 0.0862458471760795d0 0.0812458471760795d0
 0.0862458471760795d0 0.101245847176079d0 0.111245847176079d0 0.116245847176079d0
 0.0762458471760795d0 0.0362458471760795d0 0.0362458471760795d0 0.0212458471760795d0
 0.0112458471760795d0 -0.00875415282392056d0 -0.00875415282392056d0 -0.00375415282392055d0
 0.00624584717607946d0 0.00124584717607945d0 0.00624584717607946d0 -0.00375415282392055d0
 -0.0187541528239206d0 -0.0237541528239205d0 -0.0187541528239206d0 -0.0187541528239206d0
 -0.0287541528239205d0 -0.0237541528239205d0 -0.0337541528239205d0 -0.00875415282392056d0
 -0.0137541528239206d0 -0.00875415282392056d0 0.00124584717607945d0 -0.0237541528239205d0
 -0.0337541528239205d0 -0.0187541528239206d0 -0.00875415282392056d0 -0.00375415282392055d0
 -0.00875415282392056d0 -0.0287541528239205d0 -0.0437541528239205d0 -0.0387541528239205d0
 -0.0587541528239205d0 -0.103754152823921d0 -0.123754152823921d0 -0.153754152823921d0
 -0.188754152823921d0 -0.213754152823921d0 -0.183754152823921d0 -0.0937541528239205d0
 0.0212458471760795d0 0.161245847176079d0 0.306245847176079d0 0.556245847176079d0
 0.81124584717608d0 1.04124584717608d0 1.19624584717608d0 1.26124584717608d0
 1.22624584717608d01 1.07624584717608d0 0.81124584717608d0 0.486245847176079d0
 0.211245847176079d0 0.0512458471760794d0 -0.0687541528239206d0 -0.128754152823921d0
 -0.153754152823921d0 -0.133754152823921d0 -0.103754152823921d0 -0.0687541528239206d0
 -0.0687541528239206d0 -0.0637541528239206d0 -0.0687541528239206d0 -0.0587541528239205d0
 -0.0587541528239205d0 -0.0587541528239205d0 -0.0737541528239206d0 -0.0637541528239206d0
 -0.0637541528239206d0 -0.0637541528239206d0 -0.0537541528239205d0 -0.0737541528239206d0
 -0.0887541528239205d0 -0.0887541528239205d0 -0.0787541528239206d0 -0.0737541528239206d0
 -0.0687541528239206d0 -0.0837541528239206d0 -0.0737541528239206d0 -0.0637541528239206d0
 -0.0537541528239205d0 -0.0687541528239206d0 -0.0687541528239206d0 -0.0837541528239206d0
 -0.0887541528239205d0 -0.0887541528239205d0 -0.0687541528239206d0 -0.0687541528239206d0
 -0.0737541528239206d0 -0.0837541528239206d0 -0.0937541528239205d0 -0.0787541528239206d0
 -0.0887541528239205d0 -0.0837541528239206d0 -0.0887541528239205d0 -0.0937541528239205d0
 -0.0887541528239205d0 -0.0787541528239206d0 -0.0787541528239206d0 -0.0737541528239206d0
 -0.0687541528239206d0 -0.0837541528239206d0 -0.0887541528239205d0 -0.0687541528239206d0
 -0.0687541528239206d0 -0.0637541528239206d0 -0.0637541528239206d0 -0.0887541528239205d0
 -0.0837541528239206d0 -0.0737541528239206d0 -0.0687541528239206d0 -0.0537541528239205d0
 -0.0687541528239206d0 -0.0737541528239206d0 -0.0887541528239205d0 -0.0787541528239206d0
 -0.0687541528239206d0 -0.0687541528239206d0 -0.0637541528239206d0 -0.0837541528239206d0
 -0.0937541528239205d0 -0.0937541528239205d0 -0.0787541528239206d0 -0.0737541528239206d0
 -0.0837541528239206d0 -0.0937541528239205d0 -0.0987541528239205d0 -0.0987541528239205d0
 -0.0887541528239205d0 -0.0937541528239205d0 -0.103754152823921d0 -0.0987541528239205d0
 -0.113754152823921d0 -0.108754152823921d0 -0.108754152823921d0 -0.0987541528239205d0
 -0.108754152823921d0 -0.128754152823921d0 -0.133754152823921d0 -0.128754152823921d0
 -0.113754152823921d0 -0.123754152823921d0 -0.128754152823921d0 -0.133754152823921d0
 -0.148754152823921d0 -0.138754152823921d0 -0.133754152823921d0 -0.128754152823921d0
 -0.133754152823921d0 -0.148754152823921d0 -0.153754152823921d0 -0.138754152823921d0
 -0.128754152823921d0 -0.123754152823921d0 -0.118754152823921d0 -0.113754152823921d0
 -0.118754152823921d0 -0.0887541528239205d0 -0.0737541528239206d0 -0.0487541528239205d0
 -0.0437541528239205d0 -0.0387541528239205d0 -0.0437541528239205d0 -0.0187541528239206d0
 -0.00375415282392055d0 0.00624584717607946d0 0.00124584717607945d0 -0.00875415282392056d0
 -0.00875415282392056d0 0.00124584717607945d0 0.0112458471760795d0 0.0212458471760795d0
 0.0212458471760795d0 0.00124584717607945d0 0.00124584717607945d0 0.00624584717607946d0
 0.0162458471760795d0 0.0162458471760795d0 0.0262458471760795d0 0.00124584717607945d0
 -0.00875415282392056d0 0.0162458471760795d0 0.0112458471760795d0 0.0212458471760795d0
 0.0212458471760795d0 0.00124584717607945d0 -0.00375415282392055d0 0.0112458471760795d0
 0.0162458471760795d0 0.00624584717607946d0 0.0162458471760795d0 0.00624584717607946d0
 0.00624584717607946d0 0.0112458471760795d0 0.0262458471760795d0 0.0312458471760795d0
 0.0162458471760795d0 0.0112458471760795d0 0.00124584717607945d0 0.00624584717607946d0
 0.0212458471760795d0 0.00624584717607946d0 0.00624584717607946d0 0.00624584717607946d0
 -0.00875415282392056d0 0.00624584717607946d0 0.00124584717607945d0 0.00624584717607946d0
 -0.00375415282392055d0 -0.0137541528239206d0 -0.0187541528239206d0 -0.0137541528239206d0
 -0.0137541528239206d0 -0.00875415282392056d0 -0.00375415282392055d0 -0.0237541528239205d0
 -0.0287541528239205d0 -0.0237541528239205d0 -0.0137541528239206d0 -0.00875415282392056d0
 -0.00875415282392056d0 -0.0237541528239205d0 -0.0237541528239205d0 -0.0237541528239205d0
 0.00124584717607945d0 -0.00875415282392056d0 -0.0137541528239206d0 -0.0187541528239206d0
 -0.0337541528239205d0 -0.0137541528239206d0 -0.00875415282392056d0 -0.00875415282392056d0)
  "Data for example wavelet transform from doc/examples/ecg.dat.")

;;; These examples do not agree with their C counterparts; the answers
;;; are completely different.

(defun wavelet-example (&optional (cl-data *wavelet-sample*))
  "Demonstrates the use of the one-dimensional wavelet transform
   functions. It computes an approximation to an input signal (of length
   256) using the 20 largest components of the wavelet transform, while
   setting the others to zero.  See GSL manual Section 30.4."
  (let* ((n (length cl-data))
	 (vector cl-data)
	 (wavelet (make-wavelet +daubechies-wavelet+ 4))
	 (workspace (make-wavelet-workspace n)))
    (wavelet-transform-forward wavelet vector 1 workspace)
    (let ((absvector (grid:make-foreign-array 'double-float :dimensions n))
	  (permutation (make-permutation n)))
      (dotimes (i n)
	(setf (grid:gref absvector i) (abs (grid:gref vector i))))
      ;; Sort and set to 0 all but the largest 20.
      (sort-vector-index permutation absvector)
      (dotimes (i (- n 20))
	(setf (grid:gref vector (grid:gref permutation i))
	      0.0d0))) ;; Transform back
    (dotimes (i n) (format t "~&~a" (grid:gref vector i)))
    (wavelet-transform-inverse wavelet vector 1 workspace)
    (grid:copy-to vector)))

(defun wavelet-forward-example (&optional (cl-data *wavelet-sample*))
  "Simpler example, with only a Daubechies wavelet forward transformation."
  (let* ((n (length cl-data))
	 (vector cl-data)
	 (wavelet (make-wavelet +daubechies-wavelet+ 4))
	 (workspace (make-wavelet-workspace n)))
    (wavelet-transform-forward wavelet vector 1 workspace)
    (grid:copy-to vector)))
