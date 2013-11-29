(in-package :affi)

(declaim (optimize (debug 3)))

(defclass affi ()
  ;; !!! can constrain types
  ((const :initarg :const)
   (coeff :initarg :coeff)
   (domain :initarg :domain :reader domain)))

(defmethod initialize-instance :after ((affi affi) &key)
  ;; basic consistency check
  (with-slots (const coeff domain) affi
    (assert (and (integerp const)
		 (typep coeff '(simple-array fixnum (*)))
		 (typep domain '(simple-array fixnum (*)))
		 (= (length coeff) (length domain))
		 ;;(every #'plusp domain)
		 ))))
  
(defun get-const (affi)
  "Return the constant in an affine index."
  (slot-value affi 'const))

(defun get-coeff (affi)
  "Return the coefficients in an affine index (copy is made)."
  (copy-into-fixnum-vector (slot-value affi 'coeff)))

(defun get-domain (affi)
  "Return the domain in an affine index (copy is made)."
  (copy-into-fixnum-vector (domain affi)))

(defun copy-affi (affi)
  (make-instance
   'affi
   :const (get-const affi)
   :coeff (get-coeff affi)
   :domain (get-domain affi)))

(defmethod print-object ((obj affi) stream)
  (with-slots (const coeff domain) obj
    (print-unreadable-object (obj stream :type t :identity t)
      (format stream "domain ~a, const ~a, coeff ~a" domain const coeff))))

(defun rank (affi)
  "Return the rank of an affine index."
  (length (domain affi)))

(defun size (affi)
  "Return the size (ie number of integers in the range) of an affine
index.  Note that size is not necessarily the difference of the
endpoints of the range, as the range may be non-contiguous."
  (with-slots (domain) affi
    (grid::total-size-from-dimension domain)))
	      
(defun range (affi)
  "Return the smallest and the largest integer in the range of an
affine index."
  (with-slots (const coeff domain) affi
    (let ((min const)
	  (max const))
      (loop
	 for c across coeff
	 for d across domain
	 when (plusp c)
	 sum (* c (1- d)) into max
	 when (minusp c)
	 sum (* c (1- d)) into min)
      (values min max))))

(defgeneric make-affi (object &optional offset) 
  (:documentation "Create an affine mapping conforming to the given
  object."))
      
(defun make-affi-int (dimensions coeff &optional (offset 0))
  "Make an affi instance with the contents not set, except for dimensions."
  (let* ((dimensions (copy-into-fixnum-vector dimensions))
	 (n (length dimensions)))
    (assert (plusp n))
    (make-instance
     'affi
     :const offset
     :coeff (copy-into-fixnum-vector coeff)
     :domain dimensions)))

(defun row-major-coeff-from-dimensions (dimensions)
  "Find the coeff from the dimensions."
  (let* ((rank (length dimensions))
	 (coeff (make-list rank :initial-element 0)))
    (loop with p = 1
       for i :from (1- rank) :downto 0
       do
       (setf (elt coeff i) p
	     p (* p (elt dimensions i))))
    coeff))

(defmethod make-affi ((dimensions sequence) &optional (offset 0))
  "Setup a row-major affine mapping with the given dimensions."
  (make-affi-int
   dimensions (row-major-coeff-from-dimensions dimensions) offset))

(defun make-affi-cm (dimensions)
  "Setup a _column major_ affine mapping using dimensions."
  ;; Note: the current implementation does this by reversing a row-major affi
  (let ((affi (make-affi (reverse dimensions))))
    (with-slots (coeff domain) affi
      (setf coeff (reverse coeff))
      (setf domain (reverse domain)))
    affi))

(defun calculate-index (affi subscripts)
  "Calculate the index of a given vector of subscripts."
  (with-slots (const coeff domain) affi
    (assert (= (length subscripts) (length domain)))
    (+ const
       (apply
	'+
	(map 'list
	     (lambda (c d s)
	       (assert (and (<= 0 s) (< s d)))
	       (* c s))
	     coeff domain subscripts)))))

(defun delinearize-index (affi linear-index)
  ;; This is the opposite of affi:calculate-index.
  ;; (affi:calculate-index (affi:make-affi '(3 4 5)) (delinearize-index '(20 5 1) 31))
  (loop for pp across (affi:get-coeff affi)
     with index-remaining = linear-index 
     collect (multiple-value-bind (int rem) (floor index-remaining pp)
	       (setf index-remaining rem)
	       int)))

(defun map-affi (affi-in affi-out)
  "A function mapping linearized index between affis."
  (lambda (linear-index)
    (calculate-index affi-out (delinearize-index affi-in linear-index))))

(defun make-walker (affi &optional initial-subscripts)
  "Create a walker for an affine index that starts at the given
subscripts (zeroes by default).  Return two functions: one that
returns the current index and increments the walker, the other just
returns the index without any side effects."
  (with-slots (const coeff domain) affi
    (let* ((rank (rank affi))
	   (subscripts (if initial-subscripts
			   (progn
			     (assert (= (length initial-subscripts) rank))
			     (copy-into-fixnum-vector initial-subscripts))
			   (make-array rank :element-type 'fixnum 
				       :initial-element 0)))
	  (index (calculate-index affi subscripts)))
      (labels ((increment-subscript (i)
		 (unless index
		   (return-from increment-subscript))
		 (let ((c (aref coeff i))
		       (d (aref domain i)))
		   (if (= (incf (aref subscripts i)) d)
		       ;; reached maximum
		       (if (plusp i)
			   (progn 
			     (decf index (* c (1- d)))
			     (setf (aref subscripts i) 0)
			     (increment-subscript (1- i)))
			   (setf index nil))
		       ;; did not reach maximum
		       (incf index c))))
	       (next ()
		 "Return the current value of index, and step forward."
		 (let ((index index))
		   (increment-subscript (1- rank))
		   index))
	       (this ()
		 "Return the current value of index."
		 index))
	(values #'next #'this)))))

#|
;;; Driver for the iter system
(defmacro-driver (for var in-affi affi)
  "Driver for iterate to traverse affine indexes."
  (let ((walker (gensym "walker"))
	(kwd (if generate 'generate 'for)))
    `(progn
       (with ,walker := (make-walker ,affi))
       (,kwd ,var next (let ((i (funcall ,walker))) (if i i (terminate)))))))
|#

(defgeneric test-walker (object)
  (:documentation "Output the indices generated by a walker.  For
testing purposes."))

(defmethod test-walker ((walker function))
  "Output the indices generated by a walker.  For testing purposes."
  (tagbody
     top
     (let ((i (funcall walker)))
       (when i
	 (format t "~a " i)
	 (go top))))
  (format t "~%"))

(defmethod test-walker ((affi affi))
  "Make and then test a walker."
  (test-walker (make-walker affi)))

(defun parse-range (range d)
  "Parse a range specification, returning two values: the starting
point, and the length of the range, whose sign determines the
direction.  [0,d) is the domain.

A range specification is either and atom or a list of two integers.
Negative integers are interpreted as counted backwards from the right
edge of the domain, ie i < 0 denotes element d+i.

If range is an atom, then it can be :all or :rev, denoting the entire
range in regular and reversed order, respectively, or refer to a
single element.

A two-element list denotes an interval, inclusive.  If the first one
is larger then the second, reverse ordering is used."
  (flet ((convert-and-check (i)
	     (cond
	       ((and (<= 0 i) (< i d)) i)
	       ((and (minusp i) (<= 0 (+ d i))) (+ d i))
	       (t (error "subscript ~a is not in [0,~a)" i d)))))
    (cond
      ((and (symbolp range) (eq range :all)) (values 0 d))
      ((and (symbolp range) (eq range :rev)) (values (1- d) (- d)))
      ((integerp range) (values (convert-and-check range) 1))
      ((and (listp range) (= (length range) 2) (every #'integerp range))
       (let ((left (convert-and-check (first range)))
	     (right (convert-and-check (second range))))
	 (values left (- right left (if (<= left right) -1 1)))))
      (t (error "can't interpret range ~a" range)))))

(defun check-conformability (affi1 affi2 &optional (conformability :dropped))
  "Check that two affine indexes are conformable.  There are three
types of conformability: :strict requires that the two domains are
exactly the same, :dropped checks if they are the same when we drop
dimensions of size 1, and :size just checks the size of the two
ranges."
  (flet ((equal-domain (affi1 affi2)
	   (equalp (domain affi1) (domain affi2))))
    (ecase conformability
      (:strict (equal-domain affi1 affi2))
      (:dropped (equal-domain (drop affi1 t) (drop affi2 t)))
      (:size (= (grid::total-size-from-dimension (domain affi1))
		(grid::total-size-from-dimension (domain affi2)))))))
