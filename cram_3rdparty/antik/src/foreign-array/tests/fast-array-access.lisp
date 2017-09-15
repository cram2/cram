;; Test speed of foreign array access
;; Sebastian Sturm
;; Time-stamp: <2011-05-23 22:50:57EDT fast-array-access.lisp>

;;; (require :foreign-array-tests) 

(in-package :grid)

(eval-when (:compile-toplevel :load-toplevel :execute)
  (require :sb-sprof)
  (require :iterate))
(declaim (optimize (speed 3) (safety 0) (debug 0)))

(defun aref-access (dim)
  "Given an integer dim, this constructs a function that, when supplied with a 
   N-dimensional vector Z and some output vector (-> pointer?), yields the
   corresponding forces"
  (let ((temp-values (make-array 2 :element-type 'double-float :initial-element 0.0d0)))
    (lambda (zvector output)
      (declare (fixnum dim)
	       (optimize (speed 3) (safety 0) (debug 0))
	       (type vector-double-float zvector))
      (do ((i 0 (1+ i))) ((= i dim)) (declare (fixnum i))
	(setf (cl:aref temp-values 0) 0.0d0)
	(do ((m 0 (1+ m))) ((> m i)) (declare (fixnum m))
	  (do ((n i (1+ n))) ((= n dim)) (declare (fixnum n))
	    (setf (cl:aref temp-values 1) 0.0d0)
	    (do ((k m (1+ k))) ((> k n)) (declare (fixnum k))
	      (incf (cl:aref temp-values 1) (grid:aref (the vector-double-float zvector) k)))
	    (incf (cl:aref temp-values 0) (expt (cl:aref temp-values 1) -2))))
	(setf (grid:aref output i) 
	      (- (grid:aref (the vector-double-float zvector) i)
		 (cl:aref temp-values 0)))))))

(defun aref*-access (dim)
  "Given an integer dim, this constructs a function that, when supplied with a 
   N-dimensional vector Z and some output vector (-> pointer?), yields the
   corresponding forces"
  (let ((temp-values (make-array 2 :element-type 'double-float :initial-element 0.0d0)))
    (lambda (zvector output)
      (declare (fixnum dim)
	       ;;(type vector-double-float zvector)
	       (optimize (speed 3) (safety 0) (debug 0)))
      (do ((i 0 (1+ i))) ((= i dim)) (declare (fixnum i))
	(setf (cl:aref temp-values 0) 0.0d0)
	(do ((m 0 (1+ m))) ((> m i)) (declare (fixnum m))
	  (do ((n i (1+ n))) ((= n dim)) (declare (fixnum n))
	    (setf (cl:aref temp-values 1) 0.0d0)
	    (do ((k m (1+ k))) ((> k n)) (declare (fixnum k))
	      (incf (cl:aref temp-values 1)
		    (grid:aref* (the vector-double-float zvector) k)))
	    (incf (cl:aref temp-values 0) (expt (cl:aref temp-values 1) -2))))
	(setf (grid:aref* output i) 
	      (- (grid:aref* (the vector-double-float zvector) i)
		 (cl:aref temp-values 0)))))))

(defun cffi-access (dim)
  "Given an integer dim, this constructs a function that, when supplied with a 
   N-dimensional vector Z and some output vector (-> pointer?), yields the
   corresponding forces"
  (declare (fixnum dim))
  (let ((temp-values (make-array 2 :element-type 'double-float :initial-element 0.0d0)))
    (lambda (zvector output)
      (let ((zvector-fptr (grid::foreign-pointer zvector))
	    (output-fptr (grid::foreign-pointer output))
	    )
	(macrolet ((quick-ref (the-vector n)
		     `(cffi:mem-aref
		       ,(case the-vector
			  (zvector 'zvector-fptr)
			  (output 'output-fptr))
		        :double
		       ,n))) 
	  (do ((i 0 (1+ i))) ((= i dim)) (declare (fixnum i))
	    (setf (cl:aref temp-values 0) 0.0d0)
	    (do ((m 0 (1+ m))) ((> m i)) (declare (fixnum m))
	      (do ((n i (1+ n))) ((= n dim)) (declare (fixnum n))
		(setf (cl:aref temp-values 1) 0.0d0)
		(do ((k m (1+ k))) ((> k n)) (declare (fixnum k))
		  (incf (cl:aref temp-values 1) (quick-ref zvector k)))
		(incf (cl:aref temp-values 0) (expt (cl:aref temp-values 1) -2))))
	    (setf (quick-ref output i) 
		  (- (quick-ref zvector i)
		     (cl:aref temp-values 0)))))))))

(defun cffi-access-dynptr (dim)
  "Given an integer dim, this constructs a function that, when supplied with a 
   N-dimensional vector Z and some output vector (-> pointer?), yields the
   corresponding forces"
  (declare (fixnum dim))
  (let ((temp-values (make-array 2 :element-type 'double-float :initial-element 0.0d0)))
    (lambda (zvector output)
      (macrolet ((quick-ref (the-vector n)
		   `(cffi:mem-aref
		     ,(case the-vector
			    (zvector '(grid::foreign-pointer zvector))
			    (output '(grid::foreign-pointer output)))
		     :double
		     ,n))) 
	(do ((i 0 (1+ i))) ((= i dim)) (declare (fixnum i))
	  (setf (cl:aref temp-values 0) 0.0d0)
	  (do ((m 0 (1+ m))) ((> m i)) (declare (fixnum m))
	    (do ((n i (1+ n))) ((= n dim)) (declare (fixnum n))
	      (setf (cl:aref temp-values 1) 0.0d0)
	      (do ((k m (1+ k))) ((> k n)) (declare (fixnum k))
		(incf (cl:aref temp-values 1) (quick-ref zvector k)))
	      (incf (cl:aref temp-values 0) (expt (cl:aref temp-values 1) -2))))
	  (setf (quick-ref output i) 
		(- (quick-ref zvector i)
		   (cl:aref temp-values 0))))))))

(let* ((dim 30)
       (vec (grid:make-foreign-array 'double-float :initial-element 1 :dimensions dim))
       (output-1 (grid:make-foreign-array 'double-float :dimensions dim))
       (slow-fn (aref-access dim))
       (intermediate-fn (aref*-access dim))
       (quick-fn (cffi-access dim)))
  (declare (type vector-double-float output-1 vec))
  (print "aref")
  (time (funcall slow-fn vec output-1))

  (print "aref* directly")
  (time (funcall intermediate-fn vec output-1))

  (print "hardwired cffi:mem-aref")
  (time (funcall quick-fn vec output-1))
  (print "hardwired cffi:mem-aref dynptr")
  (time (funcall (cffi-access-dynptr dim) vec output-1))
  )

(time (let ((dim 50))
	(call-aref (grid:make-foreign-array 'double-float :initial-element 1 :dimensions dim)
		   (grid:make-foreign-array 'double-float :dimensions dim)
		   dim)))

(time (let ((dim 50))
	(call-aref* (grid:make-foreign-array 'double-float :initial-element 1 :dimensions dim)
		   (grid:make-foreign-array 'double-float :dimensions dim)
		   dim)))

