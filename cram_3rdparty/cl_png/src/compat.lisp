(in-package #:png)

#+sbcl
(macrolet ((make-array-storage-vector ()
             (let ((%array-data-vector (or (find-symbol "%ARRAY-DATA-VECTOR" :sb-kernel)
                                           (find-symbol "%ARRAY-DATA" :sb-kernel)))) ;; renamed in sbcl 2.1.6
               `(progn
                  (declaim (ftype (function (array) (values (simple-array * (*)) &optional)) array-storage-vector))
                  (defun array-storage-vector (array)
                    "Returns the underlying storage vector of ARRAY, which must be a non-displaced array.

In SBCL, if ARRAY is a of type \(SIMPLE-ARRAY * \(*)), it is its own storage
vector. Multidimensional arrays, arrays with fill pointers, and adjustable
arrays have an underlying storage vector with the same ARRAY-ELEMENT-TYPE as
ARRAY, which this function returns.

Important note: the underlying vector is an implementation detail. Even though
this function exposes it, changes in the implementation may cause this
function to be removed without further warning."
                    (sb-ext:truly-the (simple-array * (*))
                                      (if (sb-kernel:array-header-p array)
                                          (if (sb-kernel:%array-displaced-p array)
                                              (error "~S cannot be used with displaced arrays. Use ~S instead."
                                                     'array-storage-vector 'array-displacement)
                                              (,%array-data-vector array))
                                          array)))))))
  (make-array-storage-vector))

#+allegro
(defmacro with-pointer-to-array-data ((ptr-var array) &body body)
  "Bind PTR-VAR to a foreign pointer to the data in VECTOR. Not safe
except with array allocated by MAKE-SHAREABLE-ARRAY and
possibly arrays of type simple-array (unsigned-byte 8) (*)."
;;; An array allocated in static-reclamable is a non-simple array in
;;; the normal Lisp allocation area, pointing to a simple array in the
;;; static-reclaimable allocation area. Therefore we have to get out
;;; the simple-array to find the pointer to the actual contents.
  (let ((simple-arr (gensym "SIMPLE-ARR")))
    `(excl:with-underlying-simple-vector (,array ,simple-arr)
       (let ((,ptr-var (ff:fslot-address-typed :unsigned-char 
					       :lisp ,simple-arr)))
	 ,@body))))

#+clisp
(defmacro with-pointer-to-array-data ((ptr-var array) &body body)
  "Bind PTR-VAR to a foreign pointer to the data in ARRAY."
  (let ((array-var (gensym))
	(type (gensym))
	(nbytes (gensym))
	(bytes-per-word (gensym)))
    `(let* ((,array-var ,vector)
	    ,type ,bytes-per-word)
       (etypecase ,array-var
	 ((simple-array (unsigned-byte 8)) (setq ,type :unsigned-char
						     ,bytes-per-word 1))
	 ((simple-array (unsigned-byte 16)) (setq ,type :unsigned-short
						      ,bytes-per-word 2)))
       (with-foreign-pointer (,ptr-var (* (array-total-size ,array-var)
					  ,bytes-per-word)
				       ,nbytes)
         ;; copy-in
         (loop
	    for word from 0 
	    and byte below ,nbytes by ,bytes-per-word 
	    do (cffi-sys:%mem-set (row-major-aref ,array-var word)
				  ,ptr-var ,type byte))
         (unwind-protect (progn ,@body)
           ;; copy-out
           (loop 
	      for word from 0
	      and byte below ,nbytes by ,bytes-per-word
	      do (setf (row-major-aref ,array-var word)
		       (cffi-sys:%mem-ref ,ptr-var ,type byte))))))))

#+lispworks
(defmacro with-pointer-to-array-data ((ptr-var array) &body body)
  `(progn
     (assert (system:staticp ,array) `(,array)
	     "Array must be allocated in the static area.")
     (with-pointer-to-vector-data (,ptr-var ,array) ,@body)))

#+sbcl
(defmacro with-pointer-to-array-data ((ptr-var array) &body body)
  `(cffi:with-pointer-to-vector-data (,ptr-var (array-storage-vector ,array))
     ,@body))

#+ccl
(defmacro with-pointer-to-array-data ((ptr-var array) &body body)
  (let ((v (gensym)))
    `(let ((,v (ccl::array-data-and-offset ,array)))
       (unless (typep ,v 'ccl::ivector) 
	 (ccl::report-bad-arg ,v 'ccl::ivector))
       (ccl::without-gcing
         (ccl:with-macptrs ((,ptr-var))
           (ccl::%vect-data-to-macptr ,v ,ptr-var)
           ,@body)))))


