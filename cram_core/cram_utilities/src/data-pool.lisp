
(in-package :cut)

;;; Definitions for a simple data pool. The data pool is basically a
;;; mapping from id numbers to value. This is necessary for passing
;;; values from lisp to a foreign language interface and back into
;;; lisp again.

(define-condition pool-value-unbound (simple-error)
  ())

(defstruct (data-pool (:conc-name nil))
  (current-handle 0)
  (data (make-hash-table :test 'eql)))

(defgeneric new-pool-value (pool &optional value)
  (:documentation "Registers a new value in the data pool, optionally initialized with
                   `value'. Returns the id of the new data entry."))

(defgeneric pool-value (pool id)
  (:documentation "Gets the value that belongs to `id'."))

(defgeneric (setf pool-value) (new-value pool id)
  (:documentation "Sets a new value for `id'."))

(defgeneric delete-pool-value (pool id)
  (:documentation "Removes the value corresponding to `id'"))

(defmethod new-pool-value ((pool data-pool) &optional value)
  (let ((id (current-handle pool)))
    (setf (gethash id (data pool)) value)
    (incf (current-handle pool))
    id))

(defmethod pool-value ((pool data-pool) id)
  
  (multiple-value-bind (value found?) (gethash id (data pool))
    (unless found?
      (error 'pool-value-unbound
             :format-control "Data pool does not contain a value for id `~a'."
             :format-arguments id))
    value))

(defmethod (setf pool-value) (new-value (pool data-pool) id)
  (multiple-value-bind (value found?) (gethash id (data pool))
    (declare (ignore value))
    (unless found?
      (error 'pool-value-unbound
             :format-control "Data pool does not contain a value for id `~a'."
             :format-arguments id))
    (setf (gethash id (data pool)) new-value)))

(defmethod delete-pool-value ((pool data-pool) id)
  (remhash id (data pool)))
