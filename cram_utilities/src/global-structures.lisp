(in-package :cut)

(let ( (known-structures (make-hash-table)) )

  ;; create a new hashtable for this name with the given test
  (defun create-global-structure (name)
    "name must be Symbol"
    (setf (gethash name known-structures) (make-hash-table)))

  (declaim (inline get-global-structure))
  (defun get-global-structure (name)
    (multiple-value-bind (struct defined) (gethash name known-structures)
      (if defined
        struct
        (error "[GLOBAL-STRUCTURES] Undefined global structure ~s." name))))

  (defun remove-global-structure (name)
    (remhash name known-structures))

  (defun clear-global-structure (name)
    (clrhash (gethash name known-structures)))

  (defun show-known-global-structures ()
    (maphash #'(lambda (k v) (declare (ignore v)) (format t "~s~%" k)) known-structures))

)

; map global structure
(defun map-global-structure (fun name)
  (let ( (resultlist ()) )
    (maphash #'(lambda (k c)
                 (declare (ignore k))
                 (push (funcall fun c) resultlist))
      (get-global-structure name))
    (nreverse resultlist)))

; map global structure keys
(defun map-global-structure-keys (fun name)
  (let ( (resultlist ()) )
    (maphash #'(lambda (k c)
                 (declare (ignore c))
                 (push (funcall fun k) resultlist))
      (get-global-structure name))
    (nreverse resultlist)))

; map global structure keys
(defun map-global-structure-with-keys (fun name)
  (let ( (resultlist ()) )
    (maphash #'(lambda (k c)
                 (push (funcall fun k c) resultlist))
      (get-global-structure name))
    (nreverse resultlist)))

;;; Return matching global structures
(defun filter-global-structure (filter name)
  "Calls filter for every value in the global structure 'name'.
   Filter is a function taking exactly two parameters, the name of the
   variable and its value.  It non-nil return value indicates that the
   variable should be kept.  filter-global-structure returns an alist
   containing the matching variables and their keys."
  (loop for k being the hash-keys in (get-global-structure name) using (hash-value v)
        when (funcall filter k v) collect (cons k v) into result
        finally (return result)))

; Hilfsfunktion, wird nicht exportiert
(defun show-global-structure (name)
  (maphash #'(lambda (k c) (format t "~s: ~a~%" k c)) (get-global-structure name)))

; Verwaltung der einzelnen Strukturen
(defun addgv (structure var-name &optional value)
  (setf (gethash var-name (get-global-structure structure)) value)
  value)

(defun isgv (structure var-name)
  (multiple-value-bind (val defined) (gethash var-name (get-global-structure structure))
    (declare (ignore val))
    defined))

(defun getgv (structure var-name)
  (multiple-value-bind (value defined) (gethash var-name (get-global-structure structure))
    (if defined
      value
      (error "[GLOBAL-STRUCTURES] Undefined variable ~s in ~s." var-name structure))))

(defun (setf getgv) (val structure var-name)
  (setgv structure var-name val))

(defun setgv (structure var-name value)
  (multiple-value-bind (old-value defined) (gethash var-name (get-global-structure structure))
    (declare (ignore old-value))
    (if defined
      (setf (gethash var-name (get-global-structure structure)) value)
      (error "[GLOBAL-STRUCTURES] Undefined variable ~s in structure ~s." var-name structure))))

(defun putgv (structure var-name value)
  (setf (gethash var-name (get-global-structure structure)) value))

(defun remgv (structure var-name)
  (remhash var-name (get-global-structure structure)))
