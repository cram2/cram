(in-package :cut)

; random-list
(defun make-random-list (count &key (lower 0) (upper 1000))
  "returns a list of random numbers in range [lower, limit[ of length count"
  (labels ( (random-list-tail (count lower upper result)
              (cond ( (zerop count)
                      result )
                    ( T
                      (random-list-tail (1- count) lower upper
                                        (cons (random-number upper :lower lower) result)) ))) )
    (random-list-tail count lower upper ())))

; rec-remove
(defun rec-remove (item sequence &key (test #'eql))
  (cond ( (null sequence)
          () )
        ( (funcall test (first sequence) item)
          (rec-remove item (rest sequence) :test test) )
        ( T
          (cons
            (if (atom (first sequence))
              (first sequence)
              (rec-remove item (first sequence) :test test))
            (rec-remove item (rest sequence) :test test)) )))

; find-and-remove
(declaim (inline find-and-remove))
(defun find-and-remove (item seq &rest key-args)
  (let ( (found-item (apply #'find item seq key-args)) )
    (values
      found-item
      (apply #'remove item seq key-args))))

; find-atom-rec-if
(defun find-atom-rec-if (predicate tree)
  (cond ( (null tree)
          nil )
        ( (atom tree)
          (when (funcall predicate tree)
            (list tree)) )
        ( T
          (remove-duplicates (mapcan #'(lambda (tt) (find-atom-rec-if predicate tt)) tree)) )))

; filter-if
(defun filter-if (fn ll &key (key #'identity))
  "filters from the list ll the elements that satisfy the filter predicate and returns them in a list"
  (nreverse
    (reduce #'(lambda (res arg) (if (funcall fn (funcall key arg)) (cons arg res) res))
            ll
            :initial-value ())))

; filter
(defun filter (elem ll &key (key #'identity) (test #'equal))
  (filter-if #'(lambda (x) (funcall test x elem)) ll :key key))

; make number list
(defun make-number-list (start end &optional (delta 1) (acc ()))
  "returns list of numbers between start and end"
  (cond ( (or (and (>= delta 0) (> start end))
              (and (< delta 0) (< start end)))
          (reverse acc) )
        ( T
          (make-number-list (+ start delta) end delta (cons start acc)) )))

