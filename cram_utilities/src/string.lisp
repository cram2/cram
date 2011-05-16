(in-package :cut)

; ->string
(defun ->string (x)
  "Converts x to a string. If x is a list then the members are converted to strings"
  (cond ( (listp x)
          (mapcar #'->string x) )
        ( (or (stringp x) (characterp x) (symbolp x))
          (string x) )
        ( (numberp x)
          (let ( (*print-radix* nil) )
            (princ-to-string x)) )
        ( T
          (format nil "~s" x) )))

; string->symbol
(declaim (inline string->symbol))
(defun string->symbol (x)
  "Converts x from string to symbol."
  (when (stringp x)
    (multiple-value-bind (result-strings no-of-splits) (split-one x #\:)
      (cond ( (zerop no-of-splits)
              (alexandria:format-symbol *package* (string-upcase x)) )
            ( (equal (first result-strings) "")
              (alexandria:format-symbol :keyword (string-upcase (second result-strings))) )
            ( T
              (alexandria:format-symbol (string-upcase (first result-strings))
                                            (string-upcase (second result-strings))) )))))



; string concat
(declaim (inline string-concat))
(defun string-concat (&rest vl)
  "concatenates strings"
  (apply #'concatenate 'string vl))

; string-prefix-p
(defun string-prefix-p (pre str)
  (and
    (<= (length pre) (length str))
    (every (alexandria:compose #'not #'null) (map 'list #'char= pre str))))

; split-one
(declaim (inline split-one))
(defun split-one (seq delimiter &rest usual-keys)
  (let ( (pos (apply #'position delimiter seq usual-keys)) )
    (if pos
      (values (list (subseq seq 0 pos)
                    (subseq seq (1+ pos)))
              1)
      (values (list seq) 0))))

; split
(defun split (seq delimiter &key (test #'eq) (key nil))
  (multiple-value-bind (split-seq nr) (split-one seq delimiter :test test :key key)
    (cond ( (zerop nr)
            (values split-seq nr) )
          ( T
            (multiple-value-bind (splitt-seq nrr) (split (second split-seq) delimiter :test test :key key)
              (values (cons (first split-seq) splitt-seq)
                      (+ nr nrr))) ))))
