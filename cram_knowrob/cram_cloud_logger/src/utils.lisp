(in-package :ccl)

(defun get-designator-property-value-str (designator property-keyname)
  (string (cadr (assoc property-keyname (desig:properties designator)))))

(defun string-start-with (str prefix)
  (if (< (length str) (length prefix))
      nil
      (string= prefix (subseq str 0 (length prefix)))))

(defun get-timestamp-for-logging ()
  ;;(write-to-string (truncate (cram-utilities:current-timestamp))))
  (get-more-specific-timestamp-for-logging))

(defun get-more-specific-timestamp-for-logging ()
  ;;  (format nil "~d"  (truncate (* 1000000 (cram-utilities:current-timestamp)))))
  (format nil "~f"(cram-utilities:current-timestamp)))

(defun get-id-from-query-result (query-result)
  (let ((x (string (cdaar query-result))))
    (subseq x 2 (- (length x) 2))))

(defun get-value-of-json-prolog-dict (json-prolog-dict key-name)
  (let ((json-prolog-dict-str (string json-prolog-dict)))
    (let ((key-name-search-str (concatenate 'string key-name "\":\"")))
      (let ((key-name-pos (+ (search key-name-search-str (string json-prolog-dict-str)) (length key-name-search-str))))
        (let ((sub-value-str (subseq (string json-prolog-dict-str) key-name-pos)))
          (subseq  sub-value-str 0 (search "\"" sub-value-str)))))))

(defun create-owl-literal (literal-type literal-value)
  (concatenate 'string "literal(type(" literal-type "," literal-value "))"))

(defun convert-to-prolog-str(lisp-str)
  (if (eq 0 (search "'" lisp-str))
      (concatenate 'string "\\'" (subseq lisp-str 1 (- (length lisp-str) 1)) "\\'")
      (concatenate 'string "\\'" lisp-str "\\'")))

(defun create-float-owl-literal (value)
  (create-owl-literal "xsd:float" (format nil "~f" value)))

(defun create-string-owl-literal (value)
  (create-owl-literal "xsd:string" value))

(defun get-last-element-in-list (l)
  (if (or (eq (length l) 1) (eq (length l) 0))
      (car l)
      (get-last-element-in-list (cdr l))))


(defun create-parameters (parameter-list)
  (if (listp parameter-list)
      (let ((result (car parameter-list)))
        (dolist (item (cdr parameter-list))
          (setq result (concatenate 'string result "," item)))
        result)))

(defun create-query (query-name parameter-list)
  (concatenate 'string query-name "(" (create-parameters parameter-list) ")"))

(defun create-rdf-assert-query (a b c)
  (concatenate 'string "rdf_assert(" a "," b "," c ", \\'LoggingGraph\\')"))




