(in-package :ccl)

(defparameter *prolog-queries* (cpl:make-fluent :value '()))
(defparameter *prolog-query-save-path* nil)
(defparameter *prolog-query-filenames* (cpl:make-fluent :value '()))


(defmethod prolog::prove-one :around (query binds &optional rethrow-cut)
  (if *is-logging-enabled*
      (let ((query-id (create-prolog-log-query query))(result (call-next-method)))
        (when query-id
          (let ((solution-variable-list (print-prolog-predicate query binds)))
            (dolist (item solution-variable-list)
              (let ((variable-bind (assoc item (CRAM-UTILITIES:LAZY-CAR result))))
                (when variable-bind
                  variable-bind)))))
        (if query-id 
            (setf (cpl:value *prolog-queries*)
                  (cons (concatenate 'string query-id (get-more-specific-timestamp-for-logging)  '(#\newline))
                        (cpl:value *prolog-queries*))))
        result)
      (call-next-method)))

(defun send-batch-query ()
  (print "Sending batch query ...")
  (if *prolog-query-save-path*
      (let ((query-filename (format nil "~x.csv" (random (expt 16 8)))))
        (with-open-file (str (concatenate 'string *prolog-query-save-path* query-filename)
                             :direction :output
                             :if-exists :append
                             :if-does-not-exist :create)
          (format str (create-batch-query)))
        (setf (cpl:value *prolog-query-filenames*)
                  (cons query-filename
                        (cpl:value *prolog-query-filenames*)))))
  (print "Batch query is done"))

(defun create-prolog-log-query (query)
  (if (is-predicate-in-white-list (car query))
      (let ((query-id (concatenate 'string "PrologQuery_" (format nil "~x" (random (expt 16 8))))))
        ;;Use this block again if you will decided to log the predicates in OWL
        ;;(setf (cpl:value *prolog-queries*)
        ;;      (cons (create-rdf-assert-query query-id "rdf:type" " owl:\\'NamedIndividual\\'")
        ;;            (cpl:value *prolog-queries*)))
        ;;(setf (cpl:value *prolog-queries*)
        ;;      (cons (create-rdf-assert-query
        ;;             query-id
        ;;             "knowrob:predicate"
        ;;             (create-string-owl-literal (write-to-string predicate-name)))
        ;;            (cpl:value *prolog-queries*)))
        ;;(setf (cpl:value *prolog-queries*)
        ;;      (cons (create-query
        ;;             "cram_start_action"
        ;;             (list (convert-to-prolog-str "PrologQuery")
        ;;                   "\\'DummyContext\\'"
        ;;                   (get-timestamp-for-logging)
        ;;                   "PV"
        ;;                   query-id))
        ;;            (cpl:value *prolog-queries*)))
        (concatenate 'string (write-to-string (car query)) ";" (get-more-specific-timestamp-for-logging) ";"))
      nil)
  )

(defun is-predicate-in-white-list (predicate-name)
  (or (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-object-interfaces:object-type-grasp")
      (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-object-interfaces:object-rotationally-symmetric")
      (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-semantic-map-costmap::semantic-map-desig-objects")
      (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-semantic-map-costmap:semantic-map-objects")
      (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-designators:action-grounding")
      (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-designators:motion-grounding")
      (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-designators:location-grounding")))

;;Use this version of create-batch-query when you will decide to log the predicates in OWL
;;(defun create-batch-query()
;;  (let ((batch-query ""))
;;    (dolist (item (cdr (cpl:value *prolog-queries*)))
;;      (setq batch-query (concatenate 'string batch-query item)))
;;    (setf (cpl:value *prolog-queries*) '())
;;    batch-query))

(defun print-prolog-predicate (query binds)
  (let ((predicate-name (car query))
        (predicate-parameter-list (cdr query))
        (solution-variable-list '()))
    (dolist (item predicate-parameter-list)
      (let ((variable-bind (assoc item binds)))
        (if (not variable-bind)
            (setq solution-variable-list (cons item solution-variable-list)))))
    solution-variable-list))

(defun create-batch-query()
  (let ((batch-query ""))
    (loop while (cpl:value *prolog-queries*)
          do (setq batch-query
                   (concatenate 'string batch-query
                                (pop (cpl:value *prolog-queries*)))))
    batch-query))

(defun merge-all-csv-files ()
  (if *prolog-query-save-path*
      (progn (with-open-file (str (concatenate 'string *prolog-query-save-path* "main.csv")
                                  :direction :output
                                  :if-exists :append
                                  :if-does-not-exist :create)
               (format str (concatenate 'string "PREDICATE;STARTTIME;ENDTIME" '(#\newline))))
             (loop while (cpl:value *prolog-query-filenames*)
                   do (with-open-file (str (concatenate 'string *prolog-query-save-path* "main.csv")
                                  :direction :output
                                  :if-exists :append
                                  :if-does-not-exist :create)
                        (format str (file-string
                                     (concatenate 'string
                                                  *prolog-query-save-path*
                                                  (pop (cpl:value *prolog-query-filenames*))))))))))


(defun file-string (path)
  (with-open-file (stream path)
    (let ((data (make-string (file-length stream))))
      (read-sequence data stream)
      data)))

