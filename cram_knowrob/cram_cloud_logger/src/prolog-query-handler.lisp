(in-package :ccl)

(defparameter *prolog-queries* (cpl:make-fluent :value '()))


(defmethod prolog::prove-one :around (query binds &optional rethrow-cut)
  (if *is-logging-enabled*
      (let ((query-id (create-prolog-log-query (car query)))(result (call-next-method)))
        (if query-id 
            (setf (cpl:value *prolog-queries*)
                  (cons (concatenate 'string query-id (get-more-specific-timestamp-for-logging)  '(#\newline))
                        (cpl:value *prolog-queries*))))
        result)
      (call-next-method)))

(defun send-batch-query ()
  (print "Sending batch query ...")
  ;;Remove this if clause when the predicate white-list is done
  (if nil
      (with-open-file (str "PROLOG-CRAM-LOG.csv"
                           :direction :output
                           :if-exists :append
                           :if-does-not-exist :create)
        (format str (create-batch-query))))
  (print "Batch query is done"))

(defun create-prolog-log-query (predicate-name)
  (if (and (string/= (string-downcase (write-to-string predicate-name)) "bound")
           (string/= (string-downcase (write-to-string predicate-name)) "and")
           (string/= (string-downcase (write-to-string predicate-name)) "or")
           (string/= (string-downcase (write-to-string predicate-name)) "cram-prolog:bound")
           (string-downcase (write-to-string predicate-name))
           ) 
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
        (concatenate 'string (write-to-string predicate-name) ";" (get-more-specific-timestamp-for-logging) ";"))
      nil)
  )


;;Use this version of create-batch-query when you will decide to log the predicates in OWL
;;(defun create-batch-query()
;;  (let ((batch-query ""))
;;    (dolist (item (cdr (cpl:value *prolog-queries*)))
;;      (setq batch-query (concatenate 'string batch-query item)))
;;    (setf (cpl:value *prolog-queries*) '())
;;    batch-query))

(defun create-batch-query()
  (let ((batch-query "") (i 0))
    (loop while (and (cpl:value *prolog-queries*))
          do (setq batch-query
                   (concatenate 'string batch-query
                                (pop (cpl:value *prolog-queries*)))))
    batch-query))

