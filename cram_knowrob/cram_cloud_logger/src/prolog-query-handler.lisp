(in-package :ccl)

(defparameter *prolog-queries* (cpl:make-fluent :value '()))
(defparameter *prolog-query-save-path* nil)
(defparameter *prolog-query-filenames* (cpl:make-fluent :value '()))


(defmethod prolog::prove-one :around (query binds &optional rethrow-cut)
  (if *is-logging-enabled*
      (let ((query-id (create-prolog-log-query (car query)))(result (call-next-method)))
        ;;(when query-id
        ;;  (let ((solution-variable-list (print-prolog-predicate query binds)))
        ;;    (dolist (item solution-variable-list)
        ;;      (let ((variable-bind (assoc item (CRAM-UTILITIES:LAZY-CAR result))))
        ;;        (when variable-bind
        ;;          variable-bind)))))
        (if query-id
            (let ((end-query
                    (create-query
                     "cram_finish_action"
                     (list (car query-id) (get-timestamp-for-logging)))))
              (let ((query-list (car (cdr query-id))))
                (push end-query (cdr (last query-list)))
                (setf (cpl:value *prolog-queries*)
                      (append query-list
                              (cpl:value *prolog-queries*))))))
        result)
      (call-next-method)))

(defun send-batch-query ()
  (print "Sending batch query ...")
  (if (cpl:value *prolog-queries*)
      (send-prolog-query-1 (create-batch-query)))
  (print "Batch query is done"))

(defun create-prolog-log-query (predicate-name)
  (if (is-predicate-in-white-list predicate-name)
      (let ((query-id (concatenate 'string "PrologQuery_" (format nil "~x" (random (expt 16 8)))))
            (queries '()))
        (setf queries
              (cons (create-rdf-assert-query
                     (convert-to-prolog-str (car ccl::*action-parents*))
                     "knowrob:reasoningTask"
                     query-id)
                    queries))
        (setf queries
              (cons (create-rdf-assert-query
                     query-id
                     "knowrob:predicate"
                     (convert-to-prolog-str (write-to-string predicate-name)))
                    queries))
        (setf queries
              (cons (create-query
                     "cram_start_action"
                     (list  (concatenate 'string "knowrob:" (convert-to-prolog-str "PrologQuery"))
                           "\\'TableSetting\\'"
                           (get-timestamp-for-logging)
                           "PV"
                           query-id))
                    queries))
        (list query-id queries))
      nil))

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

(defun create-batch-query()
  (let ((batch-query (car (cpl:value *prolog-queries*))))
    (dolist (item (cdr (cpl:value *prolog-queries*)))
      (setq batch-query (concatenate 'string batch-query "," item)))
    (setf (cpl:value *prolog-queries*) '())
    ;;(print batch-query)
    batch-query))

(defun print-prolog-predicate (query binds)
  (let ((predicate-name (car query))
        (predicate-parameter-list (cdr query))
        (solution-variable-list '()))
    (dolist (item predicate-parameter-list)
      (let ((variable-bind (assoc item binds)))
        (if (not variable-bind)
            (setq solution-variable-list (cons item solution-variable-list)))))
    solution-variable-list))

