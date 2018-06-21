(in-package :ccl)

(defparameter *prolog-queries* (cpl:make-fluent :value '()))
(defparameter *prolog-query-save-path* nil)
(defparameter *prolog-query-filenames* (cpl:make-fluent :value '()))


(defmethod prolog::prove-one :around (query binds &optional rethrow-cut)
  (if *is-logging-enabled*
      (let ((query-id (create-prolog-log-query (car query)))(result (call-next-method)))
        ;; Come back here to implement parameter logging
        ;;(when query-id
        ;;  (parameter-logging query binds result))
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


(defun parameter-logging (query binds result)
  (let ((variable-list-list (print-prolog-predicate query binds))
        (predicate-name (car query)))
    (let ((bounded-variable-list (car variable-list-list))
          (solution-variable-list (cadr variable-list-list)))
      (if (is-predicate-a-designator-grounding-predicate predicate-name)
          (progn
            (print "########")
            (print query)
            (let ((temp-result (cdr (assoc (caddr query) (CRAM-UTILITIES:LAZY-CAR result)))))
              (if
               (listp temp-result)
               (dolist (item temp-result)
                 (if (is-cram-prolog-variable (write-to-string item))
                     (progn
                       (print (assoc item (CRAM-UTILITIES:LAZY-CAR result))))
                     (print item)))))
            ;;(print predicate-name)
            ;;(print bounded-variable-list)
            ;;(print solution-variable-list)
            ;;(if result
            ;;    (progn (print "TRUE")
            ;;    (print result)))
            (print "########")
            ;;(dolist (item solution-variable-list)
            ;;  (let ((variable-bind (assoc item (CRAM-UTILITIES:LAZY-CAR result))))
            ;;    (when variable-bind
            ;;      (print variable-bind))))
            (print "--------"))))))

(defun send-batch-query ()
  (print "Sending batch query ...")
  (if (cpl:value *prolog-queries*)
      (send-prolog-query-1 (create-batch-query)))
  (print "Batch query is done"))

(defun create-obj-true-false-log-query (predicate-name)
  (let ((query-id (concatenate
                   'string
                   (create-obj-log-query-class-name predicate-name)
                   (format nil "~x" (random (expt 16 8)))))
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
    (list query-id queries)))

(defun create-obj-log-query-class-name (predicate-name)
  (let ((class-name (write-to-string predicate-name)))
    (cond ((string-equal (string-downcase class-name)
                         "cram-object-interfaces:object-type-grasp")
           (setf class-name "ObjectTypeGrasp"))
          ((string-equal (string-downcase class-name)
                         "cram-object-interfaces:object-rotationally-symmetric")
           (setf class-name "ObjectRotatinallySymmetric")))
    (concatenate 'string class-name "_")))

(defun create-prolog-log-query (predicate-name)
  (let ((predicate-name-str
          (concatenate 'string
                       (package-name (symbol-package predicate-name))
                       ":"
                       (symbol-name predicate-name))))
    (if (is-predicate-in-white-list predicate-name-str)
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
                       (convert-to-prolog-str predicate-name-str))
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
        nil)))

(defun is-predicate-in-white-list (predicate-name)
  (or (string-equal (string-downcase predicate-name)
                    "cram-object-interfaces:object-type-grasp")
      (string-equal (string-downcase predicate-name)
                    "cram-object-interfaces:object-rotationally-symmetric")
      (string-equal (string-downcase predicate-name)
                    "cram-semantic-map-costmap::semantic-map-desig-objects")
      (string-equal (string-downcase predicate-name)
                    "cram-semantic-map-costmap:semantic-map-objects")
      (string-equal (string-downcase predicate-name)
                    "cram-designators:action-grounding")
      (string-equal (string-downcase predicate-name)
                    "cram-designators:motion-grounding")
      (string-equal (string-downcase predicate-name)
                    "cram-designators:location-grounding")))

(defun is-predicate-an-obj-interface-predicate (predicate-name)
  (or (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-object-interfaces:object-type-grasp")
      (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-object-interfaces:object-rotationally-symmetric")))

(defun is-predicate-a-designator-grounding-predicate (predicate-name)
  (or (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-designators:location-grounding")
      (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-designators:motion-grounding")
      (string-equal (string-downcase (write-to-string predicate-name))
                    "cram-designators:action-grounding")))

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
        (binded-variable-list '())
        (solution-variable-list '()))
    (dolist (item predicate-parameter-list)
      (let ((variable-bind (assoc item binds)))
        (if (not variable-bind)
            (setq solution-variable-list (cons item solution-variable-list))
            (setq binded-variable-list (cons variable-bind binded-variable-list)))))
    (list binded-variable-list solution-variable-list)))

(defun is-cram-prolog-variable (variable-name-str)
  (ccl::string-start-with variable-name-str "#"))

