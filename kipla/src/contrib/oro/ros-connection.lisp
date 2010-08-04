
(in-package :kipla-reasoning)

(defun oro-call (query &rest params)
  (labels ((orolize-list (seq)
             "Converts a sequence to a string in oro format.

            `seq' is normally a sequence of sequences"
             (format nil "[~{\"~{~a~^ ~}\"~^, ~}]" (symbols->strings seq)))
           (orolize-expression (exp)
             (typecase exp
               (list (orolize-list exp))
               (symbol (format nil "\"~a\"" (substitute #\_ #\- (string exp))))
               (t (format nil "\"~a\"" exp))))
           (symbols->strings (seq)
             (map-tree (lambda (elem)
                         (typecase elem
                           (symbol
                              (substitute #\_ #\- (string elem)))
                           (t elem)))
                       seq)))
    (kipla:log-msg :info "Doing a oro call: ~a ~a" query params)
    (yason:parse
     (oro_ros-srv:res-val
      (roslisp:call-service (format nil "/oro/~a" query) 'oro_ros-srv:oroserverquery
                            :params (make-array (length params)
                                                :initial-contents (mapcar #'orolize-expression params)))))))

(defun oro-assert (fact &rest other-facts)
  "Adds facts to the oro ontology."
  (oro-call "add" (cons fact other-facts)))

(defun oro-retract (fact &rest other-facts)
  "Removes facts from the oro ontology"
  (oro-call "remove" (cons fact other-facts)))

(defun oro-query (query &rest additional-queries)
  "Performs a simple query in the oro ontology.
   The queries must contain only one variable."
  (let ((vars (vars-in (cons query additional-queries))))
    (assert (eql (length vars) 1) ()
            "The query must contain exactly one variable")
    (mapcar (alexandria:compose #'list (alexandria:curry #'cons (car vars)))
            (oro-call "find" (car vars) (cons query additional-queries)))))
