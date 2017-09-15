(in-package :cpl-tests)

;;;; WITH-TASK-HIERARCHY

;;; WITH-TASK-HIERARCHY is a helper macro to conveniently and
;;; concisely establish a whole tree of tasks. In the example below,
;;; it will generate a tree of tasks which looks like:
;;;
;;;
;;;                                       A
;;;                                       |
;;;          +--------------+-------------+------------+
;;;         /              /              |             \
;;;       B_1             B_2            ...            B_N1
;;;     / / \ \         / / \ \        / / \ \        / /  \ \
;;;   C_1......C_N2  C_1.......C_N2   ..........    ............
;;;
;;;
;;;   (with-task-hierarchy ((A  -> Bs)
;;;                         (Bs -> Cs)
;;;                         (Cs ->   ))
;;;         ((:task  A     ..body A...)
;;;          (:tasks Bs N1 ..body Bs..)
;;;          (:tasks Cs N2 ..body Cs..))
;;;    ..body..)
;;;
;;; The body of WITH-TASK-HIEARCHY is executed in the main thread and
;;; is supposed to frob the task tree vigorously, and then check any
;;; appropriate invariants.

(eval-when (:compile-toplevel :load-toplevel :execute)

;;; A TASK-DESCRIPTOR holds the compile-time information needed to
;;; generate the code that will ultimatively lead to a task at
;;; run-time.
(defstruct (task-descriptor (:conc-name td-))
  (name        :unparsed)  ; name as appeared in the source
  (childs      :unparsed)  ; list of child TDs
  (count       :unparsed)  ; count per node
  (total-count 0)          ; total count in whole tree
  (body        :unparsed)  ; task definition
  (singlep     :unparsed)  ; flag indicating (:task ...) vs. (:tasks ...)
  (marked      nil))       ; flag used to detect cycles

;;; Either update an existing entry for NAME in DESCRIPTOR-MAP,
;;; or create one. DESCRIPTOR-MAP is a hash-table mapping from
;;; TD-NAME to TASK-DESCRIPTOR.
(defun ensure-task-descriptor (name descriptor-map &key (childs  :unparsed)
                                                        (count   :unparsed)
                                                        (body    :unparsed)
                                                        (singlep :unparsed))
  (flet ((assert-unparsed (value)
           (assert (eq value :unparsed) () "Duplicate clause for ~S." name)))
    (multiple-value-bind (td found?) (gethash name descriptor-map)
      (cond (found?
             (macrolet ((maybe-setf (place value)
                          `(unless (eq ,value :unparsed)
                             (assert-unparsed ,place)
                             (setf ,place ,value))))
               (maybe-setf (td-childs  td) childs)
               (maybe-setf (td-count   td) count)
               (maybe-setf (td-body    td) body)
               (maybe-setf (td-singlep td) singlep)
               td))
            (t
             (setf (gethash name descriptor-map)
                   (make-task-descriptor :name name
                                         :childs childs
                                         :count count
                                         :body body
                                         :singlep singlep)))))))

(defun find-task-descriptor (name descriptor-map)
  (values (gethash name descriptor-map)))

;;; Sugar on top of (ASSERT (NULL (SET-DIFFERENCE A B))).
;;; It's a macro to get nicer indentation.
(defmacro assert-set-difference-empty
    (A B &body (format-control . format-args))
  `(let ((diff (set-difference ,A ,B)))
     (assert (null diff) ()
             ,(concatenate 'string
                           "~@<" format-control " ~2I~:_~{~S~^, ~}.~@:>")
             ,@format-args
             diff)))

(defun assert-hierarchy-acyclic (td)
  (when (td-marked td)
    (error "Hierarchy contains a cycle."))
  (setf (td-marked td) t)
  (mapc #'assert-hierarchy-acyclic (td-childs td))
  (setf (td-marked td) nil))

;;; Takes the hierarchy specifications and task definitions from
;;; WITH-TASK-HIERARCHY, and parses them into: a list of the names
;;; specified, the TD map, and the root TD of the hierarchy.
(defun parse-task-hierarchy (hierarchy definitions)
  (flet ((parse-hierarchy (hierarchy map)
           ;; Updates MAP; returns list of LHS, and list of RHS.
           (let ((seen-parents '())
                 (seen-childs '()))
             (dolist (clause hierarchy)
               (destructuring-bind (parent -> . childs) clause
                 (assert (string= -> "->") ()
                         "Bogus hierarchy clause: ~S" clause)
                 (setf seen-parents (cons parent seen-parents))
                 (setf seen-childs  (append childs seen-childs))
                 (setq childs (loop for c in childs collecting
                                    (ensure-task-descriptor c map)))
                 (ensure-task-descriptor parent map :childs childs)))
             (values seen-parents seen-childs)))
         (parse-definitions (definitions map)
           ;; Updates MAP; returns list of names we have definitions for.
           (loop for def in definitions collecting
                 (multiple-value-bind (task-name task-count task-body)
                     (destructure-case def
                       ((:task name . body)
                        (values name nil body))
                       ((:tasks name n . body)
                        (values name n body))
                       (t
                        (error "Unknown WITH-TASK-HIERARCHY clause: ~S" def)))
                   (ensure-task-descriptor task-name map
                                           :body task-body
                                           :count (or task-count 1)
                                           :singlep (not task-count))
                   task-name))))
    (let ((td-map (make-hash-table)))
      (multiple-value-bind (task-names child-names)
          (parse-hierarchy hierarchy td-map)
        (let ((task-names* (parse-definitions definitions td-map)))
          ;; Some error checking; is the HIERARCHY specification
          ;; congruent to the DEFINITIONS, or did the user forgot
          ;; something?
          (assert-set-difference-empty task-names task-names*
            "Missing task body definition for:")
          (assert-set-difference-empty task-names* task-names
            "Missing task hierarchy specification for:")
          (assert-set-difference-empty child-names task-names
            "The following task names appear on the right hand side ~
             in the hierarchy specification yet you forgot to provide ~
             corresponding left hand sides for them:")
          (values task-names
                  td-map
                  (loop for root in (set-difference task-names child-names)
                        for root-td = (find-task-descriptor root td-map)
                        do (assert-hierarchy-acyclic root-td)
                        ;; We can't fill the TOTAL-COUNT slots within
                        ;; PARSE-HIERACHY, or PARSE-DEFINITIONS,
                        ;; because the user can specify the hierarchy
                        ;; in any order.
                        collect (fill-total-counts root-td))))))))

;;; Fills all the TOTAL-COUNT slots of the sub tree where TD
;;; is the root of. As TD-COUNT comes from the user, it can be any
;;; expression evaluating to a number; hence we actually construct
;;; expressions here.
(defun fill-total-counts (td &optional (count-so-far 1))
  (flet ((simplify (expr)
           ;; These simplifications are not strictly necessary; they
           ;; are only performed to make the macroexpansion of
           ;; WITH-TASK-HIERARCHY look prettier.
           (let ((expr (if (constantp expr)
                           (sb-int:constant-form-value expr)
                           expr)))
             (if (atom expr)
                 expr
                 (destructure-case expr
                   ((cl:+ a b) (cond ((eql a 0) b)
                                     ((eql b 0) a)
                                     (t expr)))
                   ((cl:* a b) (cond ((eql a 1) b)
                                     ((eql b 1) a)
                                     (t expr)))
                   (t expr))))))
    (let* ((old-total (td-total-count td))
           (count (td-count td))
           (new (simplify `(cl:* ,count-so-far ,count))))
      (setf (td-total-count td) (simplify `(cl:+ ,old-total ,new)))
      (dolist (child (td-childs td))
        (fill-total-counts child new))
      (when (td-singlep td)
        (assert (eql (td-total-count td) 1) ()
                "~@<You specified (:task ~A ...) albeit it must be :TASKS ~
                    as ~A will be created ~A times due to its parents.~@:>"
                (td-name td) (td-name td) (td-total-count td)))
      td)))
;;; Generate code to construct a TASK, and enqueue it in QUEUE.
(defun generate-enqueued-task (class &key name queue thread-fun)
  `(enqueue (make-instance ',class
              :name ,name
              :run-thread t
              :thread-fun ,thread-fun)
            ,queue))

;;; Grovel the compile-time task hierarchy top-down and generate code
;;; that will construct the same hierarchy at run-time:
;;; Let P be the task belonging to a parent TD, and Cs be P's
;;; children. We emit code such that the constructors for Cs are
;;; executed within P.
(defun generate-task-hierarchy (root-td barrier)
  (labels ((generate (td &optional toplevel)
             (let ((td-name (td-name td))
                   (td-body (td-body td))
                   (class   (if toplevel 'toplevel-task 'task)))
               `(dotimes (i ,(td-count td))
                  ,(generate-enqueued-task class
                    :name `(format nil "~A-~D" ',td-name i)
                    :queue td-name
                    :thread-fun `#'(lambda ()
                                     ,@(mapcar #'generate (td-childs td))
                                     (enter-barrier ,barrier)
                                     (block ,td-name ,@td-body)))))))
    (generate root-td t)))

 ) ;; (eval-when ...


;;; Now that we've got everything... let's get the party started!

(defmacro with-task-hierarchy (hierarchy definitions &body body)
  "Kitchen sink included."
  (multiple-value-bind (task-names td-map root-tds)
      (parse-task-hierarchy hierarchy definitions)
    (let ((tasks-ready (gensym "TASKS-READY-"))
          (totals (loop for task-name in task-names
                        collect (format-symbol nil "TOTAL-~A" task-name))))
      `(let ,(loop for task-name in task-names
                   for td = (find-task-descriptor task-name td-map)
                   for total in totals
                   collect `(,total ,(td-total-count td)))
         (with-synchronization-barrier (,tasks-ready (1+ (+ ,@totals)))
           (let ,(loop for name in task-names
                       collect `(,name (make-queue)))
             ;; The task hierarchy; all tasks will wait on TASKS-READY
             ;; and will enqueue themselves to the queues right above.
             ,@(loop for root-td in root-tds
                     collect (generate-task-hierarchy root-td tasks-ready))
             (enter-barrier ,tasks-ready)
             ,@(loop for name in task-names
                     for td = (find-task-descriptor name td-map)
                     if (td-singlep td)
                       collect `(let ((content  (list-queue-contents ,name)))
                                  (assert (length= content 1))
                                  (setq ,name (first content)))
                     else
                       collect `(setq ,name (list-queue-contents ,name)))
             ,@body))))))

;;;; WITH-PRODUCER-CONSUMER-TASKS

;;; TODO: - add body executed in the main thread
;;;       - make it possible for the producer / consumer
;;;         threads to get hold on the TASK-READY barrier,
;;;         so they can choose themselves at what point they're
;;;         set up.

(defmacro with-producer-consumer-tasks
    ((&key (n-producers 1) (n-consumers 1)) &body ((&key producer)
                                                   (&key consumer)))
  (assert producer) (assert consumer)
  `(with-task-hierarchy ((producer -> )
                         (consumer -> ))
       ((:tasks producer ,n-producers (funcall ,producer))
        (:tasks consumer ,n-consumers (funcall ,consumer)))
     (wait-until (become +dead+) producer consumer)))

;;;; WITH-PRODUCER-CONSUMER-THREADS
;;; Since tasks heavily depend on fluents, we don't want to use
;;; WITH-PRODUCER-CONSUMER-TASKS in fluent tests.
(defmacro with-producer-consumer-threads
    ((&key (n-producers 1) (n-consumers 1)) &body ((&key producer)
                                                   (&key consumer)))
  (assert producer) (assert consumer)
  `(let ((producer-thunk ,producer)
         (consumer-thunk ,consumer)
         (n-threads (+ ,n-producers ,n-consumers)))
     (with-synchronization-barrier (all-ready n-threads)
       (let ((producers (spawn-threads ,n-producers "Producer"
                                       (lambda ()
                                         (enter-barrier all-ready)
                                         (funcall producer-thunk))))
             (consumers (spawn-threads ,n-consumers "Consumer"
                                       (lambda ()
                                         (enter-barrier all-ready)
                                         (funcall consumer-thunk)))))
         (mapc #'join-thread producers)
         (mapc #'join-thread consumers)))))

;;;; Some more task-related utilities

;;; Most of them work either on a single task, or a list of tasks,
;;; so they can be used on the bindings established by
;;; (:THREAD X ...) and (:THREAD XS N ...).

(defun ensure-flattened-list (list)
  "If `list' contains nested lists, flatten them.
   Unlike ALEXANDRIA:FLATTEN this only flattens one level deep."
  (mappend #'ensure-list list))

(defun wait-until (predicate &rest tasks)
  "PREDICATE takes each one out of TASKS and should return a fluent.
   Wait until all the fluents return T."
  (let ((fluents (mapcar predicate (ensure-flattened-list tasks))))
    ;; Do not iterate over tasks and call WAIT-FOR on each; we want to
    ;; wait until all fluents return T at the -same- point of time.
    (wait-for (apply #'fl-and fluents))))

(defun become (&rest states)
  "Predicate suitable for WAIT-UNTIL."
  (let ((states (ensure-flattened-list states)))
    (dolist (state states)
      (check-type state status-indicator))
    #'(lambda (task)
        (fl-funcall #'member (status task) states))))

(defun no-longer (&rest states)
  "Complement of BECOME."
  (let ((states (ensure-flattened-list states)))
    (dolist (state states)
      (check-type state status-indicator))
    #'(lambda (task)
        (fl-funcall #'(lambda (s) (not (member s states))) (status task)))))

(defun task-status (task)
  (value (status task)))

(defun has-status (task status &rest more)
  (let ((actual-status (task-status task))
        (expected (cons status more)))
    (is (member actual-status expected)
        "~@<Task ~S has status ~S, ~:_expected ~:[~S~;one of ~{~S~^, ~}~]~:>"
        task actual-status more (if more expected status))))

(defun have-status (tasks status &rest more)
  (dolist (task tasks)
    (apply #'has-status task status more)))

(defun suspend-tasks (&rest tasks)
  (dolist (task (ensure-flattened-list tasks))
    (cpl-impl::suspend task)))

(defun wake-up-tasks (&rest tasks)
  (dolist (task (ensure-flattened-list tasks))
    (cpl-impl::wake-up task)))

(defun evaporate-tasks (&rest tasks)
  (dolist (task (ensure-flattened-list tasks))
    (cpl-impl::evaporate task)))

(defun evaporate-tasks-and-wait (&rest tasks)
  (apply #'evaporate-tasks tasks)
  (apply #'wait-until (become :evaporated) tasks))
