
(defpackage :tcr.synchronization-tools
  (:nicknames :tcr.sync :synchronization-tools)
  (:use :common-lisp
        :sb-thread
        :sb-rt)
  (:import-from :sb-int #:missing-arg #:bug)
  (:import-from :sb-sys #:without-interrupts
                        #:with-interrupts
                        #:with-local-interrupts
                        #:allow-with-interrupts)
  (:import-from :sb-ext #:atomic-incf #:atomic-decf)
  (:shadow #:barrier)
  (:export
   ;; Buckets
   #:with-buckets
   ;; Barriers
   #:barrier
   #:cyclic-barrier
   #:make-synchronization-barrier
   #:with-synchronization-barrier
   #:with-synchronization-barriers
   #:enter-barrier
   #:break-barrier
   #:reset-cyclic-barrier
   #:increment-barrier-threshold
   #:decrement-barrier-threshold
   ;; Gates
   #:gate
   #:make-gate
   #:open-gate
   #:close-gate
   #:wait-open-gate
   #:open-gate-p))

(in-package :tcr.sync)

;;;; Utils

(defmacro with-struct ((conc-name &rest names) obj &body body)
  "Like with-slots but works only for structs."
  (flet ((reader (slot) (intern (concatenate 'string
					     (symbol-name conc-name)
					     (symbol-name slot))
				(symbol-package conc-name))))
    (let ((tmp (gensym "OO-")))
      ` (let ((,tmp ,obj))
          (symbol-macrolet
              ,(loop for name in names collect 
                     (typecase name
                       (symbol `(,name (,(reader name) ,tmp)))
                       (cons `(,(first name) (,(reader (second name)) ,tmp)))
                       (t (error "Malformed syntax in WITH-STRUCT: ~A" name))))
            ,@body)))))

(eval-when (:compile-toplevel)
  (define-condition compile-time-assertion-failed (sb-c:compiler-error)
    ((test-form
      :initarg :test-form
      :initform (missing-arg)
      :reader compile-time-assertion-failed--test-form))
    (:report (lambda (condition stream)
               (format stream "~@<The compile-time assertion ~S failed.~@:>"
                       (compile-time-assertion-failed--test-form condition))))))

(defmacro compile-time-assert (test-form)
  ;; We signal a FATAL-COMPILER-ERROR so SBCL will abort the
  ;; compilation of the whole file, and not just of the current
  ;; top-level form.
  `(eval-when (:compile-toplevel)
     (assert ,test-form ()
             'sb-c:fatal-compiler-error
             :condition (make-condition 'compile-time-assertion-failed
                                        :test-form ',test-form))))

(eval-when (:compile-toplevel :load-toplevel :execute)
  (defun slot-physical-size (structure-name slot-name)
    "Returns physical size (in words) of slot named SLOT-NAME
   in structure named STRUCTURE-NAME."
    (let* ((dd   (sb-kernel::find-defstruct-description structure-name))
           (dsd  (find slot-name (sb-kernel::dd-slots dd)
                       :test #'string= 
                       :key #'sb-kernel::dsd-name))
           (type (sb-kernel::dsd-type dsd))
           (rsd  (sb-kernel::structure-raw-slot-data type)))
      (if rsd
          (sb-kernel::raw-slot-data-n-words rsd)
          1))))

;;;; Synchronized Collectors

;;; Assumptions:
;;;
;;;   Only the write accesses (%SYNCHRONIZED-COLLECT-INTO) are locked,
;;;   reads (COLLECTOR-LIST) are not. We assume that reading the CDR,
;;;   and setting the CDR of a cons cell are atomic operations wrt
;;;   each other. It is on SBCL+x86oids.

;;; Todo:
;;;
;;;   Perhaps it's possible to implement this lock-free?
;;;
;;;   Perhaps the assumption is bogus? Do I need memory fences?
;;;   In that case, perhaps better use an R/W lock?
;;;
;;;   WITH-BUCKETS: Make the COLLECTOR-NAME optional, default to
;;;   COLLECT-INTO interned into *PACKAGE*.
;;;
;;;   Rename COLLECTOR to BUCKET.

(defconstant +anonymous-collector+ '%anonymous-collector%)

(defstruct (collector
             (:constructor make-collector
                (&optional name
                 &aux (%list (list (or name +anonymous-collector+)))
                      (tail %list))))
  (%list (missing-arg) :type cons)      ; the CAR is the name
  (tail  (missing-arg) :type cons))

(defstruct (synchronized-collector
             (:include collector)
             (:constructor make-synchronized-collector
                           (&optional name
                            &aux (%list (list (or name +anonymous-collector+)))
                                 (tail %list)
                                 (mutex (make-mutex :name name)))))
  (mutex (missing-arg) :type mutex))

(defmethod print-object ((collector collector) stream)
  (print-unreadable-object (collector stream :type t :identity t)
    (let ((internal-head (car (collector-%list collector))))
      (unless (eq internal-head +anonymous-collector+)
        (format stream "~S" internal-head)))))

(declaim (inline collector-list))
(defun collector-list (collector)
  (declare (collector collector))
  (declare (optimize (speed 3) (safety 1)))
  (cdr (collector-%list collector)))

(defun %collect-into (collector thing)
  (declare (collector collector))
  (declare (optimize (speed 3) (safety 1)))
  (with-struct (collector- tail) collector
    (let ((new-cdr (list thing)))
      (setf (cdr tail) new-cdr)
      (setf tail new-cdr)
      thing)))

(defun %synchronized-collect-into (collector thing)
  (declare (synchronized-collector collector))
  (declare (optimize (speed 3) (safety 1)))
  (with-struct (synchronized-collector- tail mutex) collector
    (let ((new-cdr (list thing)))
      (with-mutex (mutex)
        (setf (cdr tail) new-cdr)
        (setf tail new-cdr)
        thing))))

(defmacro with-buckets (collector-name (bucket &rest more-buckets) &body body)
  (labels ((parse-bucket-designator (bucket)
             (if (symbolp bucket)
                 (list bucket (string bucket) nil)
                 (destructuring-bind (bucket-var &key (name (string bucket-var))
                                                      (synchronized))
                     bucket
                   (check-type bucket-var symbol)
                   (check-type name string)
                   (check-type synchronized boolean)
                   (list bucket-var name synchronized))))
           (parse-buckets (buckets)
             (loop for bucket in buckets
                   for (var name synchronized)
                            = (parse-bucket-designator bucket)
                   for cvar = (gensym (format nil "COLLECTOR-<~A>+" var))
                   collect var          into vars
                   collect cvar         into collector-vars
                   collect name         into names
                   collect synchronized into flags
                   finally
                     (return (values vars collector-vars names flags)))))
    (multiple-value-bind (bucket-vars collector-vars names synchronized-flags)
        (parse-buckets (cons bucket more-buckets))
      `(let ,(loop for collector-var in collector-vars
                   for name in names
                   for synchronized in synchronized-flags
                   for constructor = (if synchronized
                                         'make-synchronized-collector
                                         'make-collector)
                   collect `(,collector-var (,constructor ,name)))
         (macrolet ((,collector-name (bucket form)
                      (flet ((generate-collect (bucket form)
                               (ecase bucket
                                 ,@(loop
                                     for bucket-var    in bucket-vars
                                     for collector-var in collector-vars
                                     for synchronized  in synchronized-flags
                                     for fn = (if synchronized
                                                  '%synchronized-collect-into
                                                  '%collect-into)
                                     collecting
                                       `(,bucket-var `(,',fn ,',collector-var
                                                               ,form))))))
                        (etypecase bucket
                          (symbol (generate-collect bucket form))
                          ((cons (eql values) *)
                           (let* ((buckets (cdr bucket))
                                  (gensyms (loop repeat (length buckets)
                                                 collect (gensym))))
                             `(multiple-value-bind ,gensyms ,form
                                ,@(loop
                                    for bucket in buckets
                                    for gensym in gensyms
                                    collect
                                      (generate-collect bucket gensym)))))))))
           (symbol-macrolet 
               ,(loop for bucket-var    in bucket-vars
                      for collector-var in collector-vars
                      collect `(,bucket-var (collector-list ,collector-var)))
             ,@body))))))

(defmacro collect-into (&whole whole bucket-designator &body (form))
  (declare (ignore bucket-designator form))
  `(error "~@<COLLECT-INTO not used within lexical scope of a WITH-BUCKETS: ~
                ~2I~_~S~@:>" ',whole))


;;;; Synchronization Barrier

;;; TODO
;;;
;;;      open question: If thread "dies" while waiting on barrier;
;;;                     should the count be decremented?

(deftype machine-word ()
  'sb-vm:word)

(defconstant +most-positive-machine-word+ (1- (expt 2 sb-vm:n-word-bits)))

(deftype barrier-state ()
  `(member :initial :entered :broken :finished))

(defstruct barrier
  (mutex     (missing-arg) :type mutex                   :read-only t)
  (waitq     (missing-arg) :type waitqueue               :read-only t)
  (name      nil           :type (or null simple-string) :read-only t)
  (threshold (missing-arg) :type machine-word)
  (count     0             :type machine-word)
  (state     :initial      :type barrier-state))

(defstruct (cyclic-barrier (:include barrier))
  (reset-counter 0 :type machine-word))

;;; Assumptions:
;;;
;;;   In the PRINT-OBJECT method, we assume that reading certain slots
;;;   can be safely performed without taking up garbage while not
;;;   holding on the lock. We do that because trying to grab a lock in
;;;   a PRINT-OBJECT method sounds pretty aweful (the :WAITP parameter
;;;   of WITH-MUTEX might come handy, though.)
;;;
;;;   Here, we check that these slots have a size of one machine word,
;;;   and can hence be expected to be read/written atomically.

(compile-time-assert (= (slot-physical-size 'barrier 'name) 1))
(compile-time-assert (= (slot-physical-size 'barrier 'state) 1))
(compile-time-assert (= (slot-physical-size 'barrier 'count) 1))
(compile-time-assert (= (slot-physical-size 'barrier 'threshold) 1))
(compile-time-assert (= (slot-physical-size 'cyclic-barrier 'reset-counter) 1))

(defmethod print-object ((barrier barrier) stream)
  (print-unreadable-object (barrier stream :type t :identity t)
    (with-struct (barrier- name state count threshold) barrier
      (format stream "~@[~S ~](~S~@[ ~D~]) [~D/~D]"
              name
              state
              (when (cyclic-barrier-p barrier)
                (cyclic-barrier-reset-counter barrier))
              count
              threshold))))

(defun make-synchronization-barrier (n &key name recyclable)
  (flet ((generate-name (name thing)
           (and name (format nil "barrier ~A's ~A" name thing))))
    (let ((m (make-mutex     :name (generate-name name "lock")))
          (q (make-waitqueue :name (generate-name name "condition variable"))))
      (if recyclable
          (make-cyclic-barrier :name name :threshold n :mutex m :waitq q)
          (make-barrier        :name name :threshold n :mutex m :waitq q)))))

(defmacro with-synchronization-barrier
    ((var count &key (name (string var)) recyclable) &body body)
  `(let ((,var (make-synchronization-barrier ,count
                                             :name ',name
                                             :recyclable ,recyclable)))
     ,@body))

(defmacro with-synchronization-barriers
    (((&whole 1st-clause var count &key name recyclable) &rest more) &body body)
  (declare (ignore var count name recyclable))
  `(let ,(loop for clause in (cons 1st-clause more)
               collecting
                 (destructuring-bind (var count &key (name (string var))
                                                     recyclable)
                     clause
                   `(,var (make-synchronization-barrier ,count
                           :name ',name
                           :recyclable ,recyclable))))
     ,@body))

(define-condition broken-barrier (error)
  ((barrier
    :initarg :barrier
    :initform (missing-arg)
    :reader broken-barrier--barrier
    :type barrier))
  (:report (lambda (condition stream)
             (format stream "~@<The barrier ~S was broken.~@:>"
                     (broken-barrier--barrier condition)))))

(define-condition stale-barrier (error)
  ((barrier
    :initarg :barrier
    :initform (missing-arg)
    :reader stale-barrier--barrier
    :type barrier))
  (:report (lambda (condition stream)
             (format stream "~@<The barrier ~S is stale.~@:>"
                     (stale-barrier--barrier condition)))))

(define-condition invalid-threshold-delta (error)
  ((barrier
    :initarg :barrier
    :initform (missing-arg)
    :reader invalid-threshold-delta--barrier
    :type barrier)
   (delta
    :initarg :delta
    :initform (missing-arg)
    :reader invalid-threshold-delta--delta
    :type machine-word)
   (threshold
    :initarg :barrier
    :initform (missing-arg)
    :reader invalid-threshold-delta--threshold
    :type machine-word))
  (:report (lambda (condition stream)
             (format stream 
                     "~@<While trying to decrement threshold of ~S: ~2I~_~
                         delta ~D is too large for a threshold of ~D.~@:>"
                     (invalid-threshold-delta--barrier condition)
                     (invalid-threshold-delta--delta condition)
                     (invalid-threshold-delta--threshold condition)))))

(defun enter-barrier (barrier &key (if-broken :error) (if-stale :error))
  (declare (type barrier barrier))
  (declare (values (or barrier null)))
  (with-struct (barrier- mutex waitq state count threshold) barrier
    (prog (aux-state)
     main
       ;; Disable interrupts to decrement the BARRIER-COUNT again in
       ;; case of ungraceful unwind due to interrupt.
       (with-mutex (mutex)
         (without-interrupts
           (case state
             (:entered)
             (:initial  (setf state :entered))
             (:broken   (go broken))
             (:finished (go stale)))
           ;; ATOMIC-INCF because of the comment in the UWP cleanup
           ;; clause. The INCF can never wrap around because THRESHOLD
           ;; is a) an MACHINE-WORD, too, and b) an upper bound for
           ;; COUNT.
           (atomic-incf (barrier-count barrier))
           (cond ((< count threshold)
                  (let ((ok? nil))
                    (unwind-protect
                         (progn
                           (with-local-interrupts
                             (loop do (condition-wait waitq mutex)
                                   until (member state '(:finished :broken))))
                           (setq ok? t))
                      ;; An interrupted CONDITION-WAIT may lead to
                      ;; not holding MUTEX here, but that's no
                      ;; problem as ENTER-BARRIER is the only
                      ;; place modifying COUNT, and we know here
                      ;; that an INCF must have happened before.
                      ;; However: Due to that leakage, we cannot
                      ;; reset STATE to :INITIAL because another
                      ;; thread could enter the barrier between
                      ;; the DECF and this hypothetical SETF.
                      (unless ok?
                        ;; Needs to be atomic explicitly because DECF
                        ;; is two steps and we may not have the lock.
                        (atomic-decf (barrier-count barrier))))))
                 (t
                  ;; Leave interrupts disable here, so we can be
                  ;; sure that waked threads ee :FINISHED. We
                  ;; assume no unwind can happen here.
                  (condition-broadcast waitq)
                  (setf state :finished)))
           (case (setq aux-state state)
             (:finished (go finish))
             (:broken   (go broken))
             (:entered  (go bug--invalid-state))
             (:initial  (go bug--invalid-state)))))
     finish
       (return barrier)
     ;; Make sure to signal errors outside WITH-MUTEX, otherwise
     ;; waiting threads would not be waked up in parallel.
     broken
       (ecase if-broken
         ((:error nil) (error 'broken-barrier :barrier barrier))
         ((:punt)      (return nil)))
     stale
       (ecase if-stale
         ((:error nil) (error 'stale-barrier :barrier barrier))
         ((:punt)      (return nil)))
     bug--invalid-state
       (bug "~@<~S had crossed barrier ~S which was in an invalid ~
                  state of ~S at that point.~:@>"
            *current-thread* barrier aux-state))))

(defun break-barrier (barrier)
  (declare (type barrier barrier))
  (declare (values barrier))
  (prog (state)
     main
       (with-recursive-lock ((barrier-mutex barrier))
         (case (setq state (barrier-state barrier))
           ((:initial :entered)            
            (without-interrupts
              (condition-broadcast (barrier-waitq barrier))
              (setf (barrier-state barrier) :broken))
            (go finish))
           ((:broken :finish)
            (go stale))))
     finish
       (return barrier)
     stale
       ;; FIXME: really when :finished?
       (error 'stale-barrier :barrier barrier)))

;;; FIXME: Think about these. State?

(defun increment-barrier-threshold (barrier &optional (delta 1))
  (declare (barrier barrier) (machine-word delta))
  (with-recursive-lock ((barrier-mutex barrier))
    (incf (barrier-threshold barrier) delta))
  barrier)

(defun decrement-barrier-threshold (barrier &optional (delta 1))
  (declare (barrier barrier) (machine-word delta))
  (with-struct (barrier- mutex waitq state threshold count) barrier
    (prog (aux-threshold)
       main
         (with-recursive-lock (mutex)
           (when (< threshold delta)
             (setq aux-threshold threshold)
             (go invalid-delta))
           (without-interrupts
             (let ((new-threshold (decf threshold delta)))
               (unless (< count new-threshold)
                 (condition-broadcast waitq)
                 (setf state :finished))
               (go finish))))
        finish
          (return barrier)
        invalid-delta
          (error 'invalid-threshold-delta
                 :barrier barrier
                 :threshold aux-threshold
                 :delta delta))))

(defun reset-cyclic-barrier (barrier &optional new-threshold)
  (declare (cyclic-barrier barrier))
  (with-struct (cyclic-barrier- mutex state threshold count reset-counter)
      barrier
    (prog (aux-state)
       main
         (with-mutex (mutex)
           (case (setq aux-state state)
             ((:finished :broken :initial)
              (without-interrupts
                (incf reset-counter)
                (setf state :initial)
                (setf count 0)
                (when new-threshold
                  (setf threshold new-threshold)))
              (go finish))
             ((:entered)
              (go invalid-state))))
       finish
         (return barrier)
       invalid-state
         (error "~S: Trying to reset barrier being in use: ~S"
                *current-thread* barrier))))


;;;; Read/Write locks

;; http://en.wikipedia.org/wiki/Seqlock
;; http://en.wikipedia.org/wiki/Spinlock
;; http://en.wikipedia.org/wiki/Read-copy-update

;; http://www.cs.rochester.edu/research/synchronization/pseudocode/rw.html
;; http://www.dfki.de/web/forschung/publikationen?pubid=4422
;; ftp://ftp.cs.toronto.edu/pub/parallel/Krieger_etal_ICPP93.ps.Z


;;;; Seqlocks

;; http://blogs.sun.com/dave/entry/seqlocks_in_java
;; http://blogs.sun.com/dave/resource/synchronization-public2.pdf


;;;; Gates

;;; TODO: implement lock-free, using FUTEXES on Linux

(defconstant +gate-open+ 0)
(defconstant +gate-closed+ 1)
(defstruct (gate (:constructor %make-gate))
  (mutex (missing-arg) :type mutex)
  (waitq (missing-arg) :type waitqueue)
  (state +gate-closed+ :type fixnum)
  (name  nil           :type (or null simple-string)))

(defmethod print-object ((gate gate) stream)
  (print-unreadable-object (gate stream :type t :identity t)
    (format stream "~@[~S ~](~S)"
            (gate-name gate)
            (case (gate-state gate)
              (#.+gate-open+   :open)
              (#.+gate-closed+ :closed)
              (t "<invalid state>")))))

(defun make-gate (&key name)
  (flet ((generate-name (thing)
           (when name
             (format nil "gate ~S's ~A" name thing))))
    (%make-gate
     :name name
     :mutex (make-mutex     :name (generate-name "lock"))
     :waitq (make-waitqueue :name (generate-name "condition variable"))
     :state +gate-closed+)))

(defun open-gate (gate)
  (declare (gate gate))
  (with-mutex ((gate-mutex gate))
    (setf (gate-state gate) +gate-open+)
    (condition-broadcast (gate-waitq gate)))
  gate)

(defun close-gate (gate)
  (declare (gate gate))
  (loop until (eql (sb-ext:compare-and-swap (gate-state gate)
                                            +gate-open+
                                            +gate-closed+)
                   +gate-open+))
  gate)

(defun wait-open-gate (gate)
  (declare (gate gate))
  (with-mutex ((gate-mutex gate))
    (loop while (not (gate-open-p gate))
          do (condition-wait (gate-waitq gate) (gate-mutex gate))))
  gate)

(defun gate-open-p (gate)
  (declare (gate gate))
  (eql (gate-state gate) +gate-open+))

;;; http://blog.boyet.com/blog/blog/someone-is-wrong-on-the-internet-mdash-the-lock-free-stack-edition/
;;; http://newsgroups.derkeiler.com/Archive/Comp/comp.programming.threads/2007-04/msg00125.html
;;; http://msdn.microsoft.com/en-us/magazine/cc163715.aspx

;;; http://locklessinc.com/articles/locks/
;;; http://groups.google.com/group/comp.unix.programmer/msg/212c43216ba3e2b4?hl=en&
;;; http://groups.google.com/groups?selm=3C231C0A.6AB363D9%40genuity.com