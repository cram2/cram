;;;;
;;;; We decided to get rid of the portable-threads dependency because
;;;; "portable multithreading" in Common Lisp is more a fairytale than
;;;; reality. Even if we used portable-threads we'd check our
;;;; assumptions according to the implementation we use, SBCL. So why
;;;; bother with an additional source layer to wade through?
;;;;

(in-package :cram-utilities)

(defmacro defsubst (name arglist &body body)
  `(progn (declaim (inline ,name))
          (defun ,name ,arglist ,@body)))

(defsubst current-thread ()
  sb-thread:*current-thread*)

(defsubst all-threads ()
  (sb-thread:list-all-threads))

(defsubst thread-name (thread)
  (sb-thread:thread-name thread))

(defun spawn-thread (name function &rest args)
  (sb-thread:make-thread #'(lambda () (apply function args))
                         :name (string name)))

(defun spawn-threads (n name-prefix function)
  (loop for i from 1 to n
        collect (spawn-thread (format nil "~A-~D" name-prefix i) function)))

(defun run-in-thread (thread function &rest args)
  (sb-thread:interrupt-thread thread #'(lambda ()
                                         (apply function args))))

(defun kill-thread (thread)
  (when (sb-thread:thread-alive-p thread)
    (handler-case
        (prog1 t (sb-thread:terminate-thread thread))
      (error () nil))))

(defun thread-alive-p (thread)
  (sb-thread:thread-alive-p thread))

(defun join-thread (thread &key timeout
                                (on-timeout :timeout)
                                (on-failure :error))
  (if timeout
      (handler-case
          (sb-ext:with-timeout (+ timeout 0.1)
            (sb-sys:with-deadline (:seconds timeout)
              (sb-thread:join-thread thread :default on-failure)))
        (sb-ext:timeout ()
          on-timeout))
      (sb-thread:join-thread thread :default on-failure)))

;;; Locks

(defun make-lock (&key name)
  (sb-thread:make-mutex :name name))

(defstruct (recursive-lock (:include sb-thread:mutex)))

(defmacro with-lock-held ((lock &key whostate) &body body)
  (declare (ignore whostate))
  (let ((n-lock (gensym "LOCK+"))
        (n-body-fun (gensym "WITH-LOCK-HELD-BODY+")))
    `(flet ((,n-body-fun () ,@body))
       (declare (dynamic-extent #',n-body-fun))
       (let ((,n-lock ,lock))
         (if (recursive-lock-p ,n-lock)
             (sb-thread:with-recursive-lock (,n-lock)
               (,n-body-fun))
             (sb-thread:with-mutex (,n-lock)
               (,n-body-fun)))))))

;;; Condition variables

(defstruct condition-variable
  (lock (sb-int:missing-arg) :type sb-thread:mutex)
  (waitqueue
   (sb-thread:make-waitqueue)
   :type sb-thread:waitqueue))

(defun condition-variable-signal (cv)
  (declare (type condition-variable cv))
  (sb-thread:condition-notify (condition-variable-waitqueue cv)))

(defun condition-variable-broadcast (cv)
  (declare (type condition-variable cv))
  (sb-thread:condition-broadcast (condition-variable-waitqueue cv)))

(defun condition-variable-wait (cv)
  (declare (type condition-variable cv))
  (sb-thread:condition-wait (condition-variable-waitqueue cv)
                            (condition-variable-lock cv)))

#-sb-lutex
(defun condition-variable-wait-with-timeout (cv timeout)
  (declare (type condition-variable cv))
  (handler-case
      (sb-sys:with-deadline (:seconds timeout :override t)
        (sb-thread:condition-wait (condition-variable-waitqueue cv)
                                  (condition-variable-lock cv))
        t)
    (sb-sys:deadline-timeout ()
      nil)))

#+sb-lutex
(defun condition-variable-wait-with-timeout (cv timeout)
  (declare (type condition-variable cv))
  (handler-case
      (sb-ext:with-timeout timeout
        (sb-thread:condition-wait (condition-variable-waitqueue cv)
                                  (condition-variable-lock cv))
        t)
    (sb-ext:timeout ()
      nil)))

;;; Synchronized hash-table

;; Functions operating on one key like GETHASH, (SETF GETHASH),
;; REMHASH, but also CLRHASH are thread-safe operations on
;; synchronized hash-tables.

;; However, iterating over a hash-table via MAPHASH is not, hence the
;; WITH-HASH-TABLE-LOCKED macro is needed.

(defun make-synchronized-hash-table (&rest keys)
  (apply #'make-hash-table :synchronized t keys))

(defmacro with-hash-table-locked ((hash-table) &body body)
  `(sb-ext:with-locked-hash-table (,hash-table)
     ,@body))


;;; Thread-Local Bindings

(defsubst thread-local-binding-p (symbol)
  (nth-value 1 (sb-thread:symbol-value-in-thread symbol (current-thread) nil)))

