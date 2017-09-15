;;;
;;; Copyright (c) 2010, Tobias C Rittweiler <trittweiler@common-lisp.net>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :cpl-impl)

;;;; Customizations

(defvar *log-output* *trace-output*
  "Where should logging output go to?")

(defvar *log-right-margin* 130
  "Value for *PRINT-RIGHT-MARGIN* when printing log entries.
   Good values between 120 - 150, depending on your window width.")

;;;; LOG-EVENT

(defvar +available-log-tags+)

(defmacro log-event (&body clauses)
  "Logs an event specified by `clauses' to *LOG-OUTPUT*.

   Syntax:

      clauses ::= (:CONTEXT fmt-ctrl . fmt-args)
                | (:DISPLAY fmt-ctrl . fmt-args)
                | (:TAGS tag+)

   A log entry will look like

      <realtime>  ,  <runtime>  - <current task>  > <context>  ; <display>

   Where

      <realtime>     denotes a timestamp relative to *ZERO-REAL-TIME*,

      <runtime>      denotes a timestamp relative to *ZERO-RUN-TIME*,

      <current task> denotes the name of the currently active task,

      <context>      denotes the result of the format-control `fmt-ctrl'
                     applied to the format-arguments `fmt-args' of the
                     given :CONTEXT clause,

      <display>      ditto but for the :DISPLAY clause.


   The :TAGS clause specifies the log tags that must be active for
   this LOG-EVENT to emit any output.

   It is hence possible to specify the degree of logging in a very
   fine-grained way: log tags can be dynamically enabled by
   LOG-ENABLE, disabled by LOG-DISABLE, and set by LOG-SET.

   Most defined log tags correspond to constructs of cpl or the underlying
   implementation. Use the :debug tag for adding (temporary) debug messages.

   For convenience, the following constants are defined to enable a
   bunch of tags at once:

     +LOG-ALL+           - set of all available tags

     +LOG-DEFAULT+       - a predefined default set

     +LOG-VERBOSE+       - +LOG-DEFAULT+ + more

     +LOG-VERY-VERBOSE+  - +LOG-VERBOSE+ + even more

     +LOG-LANGUAGE+      - set of all CRAM language constructs.

   See also

     LIST-AVAILABLE-LOG-TAGS

     LIST-ACTIVE-LOG-TAGS

     DEFINE-LOG-TAGS

     *LOG-RIGHT-MARGIN*
"
  (let ((ctx-control   "") (ctx-args   '())
        (displ-control "") (displ-args '())
        (tags '()))
    (dolist (clause clauses)
      (destructure-case clause
        ((:context ctrl . args)
         (multiple-value-setq (ctx-control ctx-args)
           (values ctrl args)))
        ((:display ctrl . args)
         (multiple-value-setq (displ-control displ-args)
           (values ctrl args)))
        ((:tags . ts)
         (setq tags (ensure-list ts))
         (mapc #'(lambda (tag)
                   (assert (member tag +available-log-tags+) ()
                           "~S not a known log tag. See DEFINE-LOG-TAGS." tag))
               (ensure-list ts)))))
    (assert tags () "Missing (:TAGS ...) clause.")
    `(when (logging-enabled-p ,@tags)
       (%log-event ,ctx-control   (list ,@ctx-args)
                   ,displ-control (list ,@displ-args)))))

(declaim (inline logfmt))
(defun logfmt (stream format-control &rest format-args)
  ;; We first format into a string, and then output the whole line at
  ;; once, so concurrent output is unlikely to lead to clobbering.
  (fresh-line stream)
  (write-line (apply #'format nil format-control format-args))
  (force-output stream))

(defun %log-event (ctx-control ctx-args display-control display-args)
  (let ((*print-right-margin* *log-right-margin*))
    (logfmt *log-output*
            "~8@A , ~8A - ~20A > ~30@<~?~> ; ~@<~?~:>"
            (what-time-is-it :real)
            (what-time-is-it :run)
            (who-am-i)
            ctx-control ctx-args
            display-control display-args))
  (values))

(define-task-variable *zero-real-time* nil
  "Base timestamp to compute relative real-time timestamps from.
   This variable gets its value from the top-level task, all sub tasks
   will just inherit it."
  :type (or null timestamp)
  :init-function
  #'(lambda (task parent-task previous-value)
      (declare (ignore task parent-task))
      (or previous-value (current-timestamp :real))))

(define-task-variable *zero-run-time* nil
  "Like *ZERO-REAL-TIME*, but for run-time timestamps."
  :type (or null timestamp)
  :init-function
  #'(lambda (task parent-task previous-value)
      (declare (ignore task parent-task))
      (or previous-value (current-timestamp :run))))

(defun what-time-is-it (kind)
  "Current timestamp relative to *ZERO-REAL-TIME*, or *ZERO-RUN-TIME*
   depending on `kind'."
  (flet ((%what-time-is-it (current zero)
           (if zero
               (format nil "~,3F" (- current zero))
               "n/a")))
    (ecase kind
      (:real (%what-time-is-it (current-timestamp :real) *zero-real-time*))
      (:run  (%what-time-is-it (current-timestamp :run)  *zero-run-time*)))))

(defun truncate-string (string max-length)
  "Truncate `string' if its length exceeds `max-length'.  We truncate
   in the middle of the string because start and end are usually
   crucial to discriminate the abbreviation."
  (let ((length (length string)))
    (if (<= length max-length)
        string
        (let* ((diff     (+ (- length max-length) 2))
               (length/2 (truncate length 2)))
          (multiple-value-bind (left-diff right-diff)
              (values (ceiling diff 2) (truncate diff 2))
            (concatenate 'string
                         (subseq string 0 (- length/2 left-diff))
                         ".."
                         (subseq string (+ length/2 right-diff))))))))

(defun task-abbreviated-name (task &optional (length 20))
  (truncate-string (string (name task)) length))

(defun who-am-i ()
  "Either the name of the current task, or of the current thread."
  (task-abbreviated-name (current-task) 20))


;;;; DEFINE-LOG-TAGS

(defvar *log-tag-table* (make-hash-table)) 

(defun %define-log-tags (names)
    (let ((table *log-tag-table*))
      (dolist (name names)
        (unless (gethash name table)
          (setf (gethash name table) (1+ (hash-table-count table)))))))
 
(defmacro define-log-tags (&body names)
  (assert (<= (length names) 32) ()
          "~@<We're using (UNSIGNED-BYTE 32) for the *ACTIVE-LOG-MASK* ~
              at the moment, so we're limited to at most 32 tags. For ~
              more, some tweaking is neccessary.~@:>")
  `(eval-when (:load-toplevel :execute)
     (%define-log-tags ',names)
     (setq +available-log-tags+ ',names)))

(define-log-tags
  ;; Use :DEBUG when adding LOG-EVENT during debugging..
  :debug
  ;; default
  :finish :send-event :receive-event :debugger
  ;; verbose
  :propagate-event :join-task :wait-for :register-child :teardown
  ;; very verbose
  :event-loop :change-status :gc
  ;; language internals
  :on-suspension :with-parallel-childs
  ;; language
  :fail :par :pursue :try-all :try-in-order :with-tags)

(defparameter +log-all+
  +available-log-tags+)

(defparameter +log-default+
  '(:debug
    :finish
    :send-event
    :receive-event
    :debugger))

(defparameter +log-verbose+
  `(:propagate-event
    :join-task
    :wait-for
    :register-child
    :teardown
    ,@+log-default+))

(defparameter +log-very-verbose+
  `(:event-loop
    :change-status
    :gc
    ,@+log-verbose+))

(defparameter +log-language+
  '(:fail
    :par
    :pursue
    :try-all
    :try-in-order
    :with-parallel-childs
    :with-tags))

(defvar *active-log-mask* 0
  "A bitmask representing the currently enabled log tags.")
(declaim (type (unsigned-byte 32) *active-log-mask*))

(declaim (inline log-tag-offset))
(declaim (ftype (function (symbol) (values (integer 0 31) &optional))
                log-tag-offset))
(defun log-tag-offset (tag)
  (or (gethash tag *log-tag-table*)
      (error "~@<Do not know about log tag ~S; you probably forgot ~
                 to adjust DEFINE-LOG-TAGS.~@:>"
             tag)))

(defun log-mask (tags)
  (let ((mask 0))
    (declare (type (unsigned-byte 32) mask))
    (dolist (tag tags mask)
      (setf (ldb (byte 1 (log-tag-offset tag)) mask) 1))))

(defmacro logging-enabled-p (&rest tags)
  "Is any tag in `tags' active at the moment?"
  `(locally  (declare (optimize (speed 3) (safety 0)))
     (logtest *active-log-mask* ,(log-mask tags))))

(defun list-available-log-tags ()
  "List all available log tags."
  +available-log-tags+)

(defun list-active-log-tags ()
  "List all log tags active at the moment."
  (loop for tag being the hash-key in *log-tag-table*
                using (hash-value offset)
        when (logbitp offset *active-log-mask*)
          collect tag))

(defun log-enable (tags)
  "Enable each tag in `tags'.
   Corresponding LOG-EVENTs will begin to emit output."
  (setf *active-log-mask* (logior *active-log-mask* (log-mask tags)))
  (list-active-log-tags))

(defun log-disable (tags)
  "Disable each tag in `tags'.
   Corresponding LOG-EVENTs will cease to emit any output."
  (setf *active-log-mask* (logandc2 *active-log-mask* (log-mask tags)))
  (list-active-log-tags))

(defun log-set (tags)
  "Enable only the tags in `tags', disabling all other tags first.
   Corresponding LOG-EVENTs will being / cease to emit output."
  (setf *active-log-mask* (log-mask tags))
  (list-active-log-tags))


;;;
;;; Functionality added by Jan Winkler (supporting functions for
;;; cram_beliefstate)
;;;


(defmacro log-block (begin-hook parameters end-hook &body body)
  `(let ((log-id (first (funcall ,begin-hook ,@parameters)))
         (result t)) ;; Implicit success
     (labels ((log-succeed ()
                (setf result t))
              (log-fail ()
                (setf result nil)))
       (declare (ignorable (function log-succeed)))
       (declare (ignorable (function log-fail)))
       (unwind-protect
            ,@body
         (funcall ,end-hook log-id result)))))
