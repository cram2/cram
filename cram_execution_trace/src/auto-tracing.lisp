;;;
;;; Copyright (c) 2009 - 2010
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

(in-package #:cram-execution-trace)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Auto tracing
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;
;;; Auto tracing is a facility that automatically stores the episode knowledge
;;; for each executed plan after execution has finished. You have to set a
;;; directory where all episode knowledge files will be placed. The files will
;;; be named by the name of the top-level-plan and a current timestamp.
;;;
;;; Note that auto tracing is only about automatically saving episode
;;; knowledge to a file after an episode has ended. Independent from that
;;; fluent-tracing must be enabled seperately in order for the execution trace
;;; to actually contain traces. Use SETUP-AUTO-TRACING which enables both
;;; auto-tracing and fluent-tracing.
;;;

(defvar *auto-tracing-directory* nil
  "Path to directory where auto traces are saved.")

(defvar *auto-tracing-enabled* nil
  "Boolean indicating whether auto tracing is enabled.")

(defun enable-auto-tracing ()
  "Enables auto tracing. Note that fluents still won't be traced unless
   fluent-tracing is enabled. You might want to use SETUP-AUTO-TRACING."
  (check-directory *auto-tracing-directory*)
  (setf *auto-tracing-enabled* t))

(defun disable-auto-tracing ()
  (setf *auto-tracing-enabled* nil))

(defun auto-tracing-enabled ()
  *auto-tracing-enabled*)

(defun set-auto-tracing-directory (path &key (ensure-directory nil))
  "Sets the auto tracing directory and checks if it is a valid directory. If
   `ensure-directory' is not NIL it tries to create the given directory path."
  (let ((pathname (pathname-if-exists path)))
    (when (and (not pathname) ensure-directory)
      (ensure-directories-exist path))
    (check-directory path)
    (setf *auto-tracing-directory* (pathname path))))

(defun setup-auto-tracing (directory &key (ensure-directory nil))
  "Sets the auto tracing directory and checks if it is a valid directory. If
   `ensure-directory' is not NIL it tries to create the given directory
   path. After that it enables auto-tracing and fluent-tracing"
  (set-auto-tracing-directory directory :ensure-directory ensure-directory)
  (enable-auto-tracing)
  (enable-fluent-tracing))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Helper
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun auto-tracing-filepath (plan)
  "Generates filename and path for the auto tracing execution trace file for a
   given plan name."
  (flet ((name->path (name)
           (merge-pathnames (concatenate 'string name ".ek")
                            *auto-tracing-directory*)))
    (check-directory *auto-tracing-directory*)
    (let ((base-string (concatenate 'string (current-string-timestamp)
                                    "-" (sanitize-filename (symbol-name plan)))))
      (name->path (ensure-unused-string base-string
                                        (lambda (s)
                                          (probe-file (name->path s))))))))

(defun pathname-if-exists (path)
  (probe-file (pathname path)))

(defun check-directory (path)
  (unless (probe-file (pathname path))
    (error "Pathspec ~s does not designate a valid path for auto tracing."
           path)))
