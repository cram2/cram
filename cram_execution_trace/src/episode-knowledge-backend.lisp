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

(in-package :cram-execution-trace)

(eval-when (:load-toplevel)
  (defbackend episode-knowledge-backend
      :magic-number 966535004
      :old-magic-numbers (966535001 966535002 966535003)
      :extends (cl-store)))

;;; cl-store uses codes to about 45 aparently, so we start at a safe 80, but
;;; that number is arbitrary. (register-code would signal an error upon code
;;; reuse anyway).
(defparameter +episode-knowledge-code+ (register-code 80 'episode-knowledge))
(defparameter +code-code+ (register-code 81 'cpl-impl:code))
(defparameter +task-code+ (register-code 82 'cpl-impl:task))
(defparameter +cons-code+ (register-code 83 'cons))

;;; Version number so maybe we can support reading old versions in the future
(defconstant +episode-knowledge-backend-version+ 4)

;;; CONS (including lazy lists)
(defstore-episode-knowledge-backend (obj cons stream)
  (output-type-code +cons-code+ stream)
  (store-object (car obj) stream)
  (if (cut::lazy-cons-elem-p (cdr obj))
      ;;; FIXME / KLUDGE: For now cut off lazy lists, since we have infinite
      ;;; lazy lists and saving the et does not terminate.
      ;; (store-object (funcall (cut::lazy-cons-elem-generator (cdr obj)))
      ;;              stream)
      (store-object nil stream)
      (store-object (cdr obj) stream)))

;;; NOTE: No two restored cons cells will be eq (at the moment). This also
;;; means circular lists can't be restored, which is good enough for now.
(defrestore-episode-knowledge-backend (cons stream)
  (cons (restore-object stream)
        (restore-object stream)))

;;; EPISODE-KNOWLEDGE
(defstore-episode-knowledge-backend (obj episode-knowledge stream)
  (output-type-code +episode-knowledge-code+ stream)
  (store-object +episode-knowledge-backend-version+ stream)
  (store-object (zero-time obj) stream)
  (store-object (end-time obj) stream)
  (store-object (task-tree obj) stream)
  (store-object (traced-fluents-hash-table obj) stream))

(defrestore-episode-knowledge-backend (episode-knowledge stream)
  (flet ((do-restore ()
           (make-instance 'offline-episode-knowledge
             :zero-time (restore-object stream)
             :end-time (restore-object stream)
             :task-tree (restore-object stream)
             :execution-trace (restore-object stream))))
    (let ((version (restore-object stream)))
      (cond ((= version 3) ;; Backwards compatible version
             (warn "Loading episode-knowledge file of version ~a. The format
                    is compatible and the trace should load. However the
                    symbols used in goals and paths have changed (or rather
                    the package changed). This means, that the provided prolog
                    predicates will mostly be useless." version)
             (do-restore))
            ((= version +episode-knowledge-backend-version+)
             (do-restore))
            ((< version +episode-knowledge-backend-version+)
             (error "Trying to read old version ~a of episode-knowledge
                     file. Current version: ~a."
                    version +episode-knowledge-backend-version+))
            ((> version +episode-knowledge-backend-version+)
             (error "Wow. Episode-knowledge file with version number ~a from
                     the future.  Your version is: ~a. If you don't believe
                     time travel is possible, update CRAM."
                    version +episode-knowledge-backend-version+))))))

;;; CODE
(defstore-episode-knowledge-backend (obj cpl-impl:code stream)
  (output-type-code +code-code+ stream)
  (store-object (cpl-impl:code-sexp obj) stream)
  (store-object (cpl-impl:code-task obj) stream)
  (store-object (cpl-impl:code-parameters obj) stream))

(defrestore-episode-knowledge-backend (cpl-impl:code stream)
  (cpl-impl::make-code :sexp (restore-object stream)
                       :function nil
                       :task (restore-object stream)
                       :parameters (restore-object stream)))

;;; TASK
(defstore-episode-knowledge-backend (obj cpl-impl:task stream)
  (output-type-code +task-code+ stream)
  (store-object (cpl-impl:name (cpl-impl:status obj)) stream)
  (store-object (cpl-impl:result obj) stream))

(defrestore-episode-knowledge-backend (cpl-impl:task stream)
  (make-instance 'offline-task
    :status (make-instance 'offline-fluent
              :name (restore-object stream))
    :result (restore-object stream)))

(defrestore-episode-knowledge-backend (function stream)
  `(function ,(restore-object stream)))
