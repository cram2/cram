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
;;; Saving / Loading
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric save-episode (episode dest &key if-exists)
  (:documentation
   "Save `episode' to `dest', so it can be restored by LOAD-EPISODE.")
  (:method (episode (dest string) &key (if-exists :error))
    (save-episode-to-file episode dest if-exists))
  (:method (episode (dest pathname) &key (if-exists :error))
    (save-episode-to-file episode dest if-exists))
  (:method (episode (dest stream) &key if-exists)
    (declare (ignore if-exists))
    (store episode dest 'episode-knowledge-backend)))

(defun save-episode-to-file (episode-knowledge dest if-exists)
  (with-backend 'episode-knowledge-backend
    (with-open-file (stream dest :direction :output
                            :element-type (stream-type *default-backend*)
                            :if-exists if-exists)
      (save-episode episode-knowledge stream))))

(defgeneric load-episode (source)
  (:documentation "Load episode knowledge that has been stored with
   SAVE-EPISODE. Returns an object of type OFFLINE-EPISODE-KNOWLEDGE.")
  (:method ((source string))
    (load-episode-from-file source))
  (:method ((source pathname))
    (load-episode-from-file source))
  (:method ((source stream))
    (restore source 'episode-knowledge-backend)))

(defun load-episode-from-file (source)
  (with-backend 'episode-knowledge-backend
    (with-open-file (stream source :direction :input
                            :element-type (stream-type *default-backend*)
                            :if-does-not-exist :error)
      (load-episode stream))))
