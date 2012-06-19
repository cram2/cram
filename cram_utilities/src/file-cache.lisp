;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :cut)

(defvar *file-cache* (make-hash-table :test 'equal))

(defstruct cache-entry
  file-name
  time-stamp
  value)

(defun cached-file-value (file-name)
  (declare (type pathname file-name))
  (let ((cached-entry (gethash file-name *file-cache*)))
    (when cached-entry
      (cond ((eql (cache-entry-time-stamp cached-entry)
                   (file-write-date file-name))
             (cache-entry-value cached-entry))
            (t (remhash file-name *file-cache*) nil)))))

(defun set-cached-file-value (file-name value)
  (setf (gethash file-name *file-cache*)
        (make-cache-entry
         :file-name file-name
         :time-stamp (file-write-date file-name)
         :value value))
  value)

(defsetf cached-file-value set-cached-file-value)

(defmacro with-file-cache (variable-name file-name value &body body)
  "Establishes a lexical binding of `variable-name'. The variable is
  bound to `value'. `value' is cached and the cached value stays valid
  until the file modification date of `file-name' changes."
  `(let ((,variable-name (or (cached-file-value ,file-name)
                             (set-cached-file-value ,file-name ,value))))
     ,@body))
