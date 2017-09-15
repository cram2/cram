;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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

(in-package :cl-user)

(defpackage :cram-process-modules
  (:documentation "Package containing process module interfaces.")
  (:use #:cpl
        #:cram-designators
        #:alexandria)
  (:import-from #:cram-prolog def-fact-group <- prolog)
  (:shadowing-import-from #:cram-prolog fail)
  (:import-from #:cram-utilities lazy-mapcar force-ll var-value)
  (:nicknames :cpm)
  (:export process-module name input feedback result
           status cancel priority caller
           pm-run pm-execute pm-execute-matching pm-cancel pm-status
           def-process-module register-process-module
           process-module-alias with-process-module-aliases
           get-running-process-module get-running-process-module-name
           get-process-module-names get-running-process-module-names
           on-process-module-started on-process-module-finished
           terminate-pm continue-pm with-process-modules-running
           abstract-process-module wait-for-process-module-running
           *process-module-debugger-hook* finished-fluent running-fluent
           with-process-module-registered-running
           def-asynchronous-process-module asynchronous-process-module
           on-input on-cancel on-run synchronization-fluent
           finish-process-module fail-process-module monitor-process-module
           matching-process-module available-process-module projection-running
           matching-process-module-names matching-available-process-modules))
