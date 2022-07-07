;;;
;;; Copyright (c) 2017-2022, Sebastian Koralewski <seba@cs.uni-bremen.de>
;;;
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(defsystem cram-cloud-logger
  :depends-on (:cram-language
               :cram-designators
               :cl-transforms
               :cl-transforms-stamped
               :cram-json-prolog
               :roslisp
               :cram-utilities
               :cram-manipulation-interfaces
               :cram-executive
               :cram-projection
               :cram-physics-utils)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:module "mapper"
      :components
      ((:file "cram-2-knowrob-mapper")))
     (:file "utils" :depends-on ("package"))
     (:file "failure-handler" :depends-on ("mapper"))
     (:file "cloud-logger-client" :depends-on ("package" "utils"))
     (:file "cloud-logger-query-handler" :depends-on ("package" "cloud-logger-client" "utils"))
     (:file "prolog-query-handler" :depends-on ("package" "utils" "cloud-logger-client"))
     (:file "knowrob-action-name-handler" :depends-on ("package" "utils"))
     (:file "logging-functions" :depends-on ("package" "cloud-logger-query-handler"))
     (:file "action-parameter-handler" :depends-on ("package" "logging-functions"))
     (:file "utils-for-perform" :depends-on ("package"
                                             "cloud-logger-query-handler"
                                             "prolog-query-handler"
                                             "knowrob-action-name-handler"
                                             "action-parameter-handler"
                                             "utils"
                                             "failure-handler"))
     (:file "object-interface" :depends-on ("package" "utils" "cloud-logger-client" "utils-for-perform"))))))
