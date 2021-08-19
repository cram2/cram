;;;
;;; Copyright (c) 2021, Michael Neumann <mine1@uni-bremen.de>
;;;                     Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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

(in-package :cslg)

(defun generate-neem (&optional objects-to-fetch-deliever)
  ;; (setf cram-tf:*tf-broadcasting-enabled* t)
  (roslisp-utilities:startup-ros :name "cram" :anonymous nil)

  (let ((objects-str (roslisp:get-param "/neem_generator/objects"))
        objects '())
    (loop for x in  (split-sequence:split-sequence #\Space objects-str)
          do (setf objects (append objects (list (values (intern (string-upcase x) "KEYWORD"))))))
    (setf ccl::*is-logging-enabled* t)
    (setf ccl::*host* "'https://localhost'")
    (setf ccl::*cert-path* "'/home/koralewski/Desktop/localhost.pem'")
    (setf ccl::*api-key* "'K103jdr40Rp8UX4egmRf42VbdB1b5PW7qYOOVvTDAoiNG6lcQoaDHONf5KaFcefs'")
    (ccl::connect-to-cloud-logger)

    (let ((experiment-id (format nil "~d" (truncate (* 1000000 (cram-utilities:current-timestamp))))))
      (format t "Starting experiment ~a~%" experiment-id)

      (unwind-protect
           (if objects
               (urdf-proj::with-simulated-robot (demo::demo-random nil objects))
               (urdf-proj:with-simulated-robot (demo::demo-random)))
        (ccl::export-log-to-owl (concatenate 'string experiment-id ".owl"))
        (format t "Done with experiment ~a~%" experiment-id)
        (ccl::reset-logged-owl)))))
