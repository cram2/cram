;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :kipla-reasoning)

(defun get-classpath (package)
  (let* ((str (make-string-output-stream))
         (error-str (make-string-output-stream))
         (proc (sb-ext:run-program "rospack"
                                   (list "export"
                                         "--lang=java" "--attrib=classpath"
                                         package)
                                   :search t :output str :error error-str))
         (exit-code (sb-ext:process-exit-code proc)))
    (unless (eql exit-code 0)
      (error 'simple-error
             :format-control "rospack failed with message: `~a'"
             :format-arguments (list (get-output-stream-string error-str))))
    (substitute #\: #\Space (remove-if (rcurry #'member '(#\Newline))
                                       (get-output-stream-string str)))))

(let ((classpath-initialized nil))

  (defun init-swi-prolog ()
    ;; We need to set the CLASSPATH environment variable here
    ;; because some prolog stuff is using jpl.
    (unless classpath-initialized
      (sb-posix:putenv (format nil "CLASSPATH=~a~a"
                               (or (sb-posix:getenv "CLASSPATH") "")
                               (get-classpath "kipla")))
      (setf classpath-initialized t))
    (liswip:swi-init :options `("--nosignals"
                                "-f" ,(namestring
                                       (merge-pathnames
                                        #P"prolog/init.pl"
                                        (ros-load-manifest::ros-package-path "rosprolog")))
                                "-s" ,(namestring
                                       (merge-pathnames
                                        #P"prolog/init.pl"
                                        (ros-load-manifest::ros-package-path "kipla")))))))

;; (kipla:register-ros-init-function init-swi-prolog)
