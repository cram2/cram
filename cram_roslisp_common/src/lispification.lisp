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
;;;

(in-package :cram-roslisp-common)

(defun lispify-ros-name (str &optional (package *package*))
  "Returns a lispified symbol correstonding to the string
  `str'. Lispification inserts - signs between camel-cased words and
  makes all characters uppercase."
  (cut:deprecate "Use of deprecated form CRAM-ROSLISP-COMMON:LISPIFY-ROS-NAME. Please use ROSLISP-UTILITIES:LISPIFY-ROS-NAME.")
  (labels ((char-ucase-p (c)
             (find c "ABCDEFGHIJKLMNOPQRSTUVWXYZ"))
           (char-lcase-p (c)
             (find c "abcdefghijklmnopqrstuvwxyz0123456789"))
           (char-case (c)
             (cond ((char-ucase-p c) :upper)
                   ((char-lcase-p c) :lower)
                   (t nil))))
    (let ((s (make-string-output-stream)))
      (loop for c across str
            with case = (char-case (elt str 0))
            when (and (eq case :lower)
                      (eq (char-case c) :upper))
              do (write-char #\- s)
            do (write-char c s) (setf case (char-case c)))
      (intern (string-upcase (get-output-stream-string s)) package))))

(defun rosify-lisp-name (sym)
  (cut:deprecate "Use of deprecated form CRAM-ROSLISP-COMMON:ROSIFY-LISP-NAME. Please use ROSLISP-UTILITIES:ROSIFY-LISP-NAME.")
  (with-output-to-string (strm)
    (loop for ch across (symbol-name sym)
          with upcase = t
          if (eql ch #\-) do (setf upcase t)
            else do
              (progn
                (if upcase
                    (princ (char-upcase ch) strm)
                    (princ (char-downcase ch) strm))
                (setf upcase nil)))))
