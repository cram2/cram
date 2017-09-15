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

(in-package :cram-dummy-process-modules)

(def-process-module fake-navigation (goal-pose)
  (format t "Fake navigation invoked with pose `~a'." goal-pose))

(def-process-module fake-ptu (desig)
  (format t "Fake ptu invoked with designator `~a'.~%" desig))

(def-process-module fake-manipulation (desig)
  (format t "Fake manipulation invoked with designator `~a'.~%" desig))

(def-process-module fake-perception (desig)
  (format t "Fake perception invoked with designator `~a'.~%" desig)
  (let ((new-desig (make-designator :object (description desig))))
    (with-slots (data valid) new-desig
      (setf data :data)
      (setf valid t))
    (list new-desig)))

(def-process-module fake-anything (desig)
  (format t "Fake anything invoked with designator `~a'.~%" desig))

(defun enable-fake-process-modules ()
  (cpm:process-module-alias :navigation 'fake-navigation)
  (cpm:process-module-alias :ptu 'fake-ptu)
  (cpm:process-module-alias :manipulation 'fake-manipulation)
  (cpm:process-module-alias :perception 'fake-perception)
  (cpm:process-module-alias :anything 'fake-anything))

(defmacro with-fake-process-modules (&body body)
  `(with-process-modules-running
       ((:navigation fake-navigation)
        (:ptu fake-ptu)
        (:manipulation fake-manipulation)
        (:perception fake-perception)
        (:anything fake-anything))
     ,@body))
