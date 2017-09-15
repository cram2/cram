;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>,
;;;                     Nikolaus Demmel <demmeln@cs.tum.edu>
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

(in-package :cram-utilities)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Time
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(deftype timestamp ()
  'double-float)

(defun default-timestamp-function (&optional (what-time :real))
  "A function that returns the current time in seconds as a double
   float. Precision depends on the CL implementation, more precisely on the
   value of INTERNAL-TIME-UNITS-PER-SECOND."
  (float
   (/ (ecase what-time
        (:real (get-internal-real-time))
        (:run (get-internal-run-time)))
      internal-time-units-per-second)
   ;; CAVEAT: Using double-float-epsilon here is tied to timestamp being type
   ;; "double float"
   double-float-epsilon))

(defparameter *zero-time-seconds*
  #+sbcl (sb-ext:get-time-of-day)
  #-sbcl (error "Not supported")
  "Timestamp in seconds when lisp image was started. Used as a reference for
   to create 'small', 'relative' timestamps.")

(defun microsecond-timestamp-function (&key relative)
  "Returns current time in seconds as double precise to the micro second. If
   `relative' is true, the seconds will be counted from the startup of the
   lisp image, not from unix epoch (i.e. you get much smaller numbers, which
   could be nice if you are only interested in relative times)."
  #+sbcl
  (multiple-value-bind (sec usec) (sb-ext:get-time-of-day)
    (+ (if relative
           (- sec *zero-time-seconds*)
           sec)
       (/ usec 1d6)))
  #-sbcl (error "Not supported"))

(defvar *timestamp-function* #'default-timestamp-function
  "A function that returns the current time in seconds as a double float.")

(defun set-timestamp-function (fn)
  "Changes the timestamp function used by stamp-time"
  (setf *timestamp-function* fn))

(defun set-default-timestamp-function ()
  "Lets stamp-time use the default timestampt function"
  (setf *timestamp-function* #'default-timestamp-function))

(declaim (inline current-timestamp))
(defun current-timestamp (&optional what-time)
  "Returns the current time in seconds as a double float"
  (if what-time
      (default-timestamp-function what-time)
      (funcall *timestamp-function*)))

(defun current-string-timestamp ()
  "Returns the current time as string in the format YYYYMMDDHHMMSS. Could be
   used for filenames."
  (multiple-value-bind (second minute hour date month year)
      (get-decoded-time)
    (format nil "~d~2,'0d~2,'0d~2,'0d~2,'0d~2,'0d"
            year
            month
            date
            hour
            minute
            second)))
