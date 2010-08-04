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

(in-package :kipla)

(defvar *waypoints-file* )

(defun drive-to-waypoints (waypoints callback)
  (loop for wp in waypoints
        for n from 0
        do (approach-waypoint wp t)
           (funcall callback n)))

(defun read-waypoints-file (filename)
  (with-open-file (strm filename :direction :input)
    (let ((*read-eval* nil))
      (read strm))))

(defun drive-to-waypoints-main (&optional (waypoints-file (second sb-ext:*posix-argv*)))
  (unwind-protect
       (progn
         (startup-ros)
         
         (let ((timeout (get-param "~timeout" 2))
               (status-topic (get-param "~status_topic" "~status")))
           (assert (probe-file waypoints-file) ()
                   "Waypoints file `~a' does not exist. Please call with a
            valid filename." waypoints-file)
           (flet ((callback (n)
                    (publish status-topic (make-instance 'std_msgs-msg:<string> :data (format nil "Waypoint ~a" n)))
                    (sleep timeout)))
             (let ((waypoints (mapcar (lambda (descr)
                                        (apply #'jlo:make-jlo-rpy descr))
                                      (read-waypoints-file waypoints-file))))
               (let ((*kipla-features* nil))
                 (advertise status-topic 'std_msgs-msg:<string>)
                 (drive-to-waypoints waypoints #'callback))))))
    (shutdown-ros)))