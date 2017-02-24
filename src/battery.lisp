;;;
;;; Copyright (c) 2017, Mihai Pomarlan <blandc@cs.uni-bremen.de>
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

(in-package :commander)


;; Battery fluents will have values that are either NIL or a list (robot-name battery-level battery-drain)
;; Drain can be negative, which means the battery is being charged.
(defparameter *red-wasp-battery* (cpl-impl:make-fluent :name :red-wasp-battery :value nil))
(defparameter *blue-wasp-battery* (cpl-impl:make-fluent :name :blue-wasp-battery :value nil))
(defparameter *monitoring-battery* (cpl-impl:make-fluent :name :battery-monitoring :value nil))
(defvar *battery-monitoring-publisher* nil)
(defvar *battery-publisher-thread* nil)

(defun battery-callback (robot-name fl msg)
  (when (and msg robot-name fl)
    (roslisp:with-fields (battery_level battery_drain) msg
      (setf (cpl-impl:value fl) (list robot-name battery_level battery_drain)))))

(defun get-battery-monitor-message ()
  (let* ((red-battery (when (cpl-impl:value *red-wasp-battery*)
                        (roslisp:make-message "sherpa_msgs/LoggedBattery"
                                              :robot_name "red_wasp"
                                              :battery_level (second (cpl-impl:value *red-wasp-battery*))
                                              :battery_drain (third (cpl-impl:value *red-wasp-battery*)))))
         (blue-battery (when (cpl-impl:value *blue-wasp-battery*)
                         (roslisp:make-message "sherpa_msgs/LoggedBattery"
                                               :robot_name "blue_wasp"
                                               :battery_level (second (cpl-impl:value *blue-wasp-battery*))
                                               :battery_drain (third (cpl-impl:value *blue-wasp-battery*)))))
         (batteries (list red-battery blue-battery))
         (batteries (remove nil batteries)))
    (roslisp:make-message "sherpa_msgs/LoggedBatteryList"
                          :stamp (roslisp:ros-time)
                          :batteries (coerce batteries 'vector))))

(defun resend-battery ()
  (loop while (cpl-impl:value *monitoring-battery*) do
    (roslisp:wait-duration 1)
    (roslisp:publish *battery-monitoring-publisher* (get-battery-monitor-message))))

(defun setup-battery-monitors ()
  (roslisp:subscribe "red_wasp/battery" "sherpa_msgs/Battery"
                     (lambda (msg) (battery-callback "red_wasp" *red-wasp-battery* msg)))
  (roslisp:subscribe "blue_wasp/battery" "sherpa_msgs/Battery"
                     (lambda (msg) (battery-callback "blue_wasp" *blue-wasp-battery* msg)))
  (setf (cpl-impl:value *monitoring-battery*) T)
  (setf *battery-monitoring-publisher*
        (roslisp:advertise "logged_battery" "sherpa_msgs/LoggedBatteryList"))
  (setf *battery-publisher-thread* (sb-thread:make-thread #'resend-battery)))

(defun shutdown-battery-monitors ()
  (setf (cpl-impl:value *red-wasp-battery*) nil)
  (setf (cpl-impl:value *blue-wasp-battery*) nil)
  (setf (cpl-impl:value *monitoring-battery*) nil)
  (setf *battery-monitoring-publisher* nil)
  ;; May also let the thread return naturally once monitoring-battery is set to nil
  (when (and *battery-publisher-thread* (sb-thread:thread-alive-p *battery-publisher-thread*))
    (sb-thread:terminate-thread *battery-publisher-thread*)))

(roslisp-utilities:register-ros-init-function setup-battery-monitors)
(roslisp-utilities:register-ros-cleanup-function shutdown-battery-monitors)

