;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-tf)

(defvar *transformer* nil
  "A TF transformer object: be it TF:TRANSFORM-LISTENER or TF2:BUFFER-CLIENT etc.")

(defparameter *tf-default-timeout* 4.0
  "How long to wait until a tansform in secs. For simulation the default is 10.0")


(defvar *broadcaster* nil "A tf-broadcaster instance")

(defparameter *tf-broadcasting-enabled* nil)
(defparameter *tf-broadcasting-topic* "tf")
(defparameter *tf-broadcasting-interval* 0.1)


(defvar *fixed-frame* nil
  "World coordinate system name. Initialized in a ROS init function.")

(defvar *odom-frame* nil
  "Robot odometry frame name.")

(defvar *robot-base-frame* nil
  "Robot's TF tree root frame. Be careful, this parameter and the ones below
are only initialized when a ROS node is started. DO NOT USE THESE IN OTHER PARAMETERS,
or in general at compile-time.")

(defvar *robot-torso-frame* nil)

(defvar *robot-torso-joint* nil)

(defvar *robot-left-tool-frame* nil
  "Tool frame of the left arm. Initialized from CRAM robot description.")
(defvar *robot-right-tool-frame* nil
  "Tool frame of the right arm. Initialized from CRAM robot desciption.")


(defun init-tf ()
  (macrolet ((initialize-var (dynamic-var prolog-var)
               (let ((var-name (symbol-name dynamic-var)))
                 `(if (is-var ,prolog-var)
                      (roslisp:ros-info (cram-tf init-tf)
                                        "~a is unknown. Did you load a robot description package?"
                                        ,var-name)
                      (progn
                        (setf ,dynamic-var ,prolog-var)
                        (roslisp:ros-info (cram-tf init-tf)
                                          "Set ~a to ~s."
                                          ,var-name ,prolog-var))))))

    (setf *fixed-frame* "map")
    (roslisp:ros-info (cram-tf init-tf) "Set *fixed-frame* to ~s." *fixed-frame*)

    (setf *transformer* (make-instance 'cl-tf:transform-listener))
    (roslisp:ros-info (cram-tf init-tf)
                      "Initialized *transformer* to a ~a." (type-of *transformer*))

    (when (roslisp:get-param "use_sim_time" nil)
      (setf *tf-default-timeout* 10.0))
    (roslisp:ros-info (cram-tf init-tf)
                      "*tf-default-timeout* is ~a." *tf-default-timeout*)

    (with-vars-bound (?odom-frame)
        (lazy-car (prolog `(and (robot ?robot)
                                (robot-odom-frame ?robot ?odom-frame))))
      (initialize-var *odom-frame* ?odom-frame))

    (with-vars-bound (?base-frame)
        (lazy-car (prolog `(and (robot ?robot)
                                (robot-base-frame ?robot ?base-frame))))
      (initialize-var *robot-base-frame* ?base-frame))

    (with-vars-bound (?torso-frame ?torso-joint)
        (lazy-car (prolog `(and (robot ?robot)
                                (robot-torso-link-joint ?robot ?torso-frame ?torso-joint))))
      (initialize-var *robot-torso-frame* ?torso-frame)
      (initialize-var *robot-torso-joint* ?torso-joint))

    (with-vars-bound (?left-tool-frame ?right-tool-frame)
        (lazy-car (prolog `(and (robot ?robot)
                                (robot-tool-frame ?robot :left ?left-tool-frame)
                                (robot-tool-frame ?robot :right ?right-tool-frame))))
      (initialize-var *robot-left-tool-frame* ?left-tool-frame)
      (initialize-var *robot-right-tool-frame* ?right-tool-frame))))

(defun destroy-tf ()
  (setf *transformer* nil)
  (setf *tf-default-timeout* 4.0))

(roslisp-utilities:register-ros-init-function init-tf)
(roslisp-utilities:register-ros-cleanup-function destroy-tf)


(defun init-tf-broadcaster ()
  (setf *broadcaster*
          (make-tf-broadcaster
           *tf-broadcasting-topic*
           *tf-broadcasting-interval*))
  (when *tf-broadcasting-enabled*
    (start-publishing-transforms *broadcaster*)))

(defun destroy-tf-broadcaster ()
  (when *broadcaster*
    (stop-publishing-transforms *broadcaster*)
    (with-slots (publisher tf-topic) *broadcaster*
      (roslisp:unadvertise tf-topic)
      (setf publisher nil))
    (setf *broadcaster* nil)))

(roslisp-utilities:register-ros-init-function init-tf-broadcaster)
(roslisp-utilities:register-ros-cleanup-function destroy-tf-broadcaster)
