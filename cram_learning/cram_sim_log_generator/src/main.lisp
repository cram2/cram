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
(defparameter *mongo-logger* nil)
(defparameter num-experiments 1)
(defparameter connection-retries 0)
(defparameter *start-time* 0)
(defparameter *global-timer* 0)
(defparameter *main-result* '())

(defun prepare-logging ()
  (setf connection-retries 0)
  (print "Cleaning old ros nodes ...")
  ;;(asdf-utils:run-program (concatenate 'string "echo 'y' | rosnode cleanup"))
  (print "Cleaning old ros nodes done")
  (print "Waiting for JSON Prolog to start ...")
  ;;(asdf-utils:run-program (concatenate 'string "python ~/Desktop/lol_launcher.py"))
  (print "JSON Prolog started")
  (setf ccl::*is-client-connected* nil)
  (setf ccl::*is-logging-enabled* t)
  (setf ccl::*host* "'https://localhost'")
  (setf ccl::*cert-path* "'/home/koralewski/Desktop/localhost.pem'")
  ;;(setf ccl::*api-key* "'K103jdr40Rp8UX4egmRf42VbdB1b5PW7qYOOVvTDAoiNG6lcQoaDHONf5KaFcefs'")
  (setf ccl::*api-key* "'uWZv7X1cQkdlZkx4R4uYSIQGyCr4zgyrYEgorw7XjICK0JyLQrmitf48hMCC4pYg'")
  (loop while (and (not ccl::*is-client-connected*) (< connection-retries 10)) do
    (progn
      (ccl::connect-to-cloud-logger)
      (setf connection-retries (+ connection-retries 1))))
  (when (not ccl::*is-client-connected*)
    (/ 1 0)))

(defun call-reset-world-service (&optional (id "0"))
  (roslisp:ros-info (logging-demo reset-world) "Resetting Unreal world.")
  (if (not (roslisp:wait-for-service "/UnrealSim/reset_level" 10))
      (roslisp:ros-warn (send-reset-world-client) "timed out waiting for send-reset-world service")
      (roslisp:call-service "/UnrealSim/reset_level" 'world_control_msgs-srv:ResetLevel :id id))
  (sleep 2))

;; Use like this:
;; (main)
;; (main :objects '(:cup :bowl))
;; (main :logging-enabled T)
;; (main :objects '(:milk :spoon) :logging-enabled NIL)
(defun main (&key (objects '(;; :milk
                             ;; :breakfast-cereal
                             ;; :spoon
                             ;; :bowl
                             :cup
                             ))
               (logging-enabled NIL))
  (unless (eq (roslisp:node-status) :RUNNING)
    (roslisp-utilities:startup-ros :name "cram" :anonymous nil))
  (when logging-enabled (prepare-logging))
  ;; (setf cram-bullet-reasoning-belief-state:*spawn-debug-window* nil)
  ;; (setf cram-tf:*tf-broadcasting-enabled* t)
  ;; (setf cram-urdf-projection-reasoning::*projection-checks-enabled* nil)
  ;; (setq roslisp::*debug-stream* nil)
  ;; (pr2-pms:with-real-robot (demo::park-robot))
  
  ;; Demo loop
  (dotimes (n 100)
    (setf ccl::*retry-numbers* 0)
    (call-reset-world-service)
    (when logging-enabled (ccl:start-episode))
    
    (cpl:with-failure-handling
        ((common-fail:high-level-failure (e)
           (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
           (return)))
      (pr2-pms:with-real-robot (demo::setting-demo objects)))

    (push ccl::*retry-numbers* *main-result*)
    (when logging-enabled (ccl:stop-episode))
    (sleep 2))
  
  (print *main-result*)
  (when logging-enabled (ccl:finish-logging)))
