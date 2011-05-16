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

(defvar *ros-init-functions* (make-hash-table :test 'eq))
(defvar *ros-cleanup-functions* (make-hash-table :test 'eq))

(defmacro register-ros-init-function (name)
  `(setf (gethash ',name *ros-init-functions*)
         (symbol-function ',name)))

(defmacro register-ros-cleanup-function (name)
  `(setf (gethash ',name *ros-cleanup-functions*)
         (symbol-function ',name)))

(defun startup-ros (&key
                    (master-uri (make-uri "localhost" 11311) master-uri?)
                    (name "cram_hl")
                    (anonymous t))
  (if master-uri?
      (start-ros-node name :anonymous anonymous :master-uri master-uri)
      (start-ros-node name :anonymous anonymous))
  (loop for f being the hash-values of *ros-init-functions*
        do (progn
             (ros-info (rosnode) "ROS init ~a." f)
             (funcall f))))

(defun shutdown-ros ()
  (loop for f being the hash-values of *ros-cleanup-functions*
        do (progn
             (ros-info (rosnode) "ROS cleanup ~a." f)
             (funcall f)))
  (shutdown-ros-node))
