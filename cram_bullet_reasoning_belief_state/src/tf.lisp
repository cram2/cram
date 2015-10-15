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

(in-package :cram-bullet-reasoning-belief-state)

(setf cram-transforms-stamped:*fixed-frame* "map")
(setf cram-transforms-stamped:*odom-frame* "odom_combined")

(defun ros-tf-init ()
  (setf cram-transforms-stamped:*transformer* (make-instance 'cl-tf2:buffer-client))

  (with-vars-bound (?base-frame ?torso-frame ?torso-joint)
      (lazy-car (prolog `(and (robot ?robot)
                              (robot-base-frame ?robot ?base-frame)
                              (robot-torso-link-joint ?robot ?torso-frame ?torso-joint))))
    (unless (is-var ?base-frame)
      (setf cram-transforms-stamped:*robot-base-frame* ?base-frame))
    (unless (is-var ?torso-frame)
      (setf cram-transforms-stamped:*robot-torso-frame* ?torso-frame))
    (unless (is-var ?torso-joint)
      (setf cram-transforms-stamped:*robot-torso-joint* ?torso-joint))))

(roslisp-utilities:register-ros-init-function ros-tf-init)
