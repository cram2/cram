;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :pr2-reachability-costmap)

(defparameter *minimum* (cl-transforms:make-3d-vector -1.0 -1.5 -0.75))
(defparameter *maximum* (cl-transforms:make-3d-vector 1.0 1.5 0.5))
(defparameter *steps* (cl-transforms:make-3d-vector 0.05 0.05 0.05))
(defparameter *angles* (append
                        (loop for i from (- pi) below pi by (/ pi 4)
                              collecting (cl-transforms:q*
                                          (cl-transforms:euler->quaternion :az i)
                                          (cl-transforms:euler->quaternion :ay (/ pi 2))))
                        (loop for i from (- pi) below pi by (/ pi 4)
                              collecting (cl-transforms:euler->quaternion :az i))))

(defun generate-map-main ()
  (let ((side (cond ((equal (second sb-ext:*posix-argv*) "left")
                     :left)
                    ((equal (second sb-ext:*posix-argv*) "right")
                     :right)))
        (filename (third sb-ext:*posix-argv*)))
    (unless (and side filename)
      (error
       'simple-error
       :format-control "Invalid command line parameters: Usage ~a <left | right> <filename>"
       :format-arguments (list (first sb-ext:*posix-argv*))))
    (roslisp:with-ros-node ("generate_reachability_map" :anonymous t)
      (store-reachability-map
       (make-instance 'reachability-map
         :side side
         :minimum *minimum*
         :maximum *maximum*
         :resolution *steps*
         :orientations *angles*)
       filename))))
