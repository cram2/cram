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

(in-package :cram-manipulation-knowledge)

(defvar *grasps* nil
  "An alist that maps grasps to orientations in the robot's root
  link. For internal use only.")

(defvar *tool* nil
  "A cons containing CL-TRANSFORMS:3D-VECTOR indicating the direction
  of the tool and the default length of the tool. The vector must have
  a length of 1.0.")

(defmacro def-grasp (name orientation)
  `(eval-when (:load-toplevel)
     (when (assoc ,name *grasps*)
       (style-warn "Redefining grasp ~s." ,name))
     (setf *grasps* (cons (cons ,name ,orientation)
                          (remove ,name *grasps* :key #'car)))))

(defmacro def-tool (vector default-length)
  `(eval-when (:load-toplevel)
     (when *tool*
       (style-warn "Redefining tool."))
     (setf *tool* (cons ,vector ,default-length))))

(defun get-grasp (grasp side)
  (ecase grasp
    (:top (cdr (assoc :top *grasps*)))
    (:front (cdr (assoc :front *grasps*)))
    (:side (cdr (assoc side *grasps*)))))

(defun get-tool (tool-length grasp-orientation)
  (cl-transforms:make-pose
   (cl-transforms:v* (car *tool*) tool-length)
   grasp-orientation))
