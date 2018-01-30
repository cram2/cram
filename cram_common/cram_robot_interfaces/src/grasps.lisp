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

(in-package :cram-robot-interfaces)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;; DEPRECATION WARNING ;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; PRETTY MUCH EVERYTHING IN HERE IS DERPECATED.   ;;;
;;; USE THE cram_object_interfaces PACKAGE INSTEAD. ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *grasps* nil
  "An alist that maps grasps to orientations in the robot's root
  link. For internal use only.")

(defvar *tool* nil
  "A cons containing CL-TRANSFORMS:3D-VECTOR indicating the direction
  of the tool and the default length of the tool. The vector must have
  a length of 1.0.")

(defmacro def-grasp (name orientation &rest aliases)
  "Defines a new grasp named `name'. The grasp's orientation is
specified by `orientation'. `aliases' is a sequence of alternative
names. `name' must not be in use by a different grasp while `aliases'
can overlap. GET-GRASP then allows for matching aliases. E.g. if we
have two side grasps, :LEFT and :RIGHT, the grasp can be named :LEFT
and :RIGHT while the `aliases' parameters can be set to :SIDE."
  `(eval-when (:load-toplevel)
     (when (assoc ,name *grasps*)
       (cut:style-warn "Redefining grasp ~s." ,name))
     (setf *grasps* (cons (cons ,name (cons ,orientation ',aliases))
                          (remove ,name *grasps* :key #'car)))))

(defmacro def-tool (vector default-length)
  `(eval-when (:load-toplevel)
     (when *tool*
       (cut:style-warn "Redefining tool."))
     (setf *tool* (cons ,vector ,default-length))))

(defun get-grasp (grasp)
  "Returns the grasp named `grasp'."
  (car (cdr (assoc grasp *grasps*))))

(defun get-grasp-names ()
  (mapcar #'car *grasps*))

(defun get-grasps (grasp-name &optional (filter (constantly t)))
  "Returns the list of grasps that are either named `grasp' or have
the alias `grasp' and for which `filter' returns T."
  (mapcar #'cadr
          (remove-if-not
           (lambda (grasp)
             (destructuring-bind (name orientation &rest aliases)
                 grasp
               (declare (ignore orientation))
               (when (or (eq grasp-name name)
                         (member grasp-name aliases))
                 (funcall filter name))))
           *grasps*)))

(defun get-tool-direction-vector ()
  (car *tool*))

(defun get-tool-length ()
  (cdr *tool*))

(defun get-tool-vector ()
  (cl-transforms:v* (car *tool*) (cdr *tool*)))

(defun calculate-bounding-box-tool-length (bounding-box
                                           &key (minimal-tool-length (get-tool-length)))
  "Calculates the tool length for an object with bounding box
`bounding-box' by taking the maximum of `minimal-tool-length', the
bounding box x minus the minimal tool length and the bounding box y
minus the minimal tool length. `bounding-box' needs to be a
CL-TRANSFORMS:3D-VECTOR."
  (declare (type cl-transforms:3d-vector bounding-box)
           (type number minimal-tool-length))
  (max minimal-tool-length
       (- (/ (cl-transforms:x bounding-box) 2) minimal-tool-length)
       (- (/ (cl-transforms:y bounding-box) 2) minimal-tool-length)))

(defun calculate-tool (tool-length grasp-orientation)
  "Calculates the tool pose given a `tool-length' and a
`grasp-orientation'."
  (cl-transforms:make-pose
   (cl-transforms:v* (car *tool*) tool-length)
   grasp-orientation))

(def-fact-group grasps (grasp
                        side
                        object-type-grasp
                        object-designator-grasp)
  ;; Defines types of grasps for a particular ?robot
  (<- (grasp ?robot ?grasp-type)
    (fail))

  ;; Defines from which ?side-s ?robot can grasp
  (<- (side ?robot ?side)
    (fail))

  ;; Unifies grasps with object types and required arms to manipulate
  ;; that object.
  (<- (object-type-grasp ?object-type ?grasp ?arms)
    (fail))

  ;; Given an object-designator, binds grasps and arms to manipulate
  ;; that object.
  (<- (object-designator-grasp ?object-designator ?grasp ?arms)
    (fail)))
