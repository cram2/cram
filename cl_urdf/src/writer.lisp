;;; Copyright (c) 2014, Jannik Buckelo <jannikbu@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :cl-urdf)

(defun write-3d-vector (vector)
  "Takes a 3D-vector and converts it to string of its three values 
seperated by a whitespcae."
  (flet ((to-single-float (num) (coerce num 'single-float))
         (write-triple (x y z) (format nil "~a ~a ~a" x y z)))
    (apply #'write-triple (mapcar #'to-single-float
                                  (list (cl-transforms:x vector) 
                                        (cl-transforms:y vector) 
                                        (cl-transforms:z vector))))))

(defun generate-urdf-xml (robot)
  "Creates a urdf descriptions in the xml-struct format of the given `robot'."
  (let ((links nil)
        (joints nil))
    (maphash (lambda (hash link) 
               (declare (ignore hash))
               (push (link->xml-element link) links))
             (links robot))
    (maphash (lambda (hash joint)
               (declare (ignore hash))
               (push (joint->xml-element joint) joints))
             (joints robot))
    (s-xml:make-xml-element :name ':|robot|
                            :attributes `((:|name| . ,(name robot)))
                            :children (concatenate 'list links joints))))

(defun generate-urdf-string (robot)
  "Creates a urdf descriptions as a string of the given `robot'."
  (s-xml:print-xml-string (generate-urdf-xml robot) :input-type :xml-struct))


;;; Functions for conversion from a robot part into a xml-element

(defun link->xml-element (link)
  (let ((children nil))
    (when (inertial link)
      (push (inertial->xml-element (inertial link))
            children))
    (when (visual link)
      (push (visual->xml-element (visual link))
            children))
    (when (collision link)
      (push (collision->xml-element (collision link))
            children))
    (s-xml:make-xml-element :name ':|link|
                            :attributes `((:|name| . ,(name link)))
                            :children children)))

(defun visual->xml-element (visual)
  (let ((children (list (geometry->xml-element (geometry visual))
                        (origin->xml-element (origin visual)))))
    (when (material visual)
      (let ((xml-material (material->xml-element (material visual))))
        (when xml-material
          (push xml-material children))))
    (s-xml:make-xml-element :name ':|visual|
                            :attributes nil
                            :children children)))

(defun geometry->xml-element (geometry)
  (let ((child (case (type-of geometry)
                 (box (s-xml:make-xml-element 
                        :name ':|box|
                        :attributes `((:|size| . ,(write-3d-vector (size geometry))))))
                 (cylinder (s-xml:make-xml-element 
                             :name ':|cylinder|
                             :attributes `((:|length| . ,(write-to-string (cylinder-length geometry)))
                                           (:|radius| . ,(write-to-string (radius geometry))))))
                 (sphere (s-xml:make-xml-element 
                          :name ':|sphere|
                          :attributes `((:|radius| . ,(write-to-string (radius geometry))))))
                 (mesh  (s-xml:make-xml-element 
                         :name ':|mesh|
                         :attributes (let ((attributes `((:|filename| . ,(filename geometry)))))
                                       (when (scale geometry)
                                         (push `(:|scale| . ,(write-3d-vector (scale geometry)))
                                               attributes))
                                       attributes))))))
    (s-xml:make-xml-element :name ':|geometry|
                            :attributes nil
                            :children (list child))))
                                         
(defun material->xml-element (material)
  (when (slot-boundp material 'name)
    (let ((children nil))
      (when (and (slot-boundp material 'texture) (texture material))
        (push (s-xml:make-xml-element :name ':|texture|
                                      :attributes `((:|filename| . ,(texture material))))
              children))
      (when (color material)
        (push (s-xml:make-xml-element :name ':|color|
                                      :attributes `((:|rgba| . ,(apply #'format nil "~a ~a ~a ~a" 
                                                                       (color material)))))
              children))    
      (s-xml:make-xml-element :name ':|material|
                              :attributes `((:|name| . ,(name material)))
                              :children children))))

(defun collision->xml-element (collision)
  (let ((children (list (geometry->xml-element (geometry collision))
                        (origin->xml-element (origin collision)))))
    (s-xml:make-xml-element :name ':|collision|
                            :attributes nil
                            :children children)))

(defun inertial->xml-element (inertial)
  (s-xml:make-xml-element :name ':|inertial|
                          :attributes nil
                          :children (list (origin->xml-element (origin inertial))
                                          (mass->xml-element (mass inertial))
                                          (inertia->xml-element nil))))

(defun inertia->xml-element (inertia)
  ;; inertia isn't implemented in the parser
  (declare (ignore inertia))
  (s-xml:make-xml-element :name ':|inertia|
                          :attributes `((:|ixx| . "100")
                                        (:|ixy| . "0")
                                        (:|ixz| . "0")
                                        (:|iyy| . "100")
                                        (:|iyz| . "0")
                                        (:|izz| . "100"))))

(defun mass->xml-element (mass)
  (s-xml:make-xml-element :name ':|mass|
                          :attributes `((:|value| . 
                                          ,(write-to-string(coerce mass 'single-float))))))

(defun joint->xml-element (joint)
  (let ((children (list (joint-link->xml-element (parent joint) t)
                        (joint-link->xml-element (child joint) nil)
                        (origin->xml-element (origin joint))
                        (axis->xml-element (axis joint)))))
    (when (and (slot-boundp joint 'limits)  (limits joint))
      (push (limits->xml-element (limits joint)) children))
    (s-xml:make-xml-element :name ':|joint|
                            :attributes `((:|name| . ,(name joint))
                                          (:|type| . ,(string-downcase (string (joint-type joint)))))
                            :children children)))

(defun joint-link->xml-element (link is-parent)
  (s-xml:make-xml-element :name (if is-parent :|parent| :|child|)
                          :attributes `((:|link| . ,(name link)))))

(defun origin->xml-element (origin)
  (let ((translation (cl-transforms:translation origin))
        (rotation (quaternion->rpy (cl-transforms:rotation origin))))
    (s-xml:make-xml-element :name ':|origin|
                            :attributes `((:|xyz| . ,(write-3d-vector translation))
                                          (:|rpy| . ,(write-3d-vector rotation))))))

(defun axis->xml-element (axis)
  (s-xml:make-xml-element :name ':|axis|
                          :attributes `((:|xyz| . ,(write-3d-vector axis)))))

(defun limits->xml-element (limits)
  (s-xml:make-xml-element :name ':|limit|
                          :attributes `((:|upper| . ,(write-to-string (upper limits)))
                                        (:|lower| . ,(write-to-string (lower limits)))
                                        (:|effort| . ,(write-to-string (effort limits)))
                                        (:|velocity| . ,(write-to-string (velocity limits))))))

(defun quaternion->rpy (q)
  (let ((qx (cl-transforms:x q))
        (qy (cl-transforms:y q))
        (qz (cl-transforms:z q))
        (qw (cl-transforms:w q)))
    (let ((p (atan (- (* 2 qy qw) (* 2 qx qz)) (- 1 (* 2 qy qy) (* 2 qz qz))))
          (r (atan (- (* 2 qx qw) (* 2 qy qz)) (- 1 (* 2 qx qx) (* 2 qz qz))))
          (y (asin (+ (* 2 qx qy) (* 2 qz qw)))))
      (cl-transforms:make-3d-vector r p y))))