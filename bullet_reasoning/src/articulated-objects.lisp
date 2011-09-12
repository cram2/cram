;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :btr)

;; Until we get the articulation models from knowrob, we fake them for
;; now and use the kitchen urdf instead.

(alexandria:define-constant +drawer-max-value+ 0.5
  :documentation "Hard-coded ugly constant for maximal joint values
  when drawer is out. This will be replaced by an articulation model")

(alexandria:define-constant +door-max-value+ (* pi 0.7)
  :documentation "Hard-coded ugly constant for maximal joint values
  when door is open. This will be replaced by an articulation model")

(defun find-urdf-obj (obj link)
  (declare (type semantic-map-object obj)
           (type string link))
  (let ((urdf (urdf obj)))
    (or (sem-map-utils:urdf-obj-name link)
        (let ((link (gethash link (cl-urdf:links urdf))))
          (when (and link (cl-urdf:parent (cl-urdf:from-joint link)))
            (find-urdf-obj obj (cl-urdf:name (cl-urdf:parent
                                              (cl-urdf:from-joint link)))))))))

(defun articulation-joint-objs (obj link)
  "Returns the name of the joint that can be used to move `link' which
  has to be part of `obj'"
  (declare (type semantic-map-object obj))
  (let* ((part-name (find-urdf-obj obj link))
         (part (when part-name
                 (lazy-car (sem-map-utils:sub-parts-with-name obj part-name)))))
    (assert part-name () "Link `~a' could not be mapped to a semantic map instance"
            link)
    (assert part () "Could not find semantic map object named `~a'" part-name)
    (force-ll (sem-map-utils:sub-parts-with-type part "Joint-Physical"))))

(defun open-object (obj link)
  (when (typep obj 'semantic-map-object)
    (let* ((joints (articulation-joint-objs obj link))
           (joint (car joints)))
      (assert (eql (length joints) 1) ()
              "Opening of objects with more than one joint not supported")
      (string-case (sem-map-utils:obj-type joint)
        ("PrismaticJoint" (setf (joint-state obj (sem-map-utils:urdf-name joint))
                                +drawer-max-value+))
        ("HingedJoint" (setf (joint-state obj (sem-map-utils:urdf-name joint))
                             +door-max-value+))))))

(defun close-object (obj link)
  (when (typep obj 'semantic-map-object)
    (let* ((joints (articulation-joint-objs obj link))
           (joint (car joints)))
      (assert (eql (length joints) 1) ()
              "Opening of objects with more than one joint not supported")
      (setf (joint-state obj (sem-map-utils:urdf-name joint))
            0.0))))
