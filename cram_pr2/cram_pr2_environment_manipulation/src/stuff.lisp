;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :pr2-em)

;;; SIMULATION MANIPULATION

(defun move-joint (name angle)
  (format T "Move joint ~a to angle ~a.~%" name angle)
  (btr:set-robot-state-from-joints
   `((,name ,angle))
   (btr:object btr:*current-bullet-world* :kitchen)))

(defun open-container-joint (container-type)
  (format T "Open container ~a.~%" container-type)
  (move-joint (cdr (assoc container-type *container-joints*))
              (cdr (assoc container-type *container-angles*))))

(defun close-container-joint (container-type)
  (format T "Close container ~a.~%" container-type)
  (move-joint (cdr (assoc container-type *container-joints*))
              0))

(defun open-all ()
  (dolist (c *container*) (open-container-joint c)))

(defun close-all ()
  (dolist (c *container*) (close-container-joint c)))

;;; TESTING

(defun get-opening-desig (&optional (?name 'sink_area_left_upper_drawer_main))
  (let ((?object (get-container-desig ?name)))
    (an action
        (type opening)
        (opening-distance 0.48)
        (object ?object))))

(defun get-container-desig (&optional (?name 'sink_area_left_upper_drawer_main))
  (let* ((name-str (roslisp-utilities:rosify-underscores-lisp-name ?name))
         (urdf-pose (get-urdf-link-pose name-str)))
    (let* ((?pose (cl-tf:transform-pose-stamped cram-tf:*transformer*
                                                :target-frame "base_footprint"
                                                :pose (cl-tf:pose->pose-stamped "map" 0 urdf-pose)))
           (?transform (cl-tf:make-transform-stamped "base_footprint" name-str
                                                     (cl-tf:stamp ?pose)
                                                     (cl-tf:origin ?pose)
                                                     (cl-tf:orientation ?pose))))
      (an object
          (type container)
          (name ?name)
          (part-of kitchen)
          (pose ((pose ?pose) (transform ?transform)))))))

(defun test-open ()
  (cram-pr2-projection:with-simulated-robot
    (let ((?desig (get-opening-desig 'sink_area_left_upper_drawer_main)))
      (exe:perform ?desig))))

(defun test ()
  (cram-pr2-projection:with-simulated-robot
    (let ((?object (get-container-desig 'sink_area_left_upper_drawer_main)))
      (exe:perform (an action (type driving-and-opening) (object ?object))))
    (sleep 1)
    (let ((?object (get-container-desig 'sink_area_left_upper_drawer_main)))
      (exe:perform (an action (type closing) (object ?object))))))

(defun move-pr2 (x y)
  (cram-pr2-projection:with-simulated-robot
        (let ((?goal (cl-transforms-stamped:make-pose-stamped
                    "map"
                    0.0
                    (cl-tf:make-3d-vector x y 0.0)
                    (cl-tf:make-identity-rotation))))
        (exe:perform (an action (type going) (target (a location (pose ?goal))))))))
