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

(in-package :btr)

(defun simulate (world secs &optional (dt 0.01) realtime)
  (multiple-value-bind (steps rest) (truncate secs dt)
    (when (> rest 0.0)
      (incf steps))
    (dotimes (i steps)
      (let ((curr-time (/ (get-internal-real-time)
                          internal-time-units-per-second)))
        (step-simulation world dt)
        (when realtime
          (let ((dt-real (- dt (- (/ (get-internal-real-time)
                                  internal-time-units-per-second)
                               curr-time))))
            (when (> dt-real 0.005)
              (sleep dt-real))))))))

(defun find-objects (world &optional (pred (constantly t)))
  "Finds all objects that match the predicate"
  (remove-if-not
   pred (objects world)))

(defun contact-p (world obj-1 obj-2)
  "Returns T if obj-1 and obj-2 are in contact"
  (find-if (lambda (contact)
             (when (and
                    (> (array-dimension (contact-points contact) 0)
                       0)
                    (or (and (rigid-body obj-1 (name (body-1 contact)))
                             (rigid-body obj-2 (name (body-2 contact))))
                        (and (rigid-body obj-2 (name (body-1 contact)))
                             (rigid-body obj-1 (name (body-2 contact))))))
               t))
           (contact-manifolds world)))

(defun find-all-contacts (world)
  (let ((objects (objects world)))
    (remove-duplicates
     (remove-if-not
      #'identity
      (mapcar (lambda (contact)
                (when (> (array-dimension (contact-points contact) 0)
                         0)
                  (list (find-if (lambda (obj)
                                   (rigid-body obj (name (body-1 contact))))
                                 objects)
                        (find-if (lambda (obj)
                                   (rigid-body obj (name (body-2 contact))))
                                 objects))))
              (contact-manifolds world)))
     :test #'equal)))

(defun find-objects-in-contact (world obj)
  (let ((objects (objects world)))
    (remove-duplicates
     (remove-if-not
      #'identity
      (mapcar (lambda (contact)
                (when (> (array-dimension (contact-points contact) 0)
                         0)
                  (cond ((rigid-body obj (name (body-1 contact)))
                         (find-if (lambda (obj)
                                    (rigid-body obj (name (body-2 contact))))
                                  objects))
                        ((rigid-body obj (name (body-2 contact)))
                         (find-if (lambda (obj)
                                    (rigid-body obj (name (body-1 contact))))
                                  objects)))))
              (contact-manifolds world))))))

(defun poses-equal-p (pose-1 pose-2 dist-sigma ang-sigma)
  (and (< (cl-transforms:v-dist (cl-transforms:origin pose-1)
                                (cl-transforms:origin pose-2))
          dist-sigma)
       (< (cl-transforms:angle-between-quaternions
           (cl-transforms:orientation pose-1)
           (cl-transforms:orientation pose-2))
          ang-sigma)))

(defun stable-p (obj)
  (when obj
    (not (every (lambda (body)
                  (eq (activation-state body)
                      :active-tag))
                (rigid-bodies obj)))))

(defun above-p (obj-1 obj-2)
  "Returns T if `obj-1' is above `obj-2'"
  (> (cl-transforms:z (cl-transforms:origin (pose obj-1)))
     (cl-transforms:z (cl-transforms:origin (pose obj-2)))))

(defun find-objects-above (world obj)
  "Returns a list of all objects thate are above `obj'"
  (find-objects world (lambda (o) (above-p o obj))))

(defun below-p (obj-1 obj-2)
  "Returns T if `obj-1' is below `obj-2'"
  (< (cl-transforms:z (cl-transforms:origin (pose obj-1)))
     (cl-transforms:z (cl-transforms:origin (pose obj-2)))))

(defun find-objects-below (world obj)
  "Returns a list of all objects thate are below `obj'"
  (find-objects world (lambda (o) (below-p o obj))))
