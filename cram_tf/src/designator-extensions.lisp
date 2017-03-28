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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :cram-tf)

(defparameter *distance-equality-threshold* 0.025)
(defparameter *angle-equality-threshold* (* 5 (/ pi 180)))

;;; We need to place these methods here because in the designator
;;; package, we don't have a notion of poses, just more or less
;;; abstract interfaces

(defmethod designator-pose ((desig location-designator))
  (reference desig))

(defmethod ensure-pose-stamped ((desig location-designator) &optional frame-id stamp)
  (declare (ignore frame-id stamp))
  (reference desig))

(defmethod designator-distance ((desig-1 location-designator) (desig-2 location-designator))
  (cl-transforms:v-dist
   (cl-transforms:origin (reference desig-1))
   (cl-transforms:origin (reference desig-2))))

(defmethod designator-solutions-equal
    ((solution-1 cl-transforms:pose) (solution-2 cl-transforms:pose))
  "Checks whether two designator solutions are equal in *fixed-frame* or *robot-base-frame*."
  (labels ((poses-equal-p (pose-1 pose-2)
             ;; predicate to check equality of two cl-trasforms:pose-s
             (and (< (v-dist (origin pose-1) (origin pose-2))
                     *distance-equality-threshold*)
                  (< (angle-between-quaternions (orientation pose-1) (orientation pose-2))
                     *angle-equality-threshold*)))
           (poses-equal-in-frame-p (pose-1 pose-2 compare-frame
                                    &key (timeout *tf-default-timeout*))
             ;; Predicate to check equality of two pose-stamped-s w.r.t. a given frame."
             (handler-case
                 (let ((pose-1-transformed
                         (transform-pose-stamped
                          *transformer*
                          :pose pose-1 :target-frame compare-frame :timeout timeout))
                       (pose-2-transformed
                         (transform-pose-stamped
                          *transformer*
                          :pose pose-2 :target-frame compare-frame :timeout timeout)))
                   (poses-equal-p pose-1-transformed pose-2-transformed))
               (transform-stamped-error () nil))))
    ;; actual check: first making sure to have pose-stamped
    (assert *fixed-frame* () "cram-tf:*fixed-frame* must be set.")
    (let ((pose-1 (ensure-pose-stamped solution-1 *fixed-frame* 0.0))
          (pose-2 (ensure-pose-stamped solution-2 *fixed-frame* 0.0)))
      (if (string-equal (unslash-frame (frame-id pose-1)) (unslash-frame (frame-id pose-2)))
          (poses-equal-p pose-1 pose-2)
          ;; equality in either of the defined frames is sufficient for us
          (or (poses-equal-in-frame-p pose-1 pose-2 *fixed-frame*)
              (poses-equal-in-frame-p pose-1 pose-2 *robot-base-frame*))))))

(defmethod reference :around ((designator location-designator) &optional role)
  "Converts all cl-transforms poses into cl-tf poses in the fixed coordinate system"
  (declare (ignore role))
  (let ((solution (call-next-method)))
    (if (typep solution 'cl-transforms:pose)
        (if *fixed-frame*
            (ensure-pose-stamped solution *fixed-frame* 0.0)
            (error "cram-tf:*fixed-frame* must be set."))
        solution)))

(def-fact-group desig-tf-solutions (location-grounding)
  (<- (location-grounding ?desig ?solution)
    (spec:property ?desig (:pose ?solution))))

;; (defgeneric ground (designator)
;;   (:documentation "Returns a designator with properties augmented with subsymbolic data"))

;; (defmethod ground ((designator location-desginator))
;;   "Calls reference and appends that to the pose"
;;   )
