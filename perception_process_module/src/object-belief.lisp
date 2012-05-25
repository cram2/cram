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

(in-package :perception-pm)

(defgeneric make-new-desig-description (old-desig perceived-object)
  (:documentation "Merges the description of `old-desig' with the
  properties of `perceived-object'")
  (:method ((old-desig object-designator) (po object-designator-data))
    (let ((obj-loc-desig (make-designator 'location `((pose ,(object-pose po))))))
      (cons `(at ,obj-loc-desig)
            (remove 'at (description old-desig) :key #'car)))))

(defgeneric object-distance (obj-1 obj-2)
  (:documentation "Returns the distance between two objects."))

(defun compatible-properties (props-1 props-2)
  (or (null props-1)
      (let ((prop-1-value (cadar props-1))
            (prop-2-value (cadr (assoc (caar props-1) props-2))))
        (when (or (null prop-2-value)
                  (eq prop-1-value prop-2-value)
                  (or
                   (when (json-prolog:wait-for-prolog-service 0.5)
                     (json-prolog:prolog-1
                      `(cop-compatible-results ,prop-1-value ,prop-2-value)))
                   (prolog `(obj-subtype ,prop-1-value ,prop-2-value))
                   (prolog `(obj-subtype ,prop-2-value ,prop-1-value))))
          (compatible-properties (cdr props-1) props-2)))))

(defun merge-desig-descriptions (old-description new-description)
  "Merges the designator descriptions `old-description' and
   `new-description'. The result is a union of the two
   descriptions. For all properties that occur in both descriptions
   but have a different property value, the property of
   `new-description' is used. "
  (reduce (rcurry (flip #'adjoin) :key #'car)
          old-description :initial-value new-description))

(defmethod designator-pose ((desig object-designator))
  (object-pose (reference desig)))

(defmethod designator-distance ((desig-1 designator) (desig-2 designator))
  (object-distance (reference desig-1) (reference desig-2)))

(defmethod object-distance ((obj-1 t) (obj-2 t))
  (cl-transforms:v-dist
   (cl-transforms:origin (object-pose obj-1))
   (cl-transforms:origin (object-pose obj-1))))

(defmethod object-distance ((obj-1 t) (obj-2 cl-transforms:pose))
  (cl-transforms:v-dist
   (cl-transforms:origin (object-pose obj-1))
   (cl-transforms:origin obj-2)))

(defmethod object-distance ((obj-1 cl-transforms:pose) (obj-2 t))
  (cl-transforms:v-dist
   (cl-transforms:origin obj-1)
   (cl-transforms:origin (object-pose obj-2))))

(defun rete-assert-object-perception (desig perceived-object)
  (rete-assert `(object-perceived ,desig ,perceived-object)))
