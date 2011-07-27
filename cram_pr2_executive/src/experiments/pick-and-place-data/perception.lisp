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

(in-package :pr2-ex)

(defun filter-objects-on-table (desigs bounding-pt-1 bounding-pt-2
                                &optional (area-padding 0.2))
  "Removes all objects that are not inside our area of valid
  locations. The area is defined by the points `bounding-pt-1' and
  `bounding-pt-2' and `area-padding'. The result is the list of all
  designators in `desig' whose location is inside the aabb defined by
  `bounding-pt-1' and `bounding-pt-2' inflated by `area-padding'."
  (flet ((area-check (point)
           (let ((min-x (- (min (cl-transforms:x bounding-pt-1) (cl-transforms:x bounding-pt-2))
                           area-padding))
                 (min-y (- (min (cl-transforms:y bounding-pt-1) (cl-transforms:y bounding-pt-2))
                           area-padding))
                 (max-x (+ (max (cl-transforms:x bounding-pt-1) (cl-transforms:x bounding-pt-2))
                           area-padding))
                 (max-y (+ (max (cl-transforms:y bounding-pt-1) (cl-transforms:y bounding-pt-2))
                           area-padding)))
             (and (> (cl-transforms:x point) min-x)
                  (> (cl-transforms:y point) min-y)
                  (< (cl-transforms:x point) max-x)
                  (< (cl-transforms:y point) max-y)))))
    (remove-if-not (alexandria:compose #'area-check #'cl-transforms:origin #'designator-pose)
                   desigs)))
