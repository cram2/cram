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

(in-package :btr-desig)

(defun make-object-bounding-box-costmap-generator (object)
  (let* ((bounding-box (aabb object))
         (dimensions-x/2 (/ (cl-transforms:x (bullet:bounding-box-dimensions bounding-box))
                            2))
         (dimensions-y/2 (/ (cl-transforms:y (bullet:bounding-box-dimensions bounding-box))
                            2)))
    (lambda (x y)
      (if (and
           (< x (+ (cl-transforms:x (cl-bullet:bounding-box-center bounding-box))
                   dimensions-x/2))
           (> x (- (cl-transforms:x (cl-bullet:bounding-box-center bounding-box))
                   dimensions-x/2))
           (< y (+ (cl-transforms:y (cl-bullet:bounding-box-center bounding-box))
                   dimensions-y/2))
           (> y (- (cl-transforms:y (cl-bullet:bounding-box-center bounding-box))
                   dimensions-y/2)))
          1.0 0.0))))

(defun make-object-bounding-box-height-generator (object)
  (let ((bounding-box (aabb object)))
    (constantly (list
                 (+ (cl-transforms:z
                     (cl-bullet:bounding-box-center bounding-box))
                    (/ (cl-transforms:z
                        (cl-bullet:bounding-box-dimensions bounding-box))
                       2))))))
