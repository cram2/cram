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

(in-package :semantic-map-costmap)

(defvar *semantic-map-cache* (make-hash-table :test #'equal))

(defun caching-generator-function (name generator-function)
  "Returns a generator function that uses the cache `name'. If the
  cache hasn't been initialized for `name' before, adds a new entry
  using `generator-function'. Generator functions are functions that
  take two parameters, X and Y and return a costmap value."
  (or (gethash name *semantic-map-cache*)
      (init-generator-cache name generator-function)))

(defun init-generator-cache (name generator-function)
  (destructuring-bind (&key width height resolution origin-x origin-y)
      (costmap-metadata)
    (let ((cache (make-array (list (truncate (/ height resolution))
                                   (truncate (/ width resolution)))
                             :element-type 'single-float)))
      (declare (type simple-array cache))
      (dotimes (y (truncate (/ height resolution)))
        (dotimes (x (truncate (/ width resolution)))
          (setf (aref cache y x)
                (or
                 (funcall generator-function
                          (+ (* x resolution) origin-x)
                          (+ (* y resolution) origin-y))
                 0.0))))
      (setf (gethash name *semantic-map-cache*)
            (lambda (x y)
              (let ((value
                      (aref cache
                            (truncate (/ (- y origin-y) resolution))
                            (truncate (/ (- x origin-x) resolution)))))
                (unless (eql value 0.0)
                  value)))))))
