;;; Copyright (c) 2012, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :spatial-relations-costmap)

(register-location-validation-function
 1 spatial-relations-costmap-pose-validator
 "The potential field cost-function that we use for spatial relations is too wide - 180
degrees. Theoretically it is correct, but for practical use we need to make the angle
narrower. The best solutions are near smaller angles, so we put a threshold on the
solution values.")

(defun spatial-relations-costmap-pose-validator (desig pose)
  "If desig-props:for is specified it means we generated a costmap for a specific object.
In that case we need a costmap with specific spread angle."
  (when (typep pose 'cl-transforms:pose)
    (let ((for-prop-value (desig-prop-value desig 'for)))
      (if for-prop-value
          (let* ((cm (location-costmap::get-cached-costmap desig))
                 (p (cl-transforms:origin pose)))
            (if cm
                (let ((costmap-value (/ (get-map-value
                                         cm
                                         (cl-transforms:x p)
                                         (cl-transforms:y p))
                                        (location-costmap::get-cached-costmap-maxvalue cm))))
                  (with-vars-bound (?threshold)
                      (lazy-car
                       (prolog `(object-costmap-threshold ?_ ,for-prop-value ?threshold)))
                    ;; (format t "validate ~a: costmap-val = ~a~% thresold = ~a~%"
                    ;;         for-prop-value costmap-value ?threshold)
                    (if (is-var ?threshold)
                        t
                        (> costmap-value ?threshold))
                    ))
                t))
          t))))
  