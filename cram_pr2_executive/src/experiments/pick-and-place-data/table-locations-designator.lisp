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

(defparameter *table-locations*
  (list
   ;; Front row
   (cons 'front-1 (tf:make-pose-stamped
                   "/map" 0.0
                   (cl-transforms:make-3d-vector -1.7 2.0 0.87)
                   (cl-transforms:make-identity-rotation)))
   (cons 'front-2 (tf:make-pose-stamped
                   "/map" 0.0
                   (cl-transforms:make-3d-vector -1.75 2.2 0.87)
                   (cl-transforms:make-identity-rotation)))
   (cons 'front-3 (tf:make-pose-stamped
                   "/map" 0.0
                   (cl-transforms:make-3d-vector -1.75 2.4 0.87)
                   (cl-transforms:make-identity-rotation)))
   ;; Back row
   (cons 'back-1 (tf:make-pose-stamped
                  "/map" 0.0
                  (cl-transforms:make-3d-vector -1.9 2.0 0.87)
                  (cl-transforms:make-identity-rotation)))
   (cons 'back-2 (tf:make-pose-stamped
                  "/map" 0.0
                  (cl-transforms:make-3d-vector -1.95 2.2 0.87)
                  (cl-transforms:make-identity-rotation)))
   (cons 'back-3 (tf:make-pose-stamped
                  "/map" 0.0
                  (cl-transforms:make-3d-vector -1.95 2.4 0.87)
                  (cl-transforms:make-identity-rotation)))))

;; (defparameter *table-locations*
;;   (list
;;    ;; Front row
;;    (cons 'front-1 (tf:make-pose-stamped
;;                    "/map" 0.0
;;                    (cl-transforms:make-3d-vector -1.7 2.0 1.1)
;;                    (cl-transforms:make-identity-rotation)))
;;    (cons 'front-2 (tf:make-pose-stamped
;;                    "/map" 0.0
;;                    (cl-transforms:make-3d-vector -1.75 2.2 1.1)
;;                    (cl-transforms:make-identity-rotation)))
;;    (cons 'front-3 (tf:make-pose-stamped
;;                    "/map" 0.0
;;                    (cl-transforms:make-3d-vector -1.75 2.4 1.1)
;;                    (cl-transforms:make-identity-rotation)))
;;    ;; Back row
;;    (cons 'back-1 (tf:make-pose-stamped
;;                   "/map" 0.0
;;                   (cl-transforms:make-3d-vector -1.9 2.0 1.1)
;;                   (cl-transforms:make-identity-rotation)))
;;    (cons 'back-2 (tf:make-pose-stamped
;;                   "/map" 0.0
;;                   (cl-transforms:make-3d-vector -1.95 2.2 1.1)
;;                   (cl-transforms:make-identity-rotation)))
;;    (cons 'back-3 (tf:make-pose-stamped
;;                   "/map" 0.0
;;                   (cl-transforms:make-3d-vector -1.95 2.4 1.1)
;;                   (cl-transforms:make-identity-rotation)))))

(defun named-pose-generator (desig)
  (let* ((name (desig-prop-value desig 'name))
         (pose (cdr (assoc name *table-locations*))))
    (if pose
        (list pose)
        (alexandria:shuffle (mapcar #'cdr *table-locations*)))))

(defun named-pose-validator (desig generated-pose)
  (let* ((name (desig-prop-value desig 'name))
         (pose (cdr (assoc name *table-locations*))))
    (or (when (and name pose)
          (< (cl-transforms:v-dist
              (cl-transforms:origin pose)
              (cl-transforms:origin generated-pose))
             0.01))
        t)))

(register-location-generator 10 named-pose-generator)
(register-location-validation-function 10 named-pose-validator)
