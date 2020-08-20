;;;
;;; Copyright (c) 2020, Thomas Lipps <tlipps@uni-bremen.de>
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

(in-package :man-int-tests)

;; Run Tests in kitchen environment with PR2 or
;; in the retail environment with donbot
(defvar *run-pr2-pp-demo* T)
(defvar *run-donbot-retail-demo* NIL)

(defun vec-list-eqp (vec1 vec2 &optional (valid-diff 0.02))
  (let ((ret T))
    (loop for v in vec1
          for w in vec2 do
            (when (> (abs (- (abs v) (abs w)))
                     valid-diff)
              (setf ret nil)))
  ret))

(defun round-to (number &optional (precision 5) (what #'round))
        (let ((div (expt 5 precision)))
          (/ (funcall what (* number div)) div)))

(when *run-pr2-pp-demo*
  (defun init-tests-pr2-pp-setting-demo ()
    (roslisp-utilities:startup-ros)
    (demo::init-projection)
    (urdf-proj:with-simulated-robot
      (demo::spawn-objects-on-fixed-spots))))

(when *run-donbot-retail-demo*
  (defun init-tests-donbot-retail-demo ()
    (roslisp-utilities:startup-ros)
    (demo::init-projection)
    (urdf-proj:with-simulated-robot
      (demo::spawn-objects-on-small-shelf))))

(when *run-pr2-pp-demo*
  (define-test get-lift-transforms-in-base-household-milk-in-fridge-main
    (init-tests-pr2-pp-setting-demo)
    (sleep 2)
    (let ((calculated-lift-tfs
            (urdf-proj:with-simulated-robot
              (man-int::get-lift-transforms-in-base 
               :milk :milk-1 :right :back :fridge :iai-fridge-main)))
          (defined-lift-tfs
            (man-int:get-object-type-wrt-reference-frame-lift-transforms
             :milk :right :back :fridge)))
      (loop for calc-tf in calculated-lift-tfs
            for def-tf in defined-lift-tfs do
              (let ((calc-vec (cl-tf:translation calc-tf))
                    (def-vec (cl-tf:translation def-tf)))
                (destructuring-bind (c-x c-y c-z) 
                    (mapcar #'round-to (cram-tf:3d-vector->list calc-vec))
                  (destructuring-bind (d-x d-y d-z)
                      (mapcar #'round-to (cram-tf:3d-vector->list def-vec))
                    (assert-true (and (equalp c-x (* -1 d-x))
                                      (equalp c-y d-y)
                                      (equalp c-z d-z))))))))))

(when *run-donbot-retail-demo*
  (define-test get-lift-transforms-in-base-retail-dishwasher-tabs-in-shelf
    (init-tests-donbot-retail-demo)
    (sleep 2)
    (loop for grasp in '(:top :back :front) do
      (let ((calculated-lift-tfs
              (urdf-proj:with-simulated-robot
                (man-int::get-lift-transforms-in-base 
                 :dish-washer-tabs :dish-washer-tabs-1 :right grasp :shelf :shelf-2-base)))
            (defined-lift-tfs
              (man-int:get-object-type-wrt-reference-frame-lift-transforms
               :dish-washer-tabs :right grasp :shelf)))
        (loop for calc-tf in calculated-lift-tfs
              for def-tf in defined-lift-tfs do
                (let ((calc-vec (cl-tf:translation calc-tf))
                      (def-vec (cl-tf:translation def-tf)))
                  (destructuring-bind (c-x c-y c-z) 
                      (mapcar #'round-to (cram-tf:3d-vector->list calc-vec))
                    (destructuring-bind (d-x d-y d-z)
                        (mapcar #'round-to (cram-tf:3d-vector->list def-vec))
                      (assert-true (vec-list-eqp (list c-x c-y c-z)
                                                 (list d-x (* -1 d-y) d-z)))))))))))
