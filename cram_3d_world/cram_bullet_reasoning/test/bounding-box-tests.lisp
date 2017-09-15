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

(in-package :bullet-reasoning-tests)

(define-test test-box-bounding-box
  (let ((*epsilon* 1e-6))
    (with-current-bullet-world (copy-world *current-bullet-world*)
      (cut:lazy-car
       (prolog:prolog `(and
                     (clear-bullet-world) 
                     (bullet-world ?w)
                     (assert (object ?w static-plane floor ((0 0 0) (0 0 0 1)) :normal (0 0 1) :constant 0))
                     (assert (object ?w box box-1 ((0 0 0.15) (0 0 0 1)) :size (0.3 0.3 0.3) :mass 0.0)))))
      (let ((object-bounding-box (aabb (object *current-bullet-world* 'box-1))))
        (assert-float-equal 0.0d0 (cl-transforms:x (bounding-box-center object-bounding-box)))
        (assert-float-equal 0.0d0 (cl-transforms:y (bounding-box-center object-bounding-box)))
        (assert-float-equal 0.15d0 (cl-transforms:z (bounding-box-center object-bounding-box)))
        (assert-float-equal 0.3d0 (cl-transforms:x (bounding-box-dimensions object-bounding-box)))
        (assert-float-equal 0.3d0 (cl-transforms:y (bounding-box-dimensions object-bounding-box)))
        (assert-float-equal 0.3d0 (cl-transforms:z (bounding-box-dimensions object-bounding-box)))))))

(define-test test-poses-on
  (let ((*epsilon* 1e-6))
    (with-current-bullet-world (copy-world *current-bullet-world*)
      (cut:lazy-car
       (prolog:prolog `(and
                        (clear-bullet-world) 
                        (bullet-world ?w)
                        (assert (object ?w static-plane floor ((0 0 0) (0 0 0 1)) :normal (0 0 1) :constant 0))
                        (assert (object ?w box box-1 ((0 0 0.0) (0 0 0 1)) :size (0.3 0.3 0.3) :mass 0.0)))))
      (let ((pose-on (obj-pose-on (cl-transforms:make-identity-pose) (object *current-bullet-world* 'box-1))))
        (assert-float-equal 0.0d0 (cl-transforms:x (cl-transforms:origin pose-on)))
        (assert-float-equal 0.0d0 (cl-transforms:y (cl-transforms:origin pose-on)))
        (assert-float-equal 0.15d0 (cl-transforms:z (cl-transforms:origin pose-on)))))))
