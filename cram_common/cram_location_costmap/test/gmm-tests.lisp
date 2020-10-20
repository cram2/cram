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
;;;

(in-package :location-costmap-tests)


(define-test make-gauss-mixture-models-list-as-input-and-tests-if-array-correct-created
  (let* ((means-l (list '(1.0d0 2.0d0)
                        '(3.0d0 4.0d0)))
         (covs-l (list '((1.0d0 0.0d0)
                         (0.0d0 1.0d0))
                       '((1.0d0 0.5d0)
                         (0.5d0 1.0d0))))
         (weights-l (list 0.7d0 0.3d0))
         (gmm (make-gauss-mixture-models
               means-l covs-l weights-l 2)))
    (assert-true
     (and
      (cram-math::equalp
       (costmap::gmm-means gmm)
       (cram-math:make-double-matrix
        2 2
        :initial-contents
        means-l))
      (cram-math::equalp
       (costmap::gmm-covariance gmm)
       (make-array
        (list 2 2 2)
        :initial-contents
        covs-l))
      (cram-math::equalp
       (costmap::gmm-weights gmm)
       (cram-math:make-double-vector
        2
        :initial-contents
        weights-l))))
    (assert-equal (costmap::gmm-dimensions gmm) (costmap::gmm-components gmm)
    (assert-equal (costmap::gmm-dimensions gmm) 2)
    (assert-equal 2 (length (costmap::gmm-distributions gmm))))))

(define-test make-gauss-mixture-models-array-as-input-and-test-if-created-correct
  (let* ((means-l (list '(1.0d0 2.0d0)
                        '(1.0d0 4.0d0)))
         (means-a (cram-math:make-double-matrix
                   2 2
                   :initial-contents
                   means-l))
         (covs-l (list '((1.0d0 0.0d0)
                         (0.0d0 1.0d0))
                       '((1.0d0 0.5d0)
                         (0.5d0 1.0d0))))
         (covs-a (make-array
                  (list 2 2 2)
                  :initial-contents
                  covs-l))
         (weights-l (list 0.3d0 0.7d0))
         (weights-a (cram-math:make-double-vector
                     2
                     :initial-contents
                     weights-l))
         (gmm (make-gauss-mixture-models
               means-a covs-a weights-a 2)))
    (assert-true
     (and
      (cram-math::equalp
       (costmap::gmm-means gmm)
       (cram-math:make-double-matrix
        2 2
        :initial-contents
        means-l))
      (cram-math::equalp
       (costmap::gmm-covariance gmm)
       (make-array
        (list 2 2 2)
        :initial-contents
        covs-l))
      (cram-math::equalp
       (costmap::gmm-weights gmm)
       (cram-math:make-double-vector
        2
        :initial-contents
        weights-l))))))

(define-test gmm-2d-test
  (let* ((means-l (list '(1.0d0 2.0d0)
                        '(1.0d0 4.0d0)))
         (means-a (cram-math:make-double-matrix
                   2 2
                   :initial-contents
                   means-l))
         (covs-l (list '((1.0d0 0.0d0)
                         (0.0d0 1.0d0))
                       '((1.0d0 0.5d0)
                         (0.5d0 1.0d0))))
         (covs-a (make-array
                  (list 2 2 2)
                  :initial-contents
                  covs-l))
         (weights-l (list 0.3d0 0.7d0))
         (weights-a (cram-math:make-double-vector
                     2
                     :initial-contents
                     weights-l))
         (gmm (make-gauss-mixture-models
               means-a covs-a weights-a 2))
         (gauss-dists (costmap::gmm-distributions gmm))
         (sample-l (list '(1.3d0 0.5d0)))
         (sample-a (cram-math:make-double-vector
                    2
                    :initial-contents
                    sample-l)))
    (assert-equal 2 (length gauss-dists))
    ;; might fail sometimes due to inaccuracies
    (assert-true
     (equalp
      (+ (* (nth 0 weights-l)
            (funcall (nth 0 gauss-dists)
                     sample-a))
         (* (nth 1 weights-l)
            (funcall (nth 1 gauss-dists)
                     sample-a)))
      (funcall (gmm gmm)
               sample-a)))))

(define-test get-value-list-as-input-test
  (let* ((means-l (list '(1.0d0 2.0d0)
                        '(1.0d0 4.0d0)))
         (means-a (cram-math:make-double-matrix
                   2 2
                   :initial-contents
                   means-l))
         (covs-l (list '((1.0d0 0.0d0)
                         (0.0d0 1.0d0))
                       '((1.0d0 0.5d0)
                         (0.5d0 1.0d0))))
         (covs-a (make-array
                  (list 2 2 2)
                  :initial-contents
                  covs-l))
         (weights-l (list 0.3d0 0.7d0))
         (weights-a (cram-math:make-double-vector
                     2
                     :initial-contents
                     weights-l))
         (gmm (make-gauss-mixture-models
               means-a covs-a weights-a 2))
         (gauss-dists (costmap::gmm-distributions gmm))
         (sample-l (list '(2.3d0 1.3d0)))
         (sample-a (cram-math:make-double-vector
                    2
                    :initial-contents
                    sample-l)))
    (assert-equal 2 (length gauss-dists))
    ;; might fail sometimes due to inaccuracies
    (assert-true
     (equalp
      (get-value gmm sample-l)
      (funcall (gmm gmm)
               sample-a)))))

(define-test get-value-array-as-input-test
  (let* ((means-l (list '(1.0d0 2.0d0)
                        '(1.0d0 4.0d0)))
         (means-a (cram-math:make-double-matrix
                   2 2
                   :initial-contents
                   means-l))
         (covs-l (list '((1.0d0 0.0d0)
                         (0.0d0 1.0d0))
                       '((1.0d0 0.5d0)
                         (0.5d0 1.0d0))))
         (covs-a (make-array
                  (list 2 2 2)
                  :initial-contents
                  covs-l))
         (weights-l (list 0.3d0 0.7d0))
         (weights-a (cram-math:make-double-vector
                     2
                     :initial-contents
                     weights-l))
         (gmm (make-gauss-mixture-models
               means-a covs-a weights-a 2))
         (gauss-dists (costmap::gmm-distributions gmm))
         (sample-l (list '(3.3d0 1.0d0)))
         (sample-a (cram-math:make-double-vector
                    2
                    :initial-contents
                    sample-l)))

    ;; might fail sometimes due to inaccuracies
    (assert-true
     (equalp
      (get-value gmm sample-a)
      (funcall (gmm gmm)
               sample-a)))))

