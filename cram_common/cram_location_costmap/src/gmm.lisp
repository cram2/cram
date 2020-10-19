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

(in-package :location-costmap)


(defclass gauss-mixture-models ()
  ((dimensions
    :initform -1
    :initarg :dimensions
    :reader gmm-dimensions
    :documentation "Saves the dimension of the space.")
   (components
    :initform -1
    :initarg :components
    :reader gmm-components
    :documentation "Saves the number of components.")
   (means
    :initarg :means
    :reader gmm-means
    :documentation "Contains cram-math:double-matrix object
                    where each row is |dimensions| long and
                    has the height of |components|. Each row
                    saves the means of one component.")
   (covs
    :initarg :covs
    :reader gmm-covariance
    :documentation "Contains array of cram-math:double-matrix
                    objects, where each cram-math:double-matrix
                    contains the covariance matrix of one GMM
                    component. Therefore, this array contains
                    |components| many cram-math:double-matrix
                    objects, which are each |dimensions| times
                    |dimensions| big.")
   (gauss-dists
    :initform nil
    :reader gmm-distributions
    :documentation "Contains a list with |components| elements
                    each representing one component with a
                    multivariate gauss distribution (see
                    `cram-math:gauss' for more information).")
   (weights
    :initform nil
    :reader gmm-weights
    :initarg :weights
    :documentation "List of floats giving each component/gauss
                    distribution a weight.")))

(defmethod make-gauss-mixture-models ((means list)
                                      (covs list)
                                      (weights list)
                                      k &optional (dimensions 2))
  (make-gauss-mixture-models
   (cram-math:make-double-matrix
    dimensions k
    :initial-contents
    means)
   (make-array
    (list k dimensions dimensions)
    :initial-contents
    covs)
   (cram-math:make-double-vector
    k
    :initial-contents
    weights)
   k dimensions))
  
(defmethod make-gauss-mixture-models ((means simple-array)
                                      (covs simple-array)
                                      (weights simple-array)
                                      k &optional (dimensions 2))
  (when (and means covs weights)
    (let* ((gmm (make-instance ;; save means, covs and weights in gmm object
                 'cram-location-costmap::gauss-mixture-models 
                 :dimensions dimensions
                 :components k
                 :means means
                 :covs covs
                 :weights weights)))
      ;; create multivariate gauss distributions and save these in the
      ;; gmm object
      (with-slots (gauss-dists means covs) gmm
        (unless gauss-dists
          (setf (slot-value gmm 'gauss-dists)
                (when (and means covs)
                  (loop for i from 0 to (1- k)
                        collecting
                        (let* ((tmp-mean (cram-math::make-double-vector dimensions))
                               (tmp-cov (cram-math::make-double-matrix
                                         dimensions dimensions)))
                          ;; get means
                          (dotimes (j dimensions)
                            (setf (aref tmp-mean j 0)
                                  (aref means i j)))
                          ;; get covs
                          (dotimes (covs-row dimensions)
                            (dotimes (covs-col dimensions)
                              (setf (aref tmp-cov covs-row covs-col)
                                    (aref covs i covs-row covs-col))))
                          (cram-math:gauss tmp-cov tmp-mean)))))))
      gmm)))

(defmethod get-value ((gmm gauss-mixture-models) (x list))
  (with-slots (dimensions) gmm
    (get-value
     gmm
     (cram-math:make-double-vector dimensions
                                   :initial-contents
                                   x))))

(defmethod get-value ((gmm gauss-mixture-models) (x simple-array))
  (check-type x cram-math::double-matrix)
  (funcall (gmm gmm) x))
  
(defmethod gmm ((gmm gauss-mixture-models))
  (lambda (x)
    (with-slots (gauss-dists weights components) gmm
      (apply #'+
             (loop for gauss-dist in gauss-dists
                   for i from 0 to (1- components)
                   collecting                          
                   (* (aref weights i 0)
                      (funcall gauss-dist x)))))))
