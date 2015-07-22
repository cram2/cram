;;; Copyright (c) 2012, Georg Bartels <georg.bartels@cs.tum.edu>
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

(in-package :location-costmap-tests)

;;; main function to call to run tests

(defun run-location-costmap-tests ()
  (run-tests))

;;; auxiliary cost functions

(defun unit-cost-function (x y)
  (declare (ignore x y))
  1.0)

(defun special-cost-function-1 (x y)
  (declare (ignore y))
  (if (and (= x 0))
      1.0
      0.0))

(defun special-cost-function-2 (x y)
  (declare (ignore x))
  (if (and (= y 0))
      1.0
      0.0))

;;; auxiliary height generators

(defun height-generator-1 (x y)
  (declare (ignore x y))  
  (list 1.0))

(defun height-generator-2 (x y)
  (declare (ignore x y))  
  (list 2.0))

;;; auxiliary orientation generators

(defun orientation-generator-1 (x y old-orientation)
  (declare (ignore x y old-orientation))  
  (list (cl-transforms:make-identity-rotation)))

(defun orientation-generator-2 (x y quaternion)
  (declare (ignore x y quaternion))    
  (list (cl-transforms:axis-angle->quaternion
         (cl-transforms:make-3d-vector 1 0 0)
         (/ pi 2.0))))

;;; auxiliary macros

(defmacro with-costmaps ((&rest names) &body body)
  `(let ,(loop for n in names collect
               `(,n (make-instance 'location-costmap :width 2 :height 2
                      :origin-x 0 :origin-y 0 :resolution 1)))
     ,@body))

;;; actual test cases

(define-test register-cost-function
  (with-costmaps (map1 map2)
    ;; check if we get an error when asking for a costmap
    ;; even though no cost function has been registered, yet
    (assert-error 'no-cost-functions-registered (get-cost-map map1))
    (assert-error 'no-cost-functions-registered (get-cost-map map2))
    ;; register single cost-function and check the resulting map values
    (register-cost-function map1 #'unit-cost-function 1)
    (assert-true (= (get-map-value map1 0 0) 0.25))
    (assert-true (= (get-map-value map1 0 1) 0.25))
    (assert-true (= (get-map-value map1 1 0) 0.25))
    (assert-true (= (get-map-value map1 1 1) 0.25))
    ;; make sure that registration of further cost-function
    ;; does not change cached results
    (register-cost-function map1 #'special-cost-function-1 2)
    (assert-true (= (get-map-value map1 0 0) 0.25))
    (assert-true (= (get-map-value map1 0 1) 0.25))
    (assert-true (= (get-map-value map1 1 0) 0.25))
    (assert-true (= (get-map-value map1 1 1) 0.25))
    ;; check that multiple cost-functions also work
    (register-cost-function map2 #'special-cost-function-1 1)
    (register-cost-function map2 #'special-cost-function-2 2)
    (assert-true (= (get-map-value map2 0 0) 1.0))
    (assert-true (= (get-map-value map2 0 1) 0.0))
    (assert-true (= (get-map-value map2 1 0) 0.0))
    (assert-true (= (get-map-value map2 1 1) 0.0))))

(define-test register-height-generator
  (with-costmaps (map1)
    ;; check if default return value of 0 works
    (register-cost-function map1 #'unit-cost-function 1)
    (assert-true (= (cl-transforms:z (gen-costmap-sample-point map1)) 0))
    ;; register new height generator and check generated height
    (register-height-generator map1 #'height-generator-1)
    (assert-true (= (cl-transforms:z (gen-costmap-sample-point map1)) 1))
    ;; overwrite old height generator with different one and check
    (register-height-generator map1 #'height-generator-2)
    (assert-true (= (cl-transforms:z (gen-costmap-sample-point map1)) 2))))

(define-test register-orientation-generator
  (with-costmaps (map1)
    ;; without orientation generator, identity rotation should be returned
    (register-cost-function map1 #'unit-cost-function 1)
    (assert-true (cl-transforms:q=
                  (cl-transforms:make-identity-rotation)
                  (cl-transforms:orientation
                   (cut:lazy-car (costmap-samples map1)))))
    ;; register orientation generator, and check result
    (register-orientation-generator map1 #'orientation-generator-2)
    (assert-true (cl-transforms:q=
                  (cl-transforms:axis-angle->quaternion
                   (cl-transforms:make-3d-vector 1 0 0)
                   (/ pi 2.0))
                  (cl-transforms:orientation
                   (cut:lazy-car (costmap-samples map1)))))
    ;; register a second orientation generator, and
    ;; check that the result does not change, i.e. the first generator
    ;; discards the results of the second one
    (register-orientation-generator map1 #'orientation-generator-1)
    (assert-true (cl-transforms:q=
                  (cl-transforms:axis-angle->quaternion
                   (cl-transforms:make-3d-vector 1 0 0)
                   (/ pi 2.0))
                  (cl-transforms:orientation
                   (cut:lazy-car (costmap-samples map1)))))))

(define-test merge-costmaps
  (with-costmaps (map1 map2 map3)
    (register-cost-function map1 #'unit-cost-function 1)
    (register-cost-function map2 #'special-cost-function-1 2)
    (register-cost-function map3 #'special-cost-function-2 3)
    ;; merge single map with nothing
    (let ((map (merge-costmaps map1)))
      (assert-true (= (get-map-value map 0 0) 0.25))
      (assert-true (= (get-map-value map 0 1) 0.25))
      (assert-true (= (get-map-value map 1 0) 0.25))
      (assert-true (= (get-map-value map 1 1) 0.25)))
    ;; merge two different costmaps
    (let ((map (merge-costmaps map1 map2)))
      (assert-true (= (get-map-value map 0 0) 0.5))
      (assert-true (= (get-map-value map 0 1) 0.5))
      (assert-true (= (get-map-value map 1 0) 0.0))
      (assert-true (= (get-map-value map 1 1) 0.0)))
    ;; merge three different costmaps
    (let ((map (merge-costmaps map1 map2 map3)))
      (assert-true (= (get-map-value map 0 0) 1.0))
      (assert-true (= (get-map-value map 0 1) 0.0))
      (assert-true (= (get-map-value map 1 0) 0.0))
      (assert-true (= (get-map-value map 1 1) 0.0)))))
