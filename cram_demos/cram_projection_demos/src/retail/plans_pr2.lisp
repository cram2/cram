;;;
;;; Copyright (c) 2019, Jonas Dech <jdech[at]uni-bremen.de
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
;;;     * Neither the name of the Institute for Artificial Intelligence /
;;;       University of Bremen nor the names of its contributors
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

(in-package :cram-pr2-shopping-demo)

(defun grasp-object-from-shelf (?object shelf)
  (let* ((?environment-name
           (rob-int:get-environment-name))
         (?search-location
           (if (eql shelf 1)
               (desig:a location
                        (on (desig:an object
                                      (type shelf)
                                      (urdf-name shelf-2-footprint)
                                      (part-of ?environment-name)
                                      (level 4)))
                        (side right))
               (desig:a location
                        (on (desig:an object
                                      (type shelf)
                                      (urdf-name shelf-1-footprint)
                                      (part-of ?environment-name)
                                      (level 4)))
                        (side right))))
         (?object-desig
           (desig:an object
                     (type ?object)
                     (location ?search-location))))
    (exe:perform
     (desig:an action
               (type transporting)
               (object ?object-desig)
               (target (desig:a location
                                (on (desig:an object
                                              (type basket)
                                              (name b)))
                                (for ?object-desig)
                                (attachment in-basket)))))))

(defun place-object-in-shelf (?object-type &rest ?target-poses)
  (declare (type symbol ?object-type)
           (type list ?target-poses))
  (let ((?env (rob-int:get-environment-name)))
    (exe:perform
     (desig:an action
               (type transporting)
               (object (desig:an object
                                 (type ?object-type)
                                 (location (desig:a location
                                                    (on (desig:an object
                                                                  (type counter-top)
                                                                  (urdf-name top)
                                                                  (part-of ?env)))
                                                    (side right)))))
               (target (desig:a location
                                (poses  ?target-poses)))))))


(defun demo ()
  (urdf-proj:with-simulated-robot
    (place-object-in-shelf
     :denkmit
     (cl-transforms-stamped:make-pose-stamped
      "map" 0
      (cl-transforms:make-3d-vector 0.7 0.7 0.68)
      (cl-transforms:make-quaternion 0 0 1 0))
     (cl-transforms-stamped:make-pose-stamped
      "map" 0
      (cl-transforms:make-3d-vector 0.7 0.7 0.68)
      (cl-transforms:make-quaternion 0 0 1 1))
     (cl-transforms-stamped:make-pose-stamped
      "map" 0
      (cl-transforms:make-3d-vector 0.7 0.7 0.68)
      (cl-transforms:make-quaternion 0 0 -1 0))
     (cl-transforms-stamped:make-pose-stamped
      "map" 0
      (cl-transforms:make-3d-vector 0.7 0.7 0.68)
      (cl-transforms:make-quaternion 0 0 -1 1)))
    (grasp-object-from-shelf :heitmann 2)
    (grasp-object-from-shelf :dove 1)))
