;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :desig)

;;; Location designators are resolved a little bit differently than
;;; object designators. To resolve, the cram/reasoning prolog
;;; predicate desig-loc is used. All solutions are provided and can be
;;; accessed with next-solution. A mechanism is provided to
;;; post-process the solutions from reasoning, e.g. to sort according
;;; to eucledian distance.

(defclass location-designator (designator designator-id-mixin)
  ((current-solution :reader current-solution :initform nil)))

(register-designator-class location location-designator)

(defmethod reference ((desig location-designator) &optional (role *default-role*))
  (with-slots (data current-solution) desig
    (unless current-solution
      (setf data (resolve-designator desig role))
      (unless data
        (error 'designator-error
               :format-control "Unable to resolve designator `~a'"
               :format-arguments (list desig)))
      (setf current-solution (location-proxy-solution->pose desig
                              (location-proxy-current-solution (car data))))
      (unless current-solution
        (error 'designator-error
               :format-control "Unable to resolve designator `~a'"
               :format-arguments (list desig)))
      (assert-desig-binding desig current-solution))
    current-solution))

(defmethod resolve-designator ((desig location-designator) (role (eql 'default-role)))
  (mapcar (curry #'apply #'make-location-proxy)
          (sort (mapcar (curry #'var-value '?value)
                        (force-ll (prolog `(desig-loc ,desig ?value))))
                #'> :key (compose #'location-proxy-precedence-value #'car))))

(defmethod next-solution ((desig location-designator))
  ;; Make sure that we initialized the designator properly
  (unless (slot-value desig 'current-solution)
    (reference desig))
  (with-slots (data) desig
    (or (successor desig)
        (let ((new-desig (make-designator 'location (description desig))))
          (or
           (let ((next (location-proxy-next-solution (car data))))
             (when next
               (setf (slot-value new-desig 'data) data)
               (setf (slot-value new-desig 'current-solution)
                     (location-proxy-solution->pose new-desig next))
               (prog1 (equate desig new-desig)
                 (assert-desig-binding new-desig (slot-value new-desig 'current-solution)))))
           (when (cdr data)
             (let ((next (location-proxy-current-solution (cadr data))))
               (when next
                 (setf (slot-value new-desig 'data) (cdr data))
                 (setf (slot-value new-desig 'current-solution)
                       (location-proxy-solution->pose new-desig next))
                 (prog1
                     (equate desig new-desig)
                   (assert-desig-binding new-desig (slot-value new-desig 'current-solution)))))))))))
