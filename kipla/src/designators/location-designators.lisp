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

(in-package :kipla-reasoning)

;;; Location designators are resolved a little bit differently than
;;; object designators (at least for now.) To resolve, the
;;; cram/reasoning prolog predicate desig-loc is used. All solutions
;;; are provided an can be accessed with next-solution. A mechanism is
;;; provided to post-process the solutions from reasoning, e.g. to
;;; sort according to eucledian distance.


;;; Same locations must not only be equatable but also eq!
;;; This is necessary to make rete work!

(defvar *location-designators* nil
  "An alist containing a mapping of desig descriptions to the
   designator instance.")

(defclass location-designator (designator designator-id-mixin)
  ())
(register-designator-type location location-designator)

(defmethod make-designator :around ((type (eql (find-class 'location-designator)))
                                    description &optional parent)
  (declare (ignore parent))
  (flet ((compare-fun (obj-1 obj-2)
           (or (equal obj-1 obj-2)
               (eql (object-id obj-1)
                    (object-id obj-2)))))
    (let ((old-desig (cdr (assoc description
                                 *location-designators*
                                 :test (rcurry #'tree-equal :test #'compare-fun)))))
      (or old-desig
          (let ((desig (call-next-method)))
            (push (cons description desig) *location-designators*)
            desig)))))

(defmethod reference ((desig location-designator))
  (unless (slot-value desig 'data)
    (setf (slot-value desig 'data)
          (lazy-mapcar (alexandria:compose #'(lambda (descr)
                                               (etypecase descr
                                                 (string (jlo:make-jlo :name descr))
                                                 (number (jlo:make-jlo :id descr))
                                                 (jlo:jlo descr)))
                                           (curry #'var-value '?loc))
                       (prolog `(desig-loc ,desig ?loc))))
    (rete-assert `(desig-bound ,desig ,(slot-value desig 'data))))
  (or (lazy-car (slot-value desig 'data))
      (error "Unable to resolve location designator `~a'" (description desig))))

(defmethod next-solution ((desig location-designator))
  (with-slots (data) desig
    (when (car (cut:lazy-cdr data))
      (cond ((successor desig)
             (successor desig))
            (t
             (let ((new-desig (make-designator 'location (description desig) desig)))
               (setf (slot-value new-desig 'data) (cut:lazy-cdr data))
               (rete-assert `(desig-bound ,new-desig ,(slot-value new-desig 'data)))
               new-desig))))))
