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

(in-package :perception-pm)

(define-condition object-not-found (plan-error)
  ((object-desig :initarg :object-desig :initform nil :reader object-not-found-desig)))

(defgeneric object-search-function (type desig &optional perceived-object)
  (:documentation "A function that performs a search of an object of a
                   specific type. `desig' is the designator describing
                   the object and `perceived-object' contains a
                   previously found object of the same properties."))

(defun execute-object-search-function (desig &optional perceived-object)
  "Executes the matching search function that fits the type property
   of `desig'. `perceived-object' is an optional instance that
   previously matched the object."
  (with-desig-props (type) desig
    (object-search-function (or type t) desig perceived-object)))

(defun perceived-object->designator (desig obj &optional parent-desig)
  (let ((new-desig (make-designator 'object
                                    (make-new-desig-description
                                     desig obj))))
    ;; Todo: Merge the object properties with the desinator's props
    ;; Todo: Use weak references here to make desigs gc-able
    (assert (null (object-desig obj)) ()
            "Cannot bind a perceived-object when it's already bound.")
    (setf (object-desig obj) new-desig)
    (setf (slot-value new-desig 'data) obj)
    (setf (slot-value new-desig 'timestamp)
          (funcall cut::*timestamp-function*))
    (setf (slot-value new-desig 'valid) t)
    (when parent-desig
      (equate parent-desig new-desig))
    (assert-desig-binding new-desig obj)
    new-desig))

(defun find-with-parent-desig (desig production-name)
  "Takes the perceived-object of the parent designator as a bias for
   perception and equates with the designator if possible. Fails
   otherwise."
  (let* ((parent-desig (current-desig desig))
         (perceived-object (or (desig-current-perceived-object parent-desig)
                               (desig-current-perceived-object parent-desig 'queried-object)))
         (perceived-objects nil))
    (assert perceived-object)
    (or
     (crs:with-production-handlers
         ((production-name (op &key ?perceived-object)
            (when (eq op :assert)
              (pushnew ?perceived-object perceived-objects))))
       ;; We ignore objects that have already been perceived
       ;; since we got the info we are interested in already
       ;; (by the desig's reference) Note: Later, when falling
       ;; back to the default search, this information _is_
       ;; used, but in FIND-WITH-NEW-DESIG
       (setf perceived-objects nil)
       (execute-object-search-function parent-desig perceived-object)
       (when perceived-objects
         (list (perceived-object->designator parent-desig
                                             (car (sort perceived-objects #'>
                                                        :key #'perceived-object-probability))
                                             parent-desig))))
      ;; Ok. No object found so far. We need to use our fallback
      ;; solution.  It is like searching with a new designator, but we
      ;; need to asure that the result is not bound to any other
      ;; designator than ours. We first create a new desig with the same
      ;; properties as ours, check for the result designator not
      ;; having any other ancestor and then equating `desig' with the
      ;; new one.
      (let* ((tmp-desig (make-designator 'object (description parent-desig)))
             (result (find-with-new-desig tmp-desig production-name))
             (matching-result-desig (find-if (curry #'desig-equal parent-desig) result)))
        (unless matching-result-desig
          (when perceived-object
            (setf (slot-value parent-desig 'data) nil)
            (retract-desig-binding parent-desig perceived-object))
          (fail 'object-not-found :object-desig parent-desig))
        matching-result-desig))))

(defun find-with-new-desig (desig production-name)
  "Takes a parent-less designator. A search is performed a new
   designator is generated for every object that has been found. If a
   found object matches a previously found object, the new desingator
   and the previous one are equated. Please note that although the new
   designator might be equated to old ones, it is not equated to
   `desig' yet. This decision must be made by the caller of the
   process module."
  (let ((perceived-objects nil)
        (previous-perceived-objects nil))
    (crs:with-production-handlers
        ((production-name (op &key ?perceived-object)
           (when (eq op :assert)
             (pushnew ?perceived-object perceived-objects))))
      ;; If there are matching PERCEIVED-OBJECTS already, registration
      ;; gets triggered and they are in `perceived-objects'. We want
      ;; to try these first because perception is much faster if we
      ;; re-use old perceptions.
      (when perceived-objects
        (setf previous-perceived-objects (sort perceived-objects #'>
                                               :key #'object-timestamp))
        (setf perceived-objects nil)
        (loop for perceived-object in previous-perceived-objects
              until (execute-object-search-function desig perceived-object)))
      ;; When not found yet, continue with default search
      (unless perceived-objects
        (execute-object-search-function desig nil))
      (unless perceived-objects
        (fail 'object-not-found :object-desig desig))
      ;; Sort perceived objects according to probability
      (when perceived-objects
        (let* ((sorted-perceived-objects
                (sort perceived-objects #'> :key #'perceived-object-probability)))
          (mapcar (lambda (perceived-object)
                    ;; We need to remove incompatible objects again
                    ;; here since perceived objects can be more
                    ;; detailed than the desig description that we had
                    ;; initially (and that is used in the rete
                    ;; production). That means that some previously
                    ;; perceived objects might be incompatible after
                    ;; perception.
                    (let ((matching-object (matching-object
                                            perceived-object
                                            (remove-if-not (alexandria:compose
                                                            (curry #'compatible-properties
                                                                   (object-properties perceived-object))
                                                            #'object-properties)
                                                           previous-perceived-objects))))
                      (cond (matching-object
                             (setf previous-perceived-objects
                                   (delete matching-object previous-perceived-objects))
                             (perceived-object->designator desig perceived-object
                                                           (object-desig matching-object)))
                            (t
                             (perceived-object->designator desig perceived-object)))))
                  sorted-perceived-objects))))))

(defun newest-valid-designator (desig)
  (labels ((find-valid-desig (desig)
             (cond ((not desig) nil)
                   ((valid desig)
                    desig)
                   (t (find-valid-desig (parent desig))))))
    (find-valid-desig (current-desig desig))))

(def-process-module perception (input)
  (assert (typep input 'object-designator))
  (let ((productuion-name (gensym "DESIG-PRODUCTION-")))
    (unwind-protect
         (progn
           (crs:register-production productuion-name
                                    (designator->production input '?perceived-object))
           (cond (;; Designator that has alrady been equated
                  (parent input)
                  (find-with-parent-desig (newest-valid-designator input) productuion-name))
                 (t
                  (find-with-new-desig input productuion-name))))
      (crs:remove-production productuion-name))))
