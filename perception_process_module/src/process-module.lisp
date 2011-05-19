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

(defmacro def-object-search-function (function-name role (props desig perceived-object)
                                      &body body)
  (check-type function-name symbol)
  (check-type role symbol)
  (check-type props list)
  (check-type desig symbol)
  (check-type perceived-object symbol)
  (assert (every #'listp props) ()
          "The parameter `props' is not a valid designator property list")
  `(progn
     (defun ,function-name (,desig ,perceived-object)
       ,@body)

     (def-fact-group ,(intern (concatenate 'string (symbol-name function-name) "-FACTS"))
         (object-search-function object-search-function-order)
       
       (<- (object-search-function ?desig ,role ?fun)
         ,@(mapcar (lambda (prop)
                     `(desig-prop ?desig ,prop))
                   props)
         (lisp-fun symbol-function ,function-name ?fun))

       (<- (object-search-function-order ?fun ,(length props))
         (lisp-fun symbol-function ,function-name ?fun)))))

(defun execute-object-search-functions (desig &optional perceived-object (role *default-role*))
  "Executes the matching search functions that fit the properties of
   `desig' until one succeeds. `role' specifies the role under which
   the search function should be found. If `role' is set to NIL, all
   matching search functins are used. The order in which the search
   functions are executed is determined by the number of designator
   properties that are matched. Functions that are more specific,
   i.e. match more pros are executed first. `perceived-object' is an
   optional instance that previously matched the object."
  (let ((obj-search-functions (force-ll
                               (lazy-mapcar
                                (lambda (bdg)
                                  (with-vars-bound (?role ?fun ?order) bdg
                                    (list ?fun ?role ?order)))
                                (prolog `(and (object-search-function ,desig ?role ?fun)
                                              (object-search-function-order ?fun ?order))
                                        (when role
                                          (add-bdg '?role role nil)))))))
    (some (lambda (fun) (funcall (first fun) desig perceived-object))
          (sort  obj-search-functions #'> :key #'third))))

(defun perceived-object->designator (desig obj &optional parent-desig)
  (let ((new-desig (make-designator 'object
                                    (make-new-desig-description
                                     desig obj))))
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

(defun find-with-parent-desig (desig)
  "Takes the perceived-object of the parent designator as a bias for
   perception and equates with the designator if possible. Fails
   otherwise."
  (let* ((parent-desig (current-desig desig))
         (perceived-object (or (desig-current-perceived-object parent-desig)
                               (desig-current-perceived-object parent-desig 'queried-object)
                               (desig-current-perceived-object parent-desig 'semantic-map-object))))
    (or
     (when perceived-object
       (let ((perceived-objects (execute-object-search-functions parent-desig perceived-object)))
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
            (result (find-with-new-desig tmp-desig))
            (matching-result-desig (find-if (curry #'desig-equal parent-desig) result)))
       (unless matching-result-desig
         (when perceived-object
           (setf (slot-value parent-desig 'data) nil)
           (retract-desig-binding parent-desig perceived-object))
         (fail 'object-not-found :object-desig parent-desig))
       matching-result-desig))))

(defun find-with-new-desig (desig)
  "Takes a parent-less designator. A search is performed a new
   designator is generated for every object that has been found. If a
   found object matches a previously found object, the new desingator
   and the previous one are equated. Please note that although the new
   designator might be equated to old ones, it is not equated to
   `desig' yet. This decision must be made by the caller of the
   process module."
  (let ((perceived-objects (execute-object-search-functions desig nil)))
    (unless perceived-objects
      (fail 'object-not-found :object-desig desig))
    ;; Sort perceived objects according to probability
    (mapcar (lambda (perceived-object)
              (perceived-object->designator desig perceived-object))
            (sort perceived-objects #'> :key #'perceived-object-probability))))

(defun newest-valid-designator (desig)
  (labels ((find-valid-desig (desig)
             (cond ((not desig) nil)
                   ((valid desig)
                    desig)
                   (t (find-valid-desig (parent desig))))))
    (find-valid-desig (current-desig desig))))

(defparameter *known-roles* '(semantic-map cop)
  "Ordered list of known roles for designator resolution. They are
  processed in the order specified in this list")

(def-process-module perception (input)
  (assert (typep input 'object-designator))
  (ros-info (perception process-module) "Searching for object ~a" input)
  (let* ((newest-valid (newest-valid-designator input))
         (result
          (some (lambda (role)
                  (let ((*default-role* role))
                    (if newest-valid
                        ;; Designator that has alrady been equated to
                        ;; one with bound to a perceived-object
                        (find-with-parent-desig newest-valid)
                        (find-with-new-desig input))))
                *known-roles*)))
    (ros-info (perception process-module) "Found objects: ~a" result)
    result))
