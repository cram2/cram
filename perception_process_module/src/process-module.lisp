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

(defvar *perception-role* nil)

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

(defun execute-object-search-functions (desig &key perceived-object (role *perception-role*))
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
          (sort obj-search-functions #'> :key #'third))))

(defun perceived-object->designator (desig obj)
  (let ((new-desig (make-designator 'object
                                    (make-new-desig-description
                                     desig obj))))
    (setf (slot-value new-desig 'data) obj)
    (setf (slot-value new-desig 'timestamp) (cut:current-timestamp))
    (setf (slot-value new-desig 'valid) t)
    (assert-desig-binding new-desig obj)
    new-desig))

(defun emit-perception-event (designator)
  (cram-plan-knowledge:on-event (make-instance 'cram-plan-knowledge:object-perceived-event
                                  :perception-source :perception-process-module
                                  :object-designator designator))
  designator)

(defun find-with-parent-desig (desig)
  "Takes the perceived-object of the parent designator as a bias for
   perception."
  (let* ((parent-desig (current-desig desig))
         (perceived-object (reference (newest-valid-designator parent-desig))))
    (or
     (when perceived-object
       
       (let ((perceived-objects
               (execute-object-search-functions parent-desig :perceived-object perceived-object)))
         (when perceived-objects
           (car (mapcar (lambda (perceived-object)
                          (emit-perception-event
                           (perceived-object->designator parent-desig perceived-object)))
                        perceived-objects)))))
     (find-with-new-desig desig))))

(defun find-with-new-desig (desig)
  "Takes a parent-less designator. A search is performed a new
   designator is generated for every object that has been found."
  (let ((perceived-objects (execute-object-search-functions desig)))
    ;; Sort perceived objects according to probability
    (mapcar (lambda (perceived-object)
              (emit-perception-event
               (perceived-object->designator desig perceived-object)))
            perceived-objects)))

(defparameter *known-roles* '(semantic-map handle-detector popcorn-detector)
  "Ordered list of known roles for designator resolution. They are
  processed in the order specified in this list")

(def-process-module perception (input)
  (assert (typep input 'object-designator))
  (ros-info (perception process-module) "Searching for object ~a" input)
  (let* ((newest-valid (newest-valid-designator input))
         (result
          (some (lambda (role)
                  (let ((*perception-role* role))
                    (if newest-valid
                        ;; Designator that has alrady been equated to
                        ;; one with bound to a perceived-object
                        (find-with-parent-desig newest-valid)
                        (find-with-new-desig input))))
                *known-roles*)))
    (unless result
      (fail 'object-not-found :object-desig input))
    (ros-info (perception process-module) "Found objects: ~a" result)
    result))
