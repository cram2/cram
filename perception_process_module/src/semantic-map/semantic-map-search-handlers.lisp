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

(defclass semantic-map-perceived-object (perceived-object)
  ((name :reader name :initarg :name)
   (owl-name :reader owl-name :initarg :owl-name)))

(defmethod make-new-desig-description ((old-desig object-designator)
                                       (po semantic-map-perceived-object))
  (let ((description (call-next-method)))
    (if (member 'name description :key #'car)
        description
        (cons `(name ,(name po)) description))))

(defun prologify-obj-name (name)
  (etypecase name
    (symbol (rosify-lisp-name name))
    (string name)))

(def-object-search-function query-semantic-map-object-name semantic-map
    (((name ?name)) desig perceived-object)
  (declare (ignore perceived-object))
  (with-desig-props (name) desig
    (force-ll
     (lazy-mapcan
      (lambda (solution)
        (with-vars-bound (?pose ?o) solution
          (unless (is-var ?pose)
            (list
             (make-instance
              'semantic-map-perceived-object
              :name name
              :owl-name (remove #\' (symbol-name ?o))
              :pose (cl-transforms:matrix->transform
                     (make-array
                      '(4 4) :displaced-to (make-array
                                            16 :initial-contents ?pose)))
              :probability 1.0)))))
      (lazy-filter
       (lambda (bdg)
         (with-vars-bound (?objname) bdg
           (equal (remove #\' (symbol-name ?objname))
                  (prologify-obj-name name))))
       (cpl-impl:without-scheduling
         (json-prolog:prolog
          `(and ("rootObjects" ?objs)
                ("member" ?o ?objs)
                ("rdf_atom_no_ns" ?o ?objname)
                ("objectPose" ?o ?pose))
          :package :perception-pm)))))))

(def-object-search-function query-semantic-map-object-type semantic-map
    (((type ?type)) desig perceived-object)
  (declare (ignore perceived-object))
  (with-desig-props (type) desig
    (force-ll
     (lazy-mapcan
      (lambda (solution)
        (with-vars-bound (?objname ?o ?pose) solution
          (unless (or (is-var ?objname) (is-var ?pose))
            (list
             (make-instance
              'semantic-map-perceived-object
              :name (remove #\' (symbol-name ?objname))
              :owl-name (remove #\' (symbol-name ?o))
              :pose (cl-transforms:matrix->transform
                     (make-array
                      '(4 4) :displaced-to (make-array
                                            16 :initial-contents ?pose)))
              :probability 1.0)))))
      (lazy-filter
       (lambda (bdg)
         (with-vars-bound (?type) bdg
           (equal (remove #\' (symbol-name ?type))
                  (prologify-obj-name type))))
       (cpl-impl:without-scheduling
         (json-prolog:prolog
          `(and ("rootObjects" ?objs)
                ("member" ?o ?objs)
                ("objectType" ?o ?tp)
                ("rdf_atom_no_ns" ?o ?objname)
                ("rdf_atom_no_ns" ?tp ?type)
                ("objectPose" ?o ?pose))
          :package :perception-pm)))))))

(def-object-search-function query-semantic-map-object-part semantic-map
    (((part-of ?parent)) desig perceived-object)
  (declare (ignore perceived-object))
  (labels ((find-obj-parts (obj-name)
             (lazy-mapcan
              (lambda (solution)
                (cons
                 solution
                 (with-vars-bound (?sub) solution
                   (unless (is-var ?sub)
                     (let ((child-name (remove #\' (symbol-name ?sub))))
                       (find-obj-parts child-name))))))
              (cpl-impl:without-scheduling
                (json-prolog:prolog
                 `(and
                   ("rdf_has" ,obj-name
                              "http://ias.cs.tum.edu/kb/knowrob.owl#properPhysicalParts"
                              ?sub)
                   ("rdf_atom_no_ns" ?sub ?objname)
                   ("objectType" ?sub ?tp)
                   ("rdf_atom_no_ns" ?tp ?type)
                   ("objectPose" ?sub ?pose))
                 :package :perception-pm)))))
    (with-desig-props (part-of type name) desig
      (check-type part-of object-designator)
      (let ((parent-ref (reference part-of)))
        (when (typep parent-ref 'semantic-map-perceived-object)
          (force-ll
           (lazy-mapcan
            (lambda (solution)
              (with-vars-bound (?o ?objname ?pose) solution
                (unless (is-var ?pose)
                  (list
                   (make-instance
                    'semantic-map-perceived-object
                    :name (remove #\' (symbol-name ?objname))
                    :owl-name (remove #\' (symbol-name ?o))
                    :pose (cl-transforms:matrix->transform
                           (make-array
                            '(4 4) :displaced-to (make-array
                                                  16 :initial-contents ?pose)))
                    :probability 1.0)))))
            (lazy-filter
             (lambda (bdg)
               (with-vars-bound (?objname ?type) bdg
                 (and
                  (if type
                      (equal (remove #\' (symbol-name ?type))
                             (prologify-obj-name type))
                      t)
                  (if name
                      (equal (remove #\' (symbol-name ?objname))
                             (prologify-obj-name name))
                      t))))
             (find-obj-parts (owl-name parent-ref))))))))))
