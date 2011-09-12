;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :sem-map-utils)

(defvar *cached-semantic-map* nil)
(defvar *cached-owl-types* (make-hash-table :test 'equal))

(defclass semantic-map ()
  ((parts :initform (make-hash-table :test 'equal) :initarg :parts)))

(defclass semantic-map-part ()
  ((type :initarg :type :reader obj-type)
   (name :initarg :name :reader name)
   (owl-name :initarg :owl-name :reader owl-name)
   (urdf-name :initarg :urdf-link-name :reader urdf-name)
   (sub-parts :reader sub-parts)
   (aliases :initarg :aliases :reader aliases)))

(defclass semantic-map-geom (semantic-map-part)
  ((pose :initarg :pose :reader pose)
   (dimensions :initarg :dimensions :reader dimensions)))

(defgeneric semantic-map-parts (map &key recursive)
  (:method ((map semantic-map) &key recursive)
    (let ((direct-children (loop for part being the hash-values of (slot-value map 'parts)
                                 collecting part)))
      (append direct-children
              (when recursive
                (mapcan (lambda (child)
                          (semantic-map-parts child :recursive recursive))
                        direct-children)))))

  (:method ((map semantic-map-part) &key recursive)
    (let ((direct-children (sub-parts map)))
      (append direct-children
              (when recursive
                (mapcan (lambda (child)
                          (semantic-map-parts child :recursive recursive))
                        direct-children))))))

(defgeneric semantic-map-part-names (map)
  (:method ((map semantic-map))
    (loop for name being the hash-keys of (slot-value map 'parts)
          collecting name))

  (:method ((map semantic-map-part))
    (mapcar #'name (sub-parts map))))

(defgeneric semantic-map-part (map name &key recursive)
  (:method ((map semantic-map) name &key recursive)
    (or (gethash name (slot-value map 'parts))
        (loop for p-name being the hash-keys in (slot-value map 'parts)
              using (hash-value part)
              when (equal (cram-roslisp-common:rosify-lisp-name name)
                          (subseq p-name (1+ (position #\# p-name))))
                do (return part))
        (when recursive
          (some (lambda (part)
                  (semantic-map-part part name :recursive recursive))
                (semantic-map-parts map)))))

  (:method ((part semantic-map-part) name &key recursive)
    (let ((name (cram-roslisp-common:rosify-lisp-name name)))
      (or
       (find name (sub-parts part)
             :key #'name
             :test (lambda (lhs rhs)
                     (equal (if (find #\# lhs)
                                (subseq lhs (1+ (position #\# lhs)))
                                lhs)
                            (if (find #\# rhs)
                                (subseq rhs (1+ (position #\# rhs)))
                                rhs))))
       (when recursive
         (some (lambda (part)
                 (semantic-map-part part name :recursive recursive))
               (semantic-map-parts part)))))))

(defgeneric make-semantic-map-part (type name owl-name)
  (:method ((type t) name owlname)
    (with-vars-bound (?pose ?dim ?labels)
        (lazy-car
         (json-prolog:prolog
          `(and ("objectPose" ,owlname ?pose)
                ("objectDimensions" ,owlname ?w ?d ?h)
                ("findall" ?l ("objectLabel" ,owlname ?l) ?labels)
                (= '(?d ?w ?h) ?dim))
          :package :sem-map-utils))
      (let ((aliases (unless (is-var ?labels)
                       ?labels)))
        (if (or (is-var ?pose) (is-var ?dim))
            (make-instance 'semantic-map-part
              :type type :name name :owl-name owlname
              :aliases (mapcar (lambda (label)
                                 (remove #\' (symbol-name label)))
                               aliases))
            (make-instance 'semantic-map-geom
              :type type
              :name name
              :owl-name owlname
              :pose (cl-transforms:transform->pose
                     (cl-transforms:matrix->transform
                      (make-array
                       '(4 4) :displaced-to (make-array
                                             16 :initial-contents ?pose))))
              :dimensions (apply #'cl-transforms:make-3d-vector ?dim)
              :aliases (mapcar (lambda (label)
                                 (remove #\' (symbol-name label)))
                               aliases)))))))

(defmethod sub-parts :before ((part semantic-map-part))
  (unless (slot-boundp part 'sub-parts)
    (setf (slot-value part 'sub-parts)
          (force-ll
           (lazy-mapcan
            (lambda (bdgs)
              (with-vars-bound (?type ?name ?sub) bdgs
                (unless (or (is-var ?type) (is-var ?sub))
                  (list
                   (make-semantic-map-part
                    (remove #\' (symbol-name ?type))
                    (remove #\' (symbol-name ?name))
                    (remove #\' (symbol-name ?sub)))))))
            (json-prolog:prolog
             `(and
               ("rdf_has" ,(owl-name part) "http://ias.cs.tum.edu/kb/knowrob.owl#properPhysicalParts" ?sub)
               ("objectType" ?sub ?tp)
               ("rdf_atom_no_ns" ?tp ?type)
               ("rdf_atom_no_ns" ?sub ?name))
             :package :sem-map-utils))))))

(defmethod urdf-name :before ((part semantic-map-part))
  (unless (slot-boundp part 'urdf-name)
    (with-slots (owl-name urdf-name) part
      (let ((label (var-value
                    '?link
                    (lazy-car (json-prolog:prolog
                               `("rdf_has"
                                 ,owl-name "http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName"
                                 ("literal" ?link))
                               :package :sem-map-utils)))))
        (unless (is-var label)
          (setf urdf-name (remove #\' (symbol-name label))))))))

(defmethod (setf pose) (new-value (geom semantic-map-geom))
  (with-slots (pose) geom
    ;; TODO: update joint-state in knowrob's knowledge base
    (setf pose new-value)))

(defun urdf-obj-name (urdf-name)
  (with-vars-bound (?name)
      (lazy-car
       (json-prolog:prolog
        `(and
          ("rdf_has"
           ?owlname "http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName"
           ("literal" ,urdf-name))
          ("rdf_atom_no_ns" ?owlname ?name))
        :package :sem-map-utils))
    (unless (is-var ?name)
      (remove #\' (symbol-name ?name)))))

(defun clear-semantic-map ()
  (setf *cached-semantic-map* nil))

(defmacro with-clear-semantic-map (&body body)
  `(let ((*cached-semantic-map* nil))
     ,@body))

(defun get-semantic-map ()
  (or *cached-semantic-map*
      (setf *cached-semantic-map*
            (make-instance 'semantic-map
              :parts (alexandria:alist-hash-table
                      (mapcar (lambda (elem)
                                (cons (name elem) elem))
                              (force-ll
                               (lazy-mapcan
                                (lambda (bdgs)
                                  (with-vars-bound (?type ?n ?o) bdgs
                                    (unless (or (is-var ?type) (is-var ?o))
                                      (list (make-semantic-map-part
                                             (remove #\' (symbol-name ?type))
                                             (remove #\' (symbol-name ?n))
                                             (remove #\' (symbol-name ?o)))))))
                                (json-prolog:prolog
                                 '(and ("rootObjects" ?objs)
                                   ("member" ?o ?objs)
                                   ("objectType" ?o ?tp)
                                   ("rdf_atom_no_ns" ?tp ?type)
                                   ("rdf_atom_no_ns" ?o ?n))
                                 :package :sem-map-utils))))
                      :test 'equal)))))

(defun owl-names-equal (lhs rhs)
  (let ((lhs (etypecase lhs
               (symbol (remove #\' (symbol-name lhs)))
               (string lhs)))
        (rhs (etypecase rhs
               (symbol (remove #\' (symbol-name rhs)))
               (string rhs))))
    (equal lhs rhs)))

(defun is-owl-type (type ref-type)
  "Checks if `type' is equal to `ref-type', i.e. if it's a sub-type of
  `ref-type'"
  (flet ((cached-sub-types (ref-type)
           (declare (type string ref-type))
           (or (gethash ref-type *cached-owl-types*)
               (let* ((type-namespace-str "http://ias.cs.tum.edu/kb/knowrob.owl#")
                      (ref-type-w/ns-str (concatenate
                                          'string
                                          type-namespace-str
                                          ref-type)))
                 (setf (gethash ref-type *cached-owl-types*)
                       (force-ll
                        (lazy-mapcar (lambda (bdg)
                                       (with-vars-bound (?type) bdg
                                         (let ((type-str (remove #\' (symbol-name ?type))))
                                           (subseq type-str (1+ (position #\# type-str))))))
                                     (json-prolog:prolog
                                      `("owl_subclass_of" ?type ,ref-type-w/ns-str)
                                      :package :sem-map-utils))))))))
    (let ((ref-type-str (etypecase ref-type
                          (symbol (remove #\' (symbol-name ref-type)))
                          (string ref-type)))
          (type-str (etypecase type
                      (symbol (remove #\' (symbol-name type)))
                      (string type))))
      (find type-str (cached-sub-types ref-type-str)
            :test #'equal))))

(defun sub-parts-with-type (map type &key (recursive t))
  "Returns a lazy list of all objects of type `type' that are children
of map. When `recursive' is T, recursively traverses all sub-parts, i.e. returns not only direct children."
  ;; Update the cache if not updated yet
  (get-semantic-map)
  (let ((type (etypecase type
                (symbol (cram-roslisp-common:rosify-lisp-name type))
                (string type))))
    (lazy-mapcan (lambda (part)
                   (lazy-append
                    (when (is-owl-type (obj-type part) type)
                      (list part))
                    (when recursive
                      (sub-parts-with-type part type))))
                 (semantic-map-parts map))))

(defun sub-parts-with-name (map name &key (recursive t))
  "Returns a lazy list of all objects of type `type' that are children
of map. When `recursive' is T, recursively traverses all sub-parts, i.e. returns not only direct children."
  ;; Update the cache if not updated yet
  (get-semantic-map)
  (let ((name (etypecase name
                (symbol (cram-roslisp-common:rosify-lisp-name name))
                (string name))))
    (lazy-mapcan (lambda (part)
                   (lazy-append
                    (when (or (owl-names-equal (owl-name part) name)
                              (owl-names-equal (name part) name)
                              (member name (aliases part) :test #'owl-names-equal))
                      (list part))
                    (when recursive
                      (sub-parts-with-name part name))))
                 (semantic-map-parts map))))
