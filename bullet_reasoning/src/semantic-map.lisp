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

(in-package :btr)

(defclass semantic-map-object (robot-object)
  ((parts :initarg :parts :initform (make-hash-table :test #'equal))))

(defclass semantic-map-part ()
  ((type :initarg :type :reader obj-type)
   (name :initarg :name :reader name)
   (urdf-name :initarg :urdf-link-name :reader urdf-name)
   (sub-parts :reader sub-parts)))

(defclass semantic-map-geom (semantic-map-part)
  ((pose :initarg :pose :reader pose)
   (dimensions :initarg :dimensions :reader dimensions)))

(defgeneric semantic-map-parts (map)
  (:method ((map semantic-map-object))
    (loop for part being the hash-values of (slot-value map 'parts)
          collecting part))

  (:method ((map semantic-map-part))
    (sub-parts map)))

(defgeneric semantic-map-part-names (map)
  (:method ((map semantic-map-object))
    (loop for name being the hash-keys of (slot-value map 'parts)
          collecting name))

  (:method ((map semantic-map-part))
    (mapcar #'name (sub-parts map))))

(defgeneric semantic-map-part (map name)
  (:method ((map semantic-map-object) name)
    (or (gethash name (slot-value map 'parts))
        (loop for p-name being the hash-keys in (slot-value map 'parts)
              using (hash-value part)
              when (equal (cram-roslisp-common:rosify-lisp-name name)
                          (subseq p-name (1+ (position #\# p-name))))
                do (return part))))

  (:method ((part semantic-map-part) name)
    (let ((name (cram-roslisp-common:rosify-lisp-name name)))
      (find name (sub-parts part)
            :key #'name
            :test (lambda (lhs rhs)
                    (equal (if (find #\# lhs)
                               (subseq lhs (1+ (position #\# lhs)))
                               lhs)
                           (if (find #\# rhs)
                               (subseq rhs (1+ (position #\# rhs)))
                               rhs)))))))

(defgeneric make-semantic-map-part (type name)
  (:method ((type t) name)
    (with-vars-bound (?pose ?dim)
        (lazy-car
         (json-prolog:prolog
          `(and ("objectPose" ,name ?pose)
                ("objectDimensions" ,name ?w ?d ?h)
                (= '(?d ?w ?h) ?dim))
          :package :btr))
      (if (or (is-var ?pose) (is-var ?dim))
          (make-instance 'semantic-map-part :type type :name name)
          (make-instance 'semantic-map-geom
                         :type type
                         :name name
                         :pose (cl-transforms:transform->pose
                                (cl-transforms:matrix->transform
                                 (make-array
                                  '(4 4) :displaced-to (make-array
                                                        16 :initial-contents ?pose))))
                         :dimensions (apply #'cl-transforms:make-3d-vector ?dim))))))

(defmethod sub-parts :before ((part semantic-map-part))
  (unless (slot-boundp part 'sub-parts)
    (setf (slot-value part 'sub-parts)
          (force-ll
           (lazy-mapcan
            (lambda (bdgs)
              (with-vars-bound (?type ?sub) bdgs
                (unless (or (is-var ?type) (is-var ?sub))
                  (list
                   (make-semantic-map-part
                    ?type
                    (remove #\' (symbol-name ?sub)))))))
            (json-prolog:prolog
             `(and
               ("rdf_has" ,(name part) "http://ias.cs.tum.edu/kb/knowrob.owl#properPhysicalParts" ?sub)
               ("objectType" ?sub ?tp)
               ("rdf_atom_no_ns" ?tp ?type))
             :package :btr))))))

(defmethod urdf-name :before ((part semantic-map-part))
  (unless (slot-boundp part 'urdf-name)
    (with-slots (name urdf-name) part
      (let ((label (var-value
                    '?link
                    (lazy-car (json-prolog:prolog
                               `("rdf_has"
                                 ,name "http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName"
                                 ("literal" ?link))
                               :package :btr)))))
        (unless (is-var label)
          (setf urdf-name (remove #\' (symbol-name label))))))))

(defun query-semantic-map ()
  (force-ll
   (lazy-mapcan
    (lambda (bdgs)
      (with-vars-bound (?type ?o) bdgs
        (unless (or (is-var ?type) (is-var ?o))
          (list (make-semantic-map-part ?type (remove #\' (symbol-name ?o)))))))
    (json-prolog:prolog
     '(and ("rootObjects" ?objs)
       ("member" ?o ?objs)
       ("objectType" ?o ?tp)
       ("rdf_atom_no_ns" ?tp ?type))
     :package :btr))))

(defun owl-types-equal (lhs rhs)
  (let ((lhs (etypecase lhs
               (symbol (remove #\' (symbol-name lhs)))
               (string lhs)))
        (rhs (etypecase rhs
               (symbol (remove #\' (symbol-name rhs)))
               (string rhs))))
    (equal lhs rhs)))

(defun sub-parts-with-type (map type &key (recursive t))
  "Returns a lazy list of all objects of type `type' that are children
of map. When `recursive' is T, recursively traverses all sub-parts, i.e. returns not only direct children."
  (lazy-mapcan (lambda (part)
                 (lazy-append
                  (when (owl-types-equal
                         (obj-type part)
                         (cram-roslisp-common:rosify-lisp-name type))
                    (list part))
                  (when recursive
                    (sub-parts-with-type part type))))
               (semantic-map-parts map)))

(defmethod (setf joint-state) :before (new-value (sem-map semantic-map-object) name)
  (attach-contacting-objects sem-map :test (lambda (obj link-name)
                                             (declare (ignore link-name))
                                             (typep obj 'household-object))))

(defmethod (setf link-pose) :before (new-value (sem-map semantic-map-object) name)
  (attach-contacting-objects sem-map :test (lambda (obj link-name)
                                             (declare (ignore link-name))
                                             (typep obj 'household-object))))

(defmethod copy-object ((obj semantic-map-object) (world bt-reasoning-world))
  (with-slots (pose parts) obj
    (change-class (call-next-method) 'semantic-map-object :parts parts)))

(defmethod add-object ((world bt-world) (type (eql 'semantic-map)) name pose &key urdf)
  (let* ((parts (query-semantic-map))
         (map (change-class (add-object world 'urdf name pose :urdf urdf) 'semantic-map-object)))
    (dolist (part parts)
      (setf (gethash (name part) (slot-value map 'parts)) part))
    map))
