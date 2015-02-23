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

(in-package :sem-map-coll-env)

(defparameter *board-thickness* 0.02)

(defclass sem-map-obj ()
  ((name :initarg :name :initform "" :type string)
   (type :initarg :type :type symbol)
   (index :initarg :index :initform 0 :type fixnum)
   (pose :initarg :pose :type pose)
   (dimensions :initarg :dimensions :type 3d-vector)))

(defgeneric get-sem-map-objs (type name)
  (:documentation "Queries all sem-map objects that belong to the
  object identified by `name'. Returns an instance of type
  `sem-map-obj'.")
  (:method ((type t) name)
    (warn 'simple-warning
          :format-control "Ignoring unknown type `~s' of object `~a'"
          :format-arguments (list type name))
    nil))

(defmethod get-sem-map-objs ((type (eql '|'CounterTop'|)) name)
  (lazy-mapcar (lambda (bdg)
                 (with-vars-bound (?pose ?dim)
                     bdg
                   (make-instance 'sem-map-obj
                     :name name
                     :type type
                     :pose (pl-matrix->pose ?pose)
                     :dimensions (apply #'make-3d-vector ?dim))))
               (json-prolog:prolog
                `(and ("current_object_pose" ,name ?pose)
                      ("map_object_dimensions" ,name ?w ?d ?h)
                      (= '(?d ?w ?h) ?dim))
                :package :sem-map-coll-env)))

(defmethod get-sem-map-objs ((type (eql '|'Drawer'|)) name)
  (lazy-mapcan (lambda (bdg)
                 (with-vars-bound (?pose ?dim)
                     bdg
                   (make-box
                    name type
                    (pl-matrix->pose ?pose)
                    (apply #'make-3d-vector ?dim)
                    :top nil)))
               (json-prolog:prolog
                `(and ("current_object_pose" ,name ?pose)
                      ("map_object_dimensions" ,name ?w ?d ?h)
                      (= '(?d ?w ?h) ?dim))
                :package :sem-map-coll-env)))

(defmethod get-sem-map-objs ((type (eql '|'Refrigerator'|)) name)
  (lazy-mapcan (lambda (bdg)
                 (with-vars-bound (?pose ?dim)
                     bdg
                   (lazy-append
                    (make-box
                     name type
                     (pl-matrix->pose ?pose)
                     (apply #'make-3d-vector ?dim)
                     :front nil)
                    (query-physical-parts name))))
               (json-prolog:prolog
                `(and ("current_object_pose" ,name ?pose)
                      ("map_object_dimensions" ,name ?w ?d ?h)
                      (= '(?d ?w ?h) ?dim))
                :package :sem-map-coll-env)))

(defmethod get-sem-map-objs ((type (eql '|'Cupboard'|)) name)
  (lazy-mapcan (lambda (bdg)
                 (with-vars-bound (?pose ?dim)
                     bdg
                   (lazy-append
                    (make-box
                     name type
                     (pl-matrix->pose ?pose)
                     (apply #'make-3d-vector ?dim)
                     :front nil)
                    (query-physical-parts name))))
               (json-prolog:prolog
                `(and ("current_object_pose" ,name ?pose)
                      ("map_object_dimensions" ,name ?w ?d ?h)
                      (= '(?d ?w ?h) ?dim))
                :package :sem-map-coll-env)))

(defmethod get-sem-map-objs ((type (eql '|'Dishwasher'|)) name)
  (lazy-mapcan (lambda (bdg)
                 (with-vars-bound (?pose ?dim)
                     bdg
                   (lazy-append
                    (make-box
                     name type
                     (pl-matrix->pose ?pose)
                     (apply #'make-3d-vector ?dim)
                     :front nil)
                    (query-physical-parts name))))
               (json-prolog:prolog
                `(and ("current_object_pose" ,name ?pose)
                      ("map_object_dimensions" ,name ?w ?d ?h)
                      (= '(?d ?w ?h) ?dim))
                :package :sem-map-coll-env)))

(defmethod get-sem-map-objs ((type (eql '|'Oven'|)) name)
  (lazy-mapcan (lambda (bdg)
                 (with-vars-bound (?pose ?dim)
                     bdg
                   (lazy-append
                    (make-box
                     name type
                     (pl-matrix->pose ?pose)
                     (apply #'make-3d-vector ?dim)
                     :front nil)
                    (query-physical-parts name))))
               (json-prolog:prolog
                `(and ("current_object_pose" ,name ?pose)
                      ("map_object_dimensions" ,name ?w ?d ?h)
                      (= '(?d ?w ?h) ?dim))
                :package :sem-map-coll-env)))

(defmethod get-sem-map-objs ((type (eql '|'Door'|)) name)
  (lazy-mapcar (lambda (bdg)
                 (with-vars-bound (?pose ?dim)
                     bdg
                   (make-instance 'sem-map-obj
                     :name name
                     :type type
                     :index 0
                     :pose (pl-matrix->pose ?pose)
                     :dimensions (apply #'make-3d-vector ?dim))))
               (json-prolog:prolog
                `(and ("current_object_pose" ,name ?pose)
                      ("map_object_dimensions" ,name ?w ?d ?h)
                      (= '(?d ?w ?h) ?dim))
                :package :sem-map-coll-env)))


(defun make-box (name type pose dimensions
                 &key (front t) (back t) (top t)
                   (bottom t) (left t) (right t))
  (declare (type pose pose)
           (type 3d-vector dimensions)
           (type string name)
           (type symbol type))
  "Returns a list of instances of type SEM-MAP-OBJ that form a box. By
  setting the parameters `front', `back', `top', `bottom', `left',
  `right' to NIL, the corresponding boards can be left out"
  (let ((pose-tf (reference-transform pose))
        (x (x dimensions))
        (y (y dimensions))
        (z (z dimensions)))
    (remove
     nil
     (list
      (when front
        (make-instance 'sem-map-obj
          :name name
          :type type
          :index 0
          :pose (cl-transforms:transform-pose
                 pose-tf
                 (make-pose
                  (make-3d-vector
                   (+ (/ x -2) (/ *board-thickness* 2)) 0 0)
                  (make-identity-rotation)))
          :dimensions (make-3d-vector *board-thickness* y z)))
      (when back
        (make-instance 'sem-map-obj
          :name name
          :type type
          :index 1
          :pose (cl-transforms:transform-pose
                 pose-tf
                 (make-pose
                  (make-3d-vector
                   (- (/ x 2) (/ *board-thickness* 2)) 0 0)
                  (make-identity-rotation)))
          :dimensions (make-3d-vector *board-thickness* y z)))
      (when right
        (make-instance 'sem-map-obj
          :name name
          :type type
          :index 2
          :pose (cl-transforms:transform-pose
                 pose-tf
                 (make-pose
                  (make-3d-vector
                   0 (+ (/ y -2) (/ *board-thickness* 2)) 0)
                  (make-identity-rotation)))
          :dimensions (make-3d-vector x *board-thickness* z)))
      (when left
        (make-instance 'sem-map-obj
          :name name
          :type type
          :index 3
          :pose (cl-transforms:transform-pose
                 pose-tf
                 (make-pose
                  (make-3d-vector
                   0 (- (/ y 2) (/ *board-thickness* 2)) 0)
                  (make-identity-rotation)))
          :dimensions (make-3d-vector x *board-thickness* z)))
      (when bottom
        (make-instance 'sem-map-obj
          :name name
          :type type
          :index 4
          :pose (cl-transforms:transform-pose
                 pose-tf
                 (make-pose
                  (make-3d-vector
                   0 0 (+ (/ z -2) (/ *board-thickness* 2)))
                  (make-identity-rotation)))
          :dimensions (make-3d-vector x y *board-thickness*)))
      (when top
        (make-instance 'sem-map-obj
          :name name
          :type type
          :index 5
          :pose (cl-transforms:transform-pose
                 pose-tf
                 (make-pose
                  (make-3d-vector
                   0 0 (- (/ z 2) (/ *board-thickness* 2)))
                  (make-identity-rotation)))
          :dimensions (make-3d-vector x y *board-thickness*)))))))

(defun query-sem-map (&optional map-name)
  (let ((map-name (or map-name
                      (with-vars-bound (?map-name)
                          (lazy-car
                           (crs:prolog `(semantic-map-name ?map-name)))
                        ?map-name))))
    (lazy-mapcan (lambda (bdg)
                   (with-vars-bound (?o ?type)
                       bdg
                     (get-sem-map-objs ?type (obj-name-sym->string ?o))))
                 (json-prolog:prolog
                  `(and ("map_root_objects" ,map-name ?objs)
                        ("member" ?o ?objs)
                        ("map_object_type" ?o ?tp)
                        ("rdf_atom_no_ns" ?tp ?type))
                  :package :sem-map-coll-env))))

(defun query-physical-parts (name)
  (lazy-mapcan
   (lambda (bdg)
     (with-vars-bound (?sub ?type) bdg
       (get-sem-map-objs ?type (obj-name-sym->string ?sub))))
   (json-prolog:prolog
    `(and
      ("rdf_has" ,name
                 "http://knowrob.org/kb/knowrob.owl#properPhysicalParts"
                 ?sub)
      ("map_object_type" ?sub ?tp)
      ("rdf_atom_no_ns" ?tp ?type))
    :package :sem-map-coll-env)))

(defun obj-name-sym->string (name-sym)
  (remove #\' (symbol-name name-sym)))

(defun pl-matrix->pose (matrix)
  (transform->pose
   (matrix->transform
    (make-array
     '(4 4) :displaced-to (make-array
                           16 :initial-contents matrix)))))
