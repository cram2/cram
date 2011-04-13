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

(defclass semantic-map-object (object)
  ((pose :initarg :pose)
   (geoms :initarg :geoms :initform (make-hash-table :test #'equal))))

(defgeneric semantic-map-geoms (map)
  (:method ((map semantic-map-object))
    (loop for geom being the hash-values of (slot-value map 'geoms)
          collecting geom)))

(defgeneric semantic-map-geom-names (map)
  (:method ((map semantic-map-object))
    (loop for name being the hash-keys of (slot-value map 'geoms)
          collecting name)))

(defgeneric semantic-map-geom (map name)
  (:method ((map semantic-map-object) name)
    (gethash name (slot-value map 'geoms))))

(defclass semantic-map-geom ()
  ((type :initarg :type :reader obj-type)
   (name :initarg :name :reader name)
   (pose :initarg :pose :reader pose)
   (dimensions :initarg :dimensions :reader dimensions)))

(defgeneric make-semantic-map-geom (type name pose dimensions)
  (:method ((type t) name pose dimensions)
    (make-instance 'semantic-map-geom
                   :type type
                   :name name
                   :pose pose
                   :dimensions dimensions)))

(defun query-semantic-map-geoms (pose-transform)
  (force-ll
   (lazy-mapcar
    (lambda (bdgs)
      (make-semantic-map-geom
       (var-value '?type bdgs)
       (var-value '?o bdgs)
       (cl-transforms:transform->pose
        (cl-transforms:transform*
         pose-transform
         (cl-transforms:matrix->transform
          (make-array
           '(4 4) :displaced-to (make-array
                                 16 :initial-contents (var-value '?pose bdgs))))))
       (apply #'cl-transforms:make-3d-vector (var-value '?dim bdgs))))
    (json-prolog:prolog
     '(and ("rootObjects" ?objs)
       ("member" ?o ?objs)
       ("objectType" ?o ?tp)
       ("rdf_atom_no_ns" ?tp ?type)
       ("objectPose" ?o ?pose)
       ("objectDimensions" ?o ?w ?d ?h)
       (= '(?d ?w ?h) ?dim))))))

(defmethod copy-object ((obj semantic-map-object) (world bt-reasoning-world))
  (with-slots (pose geoms) obj
    (change-class (call-next-method) 'semantic-map-object :pose pose :geoms geoms)))

(defmethod add-object ((world bt-world) (type (eql 'semantic-map)) name pose &key (color '(0.8 0.8 0.8 1.0)))
  (let* ((pose-transform (cl-transforms:reference-transform
                          (ensure-pose pose)))
         (geoms (query-semantic-map-geoms pose-transform))
         (map
          (make-instance
           'semantic-map-object
           :world world
           :name name
           :pose (ensure-pose pose)
           :rigid-bodies (mapcar (lambda (obj)
                                   (make-instance
                                    'rigid-body
                                    :name (make-rigid-body-name name (name obj))
                                    :pose (pose obj)
                                    :group :static-filter
                                    :collision-shape (make-instance
                                                      'colored-box-shape
                                                      :half-extents (cl-transforms:v*
                                                                     (dimensions obj)
                                                                     0.5)
                                                      :color color)))
                                 geoms))))
    (dolist (geom geoms)
      (setf (gethash (name geom) (slot-value map 'geoms)) geom))
    map))

(defmethod pose ((obj semantic-map-object))
  (slot-value obj 'pose))
