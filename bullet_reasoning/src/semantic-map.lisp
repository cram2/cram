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
  ((geoms :initarg :geoms :initform (make-hash-table :test #'equal))))

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

(defclass semantic-map-part ()
  ((type :initarg :type :reader obj-type)
   (name :initarg :name :reader name)
   (urdf-name :initarg :urdf-link-name :reader urdf-name)
   (sub-parts :reader sub-parts)))

(defclass semantic-map-geom (semantic-map-part)
  ((pose :initarg :pose :reader pose)
   (dimensions :initarg :dimensions :reader dimensions)))

(defgeneric make-semantic-map-part (type name)
  (:method ((type t) name)
    (with-vars-bound (?pose ?dim)
        (lazy-car
         (json-prolog:prolog
          `(and ("objectPose" ,name ?pose)
                ("objectDimensions" ,name ?w ?d ?h)
                (= '(?d ?w ?h) ?dim))))
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
           (lazy-mapcar
            (lambda (bdgs)
              (with-vars-bound (?type ?sub) bdgs
                (make-semantic-map-part
                 ?type
                 (remove #\' (symbol-name ?sub)))))
            (json-prolog:prolog
             `(and
               ("rdf_has" ,(name part) "http://ias.cs.tum.edu/kb/knowrob.owl#properPhysicalParts" ?sub)
               ("objectType" ?sub ?tp)
               ("rdf_atom_no_ns" ?tp ?type))))))))

(defmethod urdf-name :before ((geom semantic-map-part))
  (unless (slot-boundp geom 'urdf-name)
    (with-slots (name urdf-name) geom
      (let ((label (var-value
                    '?link
                    (lazy-car (json-prolog:prolog
                               `("rdf_has"
                                 ,name "http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName"
                                 ("literal" ?link)))))))
        (unless (is-var label)
          (setf urdf-name (remove #\' (symbol-name label))))))))

(defun query-semantic-map ()
  (force-ll
   (lazy-mapcar
    (lambda (bdgs)
      (with-vars-bound (?type ?o) bdgs
        (make-semantic-map-part ?type (remove #\' (symbol-name ?o)))))
    (json-prolog:prolog
     '(and ("rootObjects" ?objs)
       ("member" ?o ?objs)
       ("objectType" ?o ?tp)
       ("rdf_atom_no_ns" ?tp ?type))))))

(defmethod copy-object ((obj semantic-map-object) (world bt-reasoning-world))
  (with-slots (pose geoms) obj
    (change-class (call-next-method) 'semantic-map-object :geoms geoms)))

(defmethod add-object ((world bt-world) (type (eql 'semantic-map)) name pose &key urdf)
  (let* ((geoms (query-semantic-map))
         (map (change-class (add-object world 'urdf name pose :urdf urdf) 'semantic-map-object)))
    (dolist (geom geoms)
      (setf (gethash (name geom) (slot-value map 'geoms)) geom))
    map))

