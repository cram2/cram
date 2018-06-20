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

(in-package :physics-utils)

(defstruct face
  points
  normals)

(defstruct 3d-model
  vertices
  faces)

(defstruct compound-3d-model
  (models '() :type list))


(define-condition 3d-model-import-error (simple-error) ())

(defun ai-vector-3d->3d-vector (ai-vector-plist)
  (cl-transforms:make-3d-vector
   (getf ai-vector-plist 'x)
   (getf ai-vector-plist 'y)
   (getf ai-vector-plist 'z)
   ;; (foreign-slot-value ai-vector '(:struct ai-vector-3d) 'x)
   ;; (foreign-slot-value ai-vector '(:struct ai-vector-3d) 'y)
   ;; (foreign-slot-value ai-vector '(:struct ai-vector-3d) 'z)
   ))

(defun parse-ai-3d-vector-array (foreign-array size)
  (let ((result (make-array
                 size
                 :element-type 'cl-transforms:3d-vector
                 :initial-element (cl-transforms:make-3d-vector 0 0 0))))
    (dotimes (i size result)
      (setf (aref result i)
            (ai-vector-3d->3d-vector
             (mem-aref foreign-array '(:struct ai-vector-3d) i))))))

(defun ai-matrix4x4->transform (ai-matrix-plist)
  (cl-transforms:matrix->transform
   (make-array
    '(4 4) :element-type 'double-float
           :initial-contents
           `((,(float (getf ai-matrix-plist 'a1)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'a1)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'a2)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'a2)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'a3)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'a3)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'a4)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'a4)
                      0.0d0))
             (,(float (getf ai-matrix-plist 'b1)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'b1)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'b2)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'b2)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'b3)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'b3)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'b4)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'b4)
                      0.0d0))
             (,(float (getf ai-matrix-plist 'c1)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'c1)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'c2)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'c2)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'c3)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'c3)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'c4)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'c4)
                      0.0d0))
             (,(float (getf ai-matrix-plist 'd1)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'd1)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'd2)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'd2)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'd3)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'd3)
                      0.0d0)
              ,(float (getf ai-matrix-plist 'd4)
                      ;; (foreign-slot-value ai-matrix '(:struct ai-matrix-4x4) 'd4)
                      0.0d0))))))

(defun get-vertices (mesh)
  (parse-ai-3d-vector-array (foreign-slot-value mesh '(:struct ai-mesh) 'vertices)
                            (foreign-slot-value mesh '(:struct ai-mesh) 'num-vertices)))

(defun get-normals (mesh)
  (parse-ai-3d-vector-array (foreign-slot-value mesh '(:struct ai-mesh) 'normals)
                            (foreign-slot-value mesh '(:struct ai-mesh) 'num-vertices)))

(defun vertex-accessor-function (vertices)
  (lambda (index)
    (aref vertices index)))

(defun get-faces (mesh &key (post-process-vertex-index
                             (vertex-accessor-function (get-vertices mesh)))
                         (normals (get-normals mesh)))
  (let ((result (make-array
                 (foreign-slot-value mesh '(:struct ai-mesh) 'num-faces)
                 :element-type 'list
                 :initial-element nil)))
    (assert (not (null-pointer-p (foreign-slot-value mesh '(:struct ai-mesh) 'faces)))
            ()
            "Faces unbound")
    (dotimes (i (array-dimension result 0) result)
      (let* ((face-plist (mem-aref
                          (foreign-slot-value mesh '(:struct ai-mesh) 'faces)
                          '(:struct ai-face) i))
             (num-indices (getf face-plist 'num-indices)
                          ;; (foreign-slot-value face '(:struct ai-face) 'num-indices)
                          ))
        (setf (aref result i)
              (apply
               #'make-face
               (loop for pt-index below num-indices
                     for index = (mem-aref
                                  (getf face-plist 'indices)
                                  ;; (foreign-slot-value face '(:struct ai-face) 'indices)
                                  :unsigned-int pt-index)
                     collect (funcall post-process-vertex-index index) into points
                     collect (aref normals index) into vertex-normals
                     finally (return (list :points points :normals vertex-normals)))))))))

(defun get-mesh (ai-mesh &key post-process-vertex-index)
  (if (equal (foreign-slot-value ai-mesh '(:struct ai-mesh) 'primitive-types)
             :triangle)
      (if post-process-vertex-index
          (make-3d-model
           :vertices (get-vertices ai-mesh)
           :faces (get-faces ai-mesh :post-process-vertex-index post-process-vertex-index))
          (make-3d-model
           :vertices (get-vertices ai-mesh)
           :faces (get-faces ai-mesh)))
      (warn "Non-triangle meshes are not supported by cram-physics-utils...")))

(defun get-meshes (ai-scene &key post-process-vertex-index)
  (let ((result (make-array (foreign-slot-value ai-scene '(:struct ai-scene) 'num-meshes)))
        (meshes (foreign-slot-value ai-scene '(:struct ai-scene) 'meshes)))
    (dotimes (i (array-dimension result 0) result)
      (setf (aref result i)
            (if post-process-vertex-index
                (get-mesh (mem-aref meshes :pointer i)
                          :post-process-vertex-index post-process-vertex-index)
                (get-mesh (mem-aref meshes :pointer i)))))))

(defun convert-post-process-enums-to-bitfield (enums-list)
  (reduce #'logior
          (mapcar (lambda (enum)
                    (cffi:convert-to-foreign enum 'ai-post-process-steps))
                  enums-list)))

(defun load-3d-model (filename &key
                                 flip-winding-order
                                 (remove-identical-vertices t) (fix-normals t) (compound nil))
  "Loads the mesh with index `mesh-index' from the file named
  `filename' and returns an instance of type 3D-MODEL. If `compound' is T, the single
  meshes are returned as multiple-values-list."
  (let ((scene nil))
    (unwind-protect
         (progn
           (setf scene (ai-import-file (etypecase filename
                                         (string filename)
                                         (pathname (namestring filename)))
                                       (convert-post-process-enums-to-bitfield
                                        `(:join-identical-vertices
                                          :gen-smooth-normals
                                          :fix-infacing-normals
                                          :triangulate
                                          ,@(when flip-winding-order
                                              (list :flip-winding-order))))))
           (when (null-pointer-p scene)
             (error '3d-model-import-error
                    :format-control "Unable to load 3d model from file `~a': ~a"
                    :format-arguments (list filename (ai-get-error-string))))
           (unless (> (foreign-slot-value scene '(:struct ai-scene) 'num-meshes) 0)
             (error '3d-model-import-error
                    :format-control "The file does not contain any meshes."))
           (let ((meshes (multiple-value-list
                          (build-model-mesh scene :fix-normals fix-normals :compound compound))))
             (values-list
              (if remove-identical-vertices
                  (mapcar
                   (lambda (mesh) (make-3d-model
                                   :vertices (remove-identical-vertices (3d-model-vertices mesh))
                                   :faces (3d-model-faces mesh))) meshes)
                  meshes))))
      (when scene
        (ai-release-import scene)))))

(defun build-model-mesh (scene &key (fix-normals t) (compound nil))
  "Recursively traverses all scene nodes and builds a single mesh file
  from it, unless `compound' is T, in which case they are returned as
  values-list of 3d-models."
  (labels ((insert-node (ai-node meshes parent-transformation 3d-model)
             (let ((node-transformation
                     ;; Assimp's root node transformation might do
                     ;; stupid things, for instance transforming the
                     ;; mesh to y-up. Ignore the node transformation
                     ;; if we are processing the root node.
                     (if (null-pointer-p
                          (foreign-slot-value ai-node '(:struct ai-node) 'parent))
                         (cl-transforms:make-identity-transform)
                         (cl-transforms:transform*
                          parent-transformation
                          (ai-matrix4x4->transform
                           (foreign-slot-value ai-node '(:struct ai-node)
                                               'transform))))))
               (if compound
                   (setf 3d-model
                         (make-compound-3d-model
                          :models
                          (map 'list #'identity
                               (get-meshes scene :post-process-vertex-index #'identity))))
                   (progn (dotimes (i (foreign-slot-value ai-node '(:struct ai-node)
                                                          'num-meshes))
                            (let ((mesh (aref meshes
                                              (mem-aref
                                               (foreign-slot-value ai-node '(:struct ai-node) 'meshes)
                                               :unsigned-int i))))
                              (when mesh
                                (setf 3d-model (insert-mesh mesh 3d-model node-transformation)))))
                          (dotimes (i (foreign-slot-value ai-node '(:struct ai-node)
                                                          'num-children))
                            (setf 3d-model
                                  (insert-node
                                   (mem-aref (foreign-slot-value ai-node '(:struct ai-node) 'children)
                                             :pointer i)
                                   meshes node-transformation 3d-model))))))
             3d-model))
    (let ((result (insert-node
                   (foreign-slot-value scene '(:struct ai-scene) 'root-node)
                   (get-meshes scene :post-process-vertex-index #'identity)
                   (cl-transforms:make-identity-transform)
                   (if compound
                       (make-compound-3d-model)
                       (make-3d-model :vertices #() :faces #())))))
      (flet ((make-faces (vertices-model)
               (make-3d-model
                :vertices (3d-model-vertices vertices-model)
                :faces (map 'vector
                            (lambda (face)
                              (let ((new-face
                                      (make-face
                                       :points (mapcar (lambda (index)
                                                         (aref (3d-model-vertices vertices-model)
                                                               index))
                                                       (face-points face))
                                       :normals (face-normals face))))
                                (if fix-normals
                                    (fix-face-normals new-face)
                                    face)))
                            (3d-model-faces vertices-model)))))
        (if compound
            (values-list (mapcar #'make-faces (compound-3d-model-models result)))
            (make-faces result))))))

(defun insert-mesh (3d-model destination-3d-model transform)
  "Inserts `3d-model' into `destination-3d-model' at `transform'. All
  points of the two 3d models are merged and all points in the source
  model are transformed by the specified transform."
  (declare (type 3d-model 3d-model destination-3d-model)
           (type cl-transforms:transform transform))
  (flet ((transform-point (point)
           (declare (type cl-transforms:3d-vector point))
           (cl-transforms:transform-point transform point))
         (rotate-vector (vector)
           (declare (type cl-transforms:3d-vector vector))
           (cl-transforms:rotate
            (cl-transforms:rotation transform) vector))
         (reindex-face (face offset)
           (make-face
            :points (mapcar (lambda (n) (+ n offset)) (face-points face))
            :normals (face-normals face))))
    (let ((original-vertices-size (array-dimension
                                   (3d-model-vertices destination-3d-model)
                                   0)))
      (make-3d-model
       :vertices (concatenate
                  'vector (3d-model-vertices destination-3d-model)
                  (map 'vector #'transform-point (3d-model-vertices 3d-model)))
       :faces (concatenate
               'vector (3d-model-faces destination-3d-model)
               (map 'vector
                    (lambda (face)
                      (let ((reindexed-face (reindex-face face original-vertices-size)))
                        (make-face
                         :points (face-points reindexed-face)
                         :normals (mapcar #'rotate-vector (face-normals reindexed-face)))))
                    (3d-model-faces 3d-model)))))))

(defun remove-identical-vertices (vertex-array)
  (flet ((find-point-in-array (pt pts n &optional (epsilon 0.0001))
           (dotimes (i n nil)
             (when (and (< (abs (- (cl-transforms:x pt)
                                   (cl-transforms:x (aref pts i))))
                           epsilon)
                        (< (abs (- (cl-transforms:y pt)
                                   (cl-transforms:y (aref pts i))))
                           epsilon)
                        (< (abs (- (cl-transforms:z pt)
                                   (cl-transforms:z (aref pts i))))
                           epsilon))
               (return t)))))
    (let ((result-array (make-array (array-dimension vertex-array 0)
                                    :element-type (array-element-type vertex-array)
                                    :initial-element (cl-transforms:make-3d-vector 0 0 0)))
          (result-index 0))
      (dotimes (i (array-dimension vertex-array 0))
        (let ((pt (aref vertex-array i)))
          (unless (find-point-in-array pt result-array result-index)
            (setf (aref result-array result-index) pt)
            (incf result-index))))
      (adjust-array result-array result-index))))

(defun fix-normals (faces &key always-recalculate)
  (map 'vector
       (lambda (face)
         (fix-face-normals face :always-recalculate always-recalculate))
       faces))

(defun fix-face-normals (face &key always-recalculate)
  (let* ((normal (cl-transforms:cross-product
                  (cl-transforms:v- (second (face-points face))
                                    (first (face-points face)))
                  (cl-transforms:v- (third (face-points face))
                                    (first (face-points face)))))
         (normal-norm (cl-transforms:v-norm normal)))
    (make-face
     :points (face-points face)
     :normals (cond (always-recalculate
                     (loop for point in (face-points face)
                           collecting (cl-transforms:v* normal (/ normal-norm))))
                    (t
                     (loop for n in (face-normals face)
                           when (< (cl-transforms:v-norm n) 1)
                             collecting (if (> normal-norm 0)
                                            (cl-transforms:v* normal (/ normal-norm))
                                            normal)
                           else collecting n))))))

(defun merge-3d-models (models &key remove-identical-vertices)
  (let ((merged-vertices (apply #'concatenate 'vector
                                (mapcar #'3d-model-vertices models)))
        (merged-faces (apply #'concatenate 'vector
                             (mapcar #'3d-model-faces models))))
    (make-3d-model
     :vertices (if remove-identical-vertices

                   (remove-identical-vertices merged-vertices)
                   merged-vertices)
     :faces merged-faces)))
