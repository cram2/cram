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
  normal)

(defstruct 3d-model
  vertices
  faces)

(define-condition 3d-model-import-error (simple-error) ())

(defun ai-vector-3d->3d-vector (ai-vector)
  (cl-transforms:make-3d-vector
   (foreign-slot-value ai-vector 'ai-vector-3d 'x)
   (foreign-slot-value ai-vector 'ai-vector-3d 'y)
   (foreign-slot-value ai-vector 'ai-vector-3d 'z)))

(defun parse-ai-3d-vector-array (foreign-array size)
  (let ((result (make-array
                 size
                 :element-type 'cl-transforms:3d-vector
                 :initial-element (cl-transforms:make-3d-vector 0 0 0))))
    (dotimes (i size result)
      (setf (aref result i)
            (ai-vector-3d->3d-vector
             (mem-aref foreign-array
                       'ai-vector-3d i))))))

(defun get-vertices (mesh)
  (parse-ai-3d-vector-array (foreign-slot-value mesh 'ai-mesh 'vertices)
                            (foreign-slot-value mesh 'ai-mesh 'num-vertices)))

(defun get-normals (mesh)
  (parse-ai-3d-vector-array (foreign-slot-value mesh 'ai-mesh 'normals)
                            (foreign-slot-value mesh 'ai-mesh 'num-faces)))

(defun get-faces (mesh &key
                  (vertices (get-vertices mesh))
                  (normals (get-normals mesh)))
  (let ((result (make-array
                 (foreign-slot-value mesh 'ai-mesh 'num-faces)
                 :element-type 'list
                 :initial-element nil)))
    (dotimes (i (array-dimension result 0) result)
      (let* ((face (mem-aref
                    (foreign-slot-value mesh 'ai-mesh 'faces)
                    'ai-face i))
             (num-indices (foreign-slot-value face 'ai-face 'num-indices)))
        (setf (aref result i)
              (make-face
               :points (loop for pt-index below num-indices
                             for index = (mem-aref
                                          (foreign-slot-value face 'ai-face 'indices)
                                             :unsigned-int pt-index)
                             collecting (aref vertices index))
               :normal (aref normals i)))))))

(defun load-3d-model (filename &optional (mesh-index 0))
  "Loads the mesh with index `mesh-index' from the file named
`filename' and returns an instance of type 3D-MODEL."
  (let ((scene nil))
    (unwind-protect
         (progn
           (setf scene (ai-import-file (etypecase filename
                                         (string filename)
                                         (pathname (namestring filename)))
                                       '(:join-identical-vertices
                                         :gen-smooth-normals :optimize-meshes
                                         :fix-infacing-normals
                                         :triangulate)))
           (when (null-pointer-p scene)
             (error '3d-model-import-error
                    :format-control "Unable to load 3d model from file `~a'"
                    :format-arguments `(,filename)))
           (unless (< mesh-index
                      (foreign-slot-value scene 'ai-scene 'num-meshes))
             (error '3d-model-import-error
                    :format-control "Invalid mesh index `~a'. The file contains only ~a meshes."
                    :format-arguments `(,mesh-index ,(foreign-slot-value scene 'ai-scene 'num-meshes))))
           (let* ((mesh (mem-aref (foreign-slot-value scene 'ai-scene 'meshes)
                                  :pointer mesh-index))
                  (vertices (get-vertices mesh))
                  (faces (get-faces mesh :vertices vertices)))
             (make-3d-model
              :vertices (remove-identical-vertices vertices)
              :faces faces)))
      (when scene
        (ai-release-import scene)))))

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
