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

(defstruct model
  vertices
  triangles)

(define-foreign-ros-library assimp "libassimp.so")

(use-foreign-library assimp)

(defconstant +max-number-of-color-sets+ 4)
(defconstant +max-number-of-texture-coords+ 4)

(defbitfield ai-post-process-steps
  (:calc-tangent-space #x01)
  (:join-identical-vertices #x02)
  (:make-left-handed #x04)
  (:triangulate #x08)
  (:remove-component #x10)
  (:gen-normals #x20)
  (:gen-smooth-normals #x40)
  (:split-large-meshes #x80)
  (:pre-transform-vertices #x100)
  (:limit-bone-weights #x200)
  (:validate-data-structure #x400)
  (:improve-cache-locality #x800)
  (:remove-redundant-matrerials #x1000)
  (:fix-infacing-normals #x2000)
  (:sort-by-p-type #x8000)
  (:find-degenerates #x10000)
  (:find-invalid-data #x20000)
  (:gen-uv-coords #x40000)
  (:find-instances #x100000)
  (:optimize-meshes #x200000)
  (:optimize-graph #x400000)
  (:flip-uvs #x800000)
  (:flip-winding-order #x1000000))

(defbitfield ai-primitive-type
  (:point #x01)
  (:line #x02)
  (:triangle #x04)
  (:polygon #x08))

(defcstruct ai-vector-3d
  (x :float)
  (y :float)
  (z :float))

(defcstruct ai-face
  (num-indices :unsigned-int)
  (indices :pointer))

(defcstruct ai-mesh
  (primitive-types ai-primitive-type)
  (num-vertices :unsigned-int)
  (num-faces :unsigned-int)
  (vertices :pointer)
  (normals :pointer)
  (tangents :pointer)
  (bitangents :pointer)
  (colors :pointer :count 4)
  (texture-coords :pointer :count 4)
  (num-uv-components :unsigned-int :count 4)
  (faces :pointer)
  (num-bones :unsigned-int)
  (bones :pointer)
  (material-index :unsigned-int))

(defcstruct ai-scene
  (flags :unsigned-int)
  ;; For now, we are too lazy to implement the whole interface. We
  ;; don't care about nodes yet so we just ignore them.
  (root-node :pointer)
  (num-meshes :unsigned-int)
  (meshes :pointer)
  (num-materials :unsigned-int)
  (materials :pointer)
  (num-animations :unsigned-int)
  (animations :pointer)
  (num-textures :unsigned-int)
  (textures :pointer)
  (num-lights :unsigned-int)
  (lights :pointer)
  (num-cameras :unsigned-int)
  (cameras :pointer))

(defcfun ("aiImportFile" ai-import-file) ai-scene
  (file :string)
  (pflags ai-post-process-steps))

(defcfun ("aiReleaseImport" ai-release-import) :void
  (scene ai-scene))

(defcfun ("aiGetErrorString" ai-get-error-string) :string)
