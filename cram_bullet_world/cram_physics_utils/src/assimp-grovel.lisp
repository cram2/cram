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

(include "assimp/cimport.h")
(include "assimp/mesh.h")
(include "assimp/scene.h")
(include "assimp/material.h")
(include "assimp/postprocess.h")

(in-package :physics-utils)

(cenum ; bitfield
 ai-post-process-steps
 ((:calc-tangent-space "aiProcess_CalcTangentSpace"))
 ((:join-identical-vertices "aiProcess_JoinIdenticalVertices"))
 ((:make-left-handed "aiProcess_MakeLeftHanded"))
 ((:triangulate "aiProcess_Triangulate"))
 ((:remove-component "aiProcess_RemoveComponent"))
 ((:gen-normals "aiProcess_GenNormals"))
 ((:gen-smooth-normals "aiProcess_GenSmoothNormals"))
 ((:split-large-meshes "aiProcess_SplitLargeMeshes"))
 ((:pre-transform-vertices "aiProcess_PreTransformVertices"))
 ((:limit-bone-weights "aiProcess_LimitBoneWeights"))
 ((:validate-data-structure "aiProcess_ValidateDataStructure"))
 ((:improve-cache-locality "aiProcess_ImproveCacheLocality"))
 ((:remove-redundant-matrerials "aiProcess_RemoveRedundantMaterials"))
 ((:fix-infacing-normals "aiProcess_FixInfacingNormals"))
 ((:sort-by-p-type "aiProcess_SortByPType"))
 ((:find-degenerates "aiProcess_FindDegenerates"))
 ((:find-invalid-data "aiProcess_FindInvalidData"))
 ((:gen-uv-coords "aiProcess_GenUVCoords"))
 ((:find-instances "aiProcess_FindInstances"))
 ((:optimize-meshes "aiProcess_OptimizeMeshes"))
 ((:optimize-graph "aiProcess_OptimizeGraph"))
 ((:flip-uvs "aiProcess_FlipUVs"))
 ((:flip-winding-order "aiProcess_FlipWindingOrder")))

(cenum ; bitfield
 ai-primitive-type
 ((:point "aiPrimitiveType_POINT"))
 ((:line "aiPrimitiveType_LINE"))
 ((:triangle "aiPrimitiveType_TRIANGLE"))
 ((:polygon "aiPrimitiveType_POLYGON")))

(cenum
 ai-texture-op
 ((:multiply "aiTextureOp_Multiply"))
 ((:add "aiTextureOp_Add"))
 ((:substract "aiTextureOp_Subtract"))
 ((:divide "aiTextureOp_Divide"))
 ((:smooth-add "aiTextureOp_SmoothAdd"))
 ((:signed-add "aiTextureOp_SignedAdd")))

(cenum
 ai-texture-map-mode
 ((:wrap "aiTextureMapMode_Wrap"))
 ((:clamp "aiTextureMapMode_Clamp"))
 ((:decal "aiTextureMapMode_Decal"))
 ((:mirror "aiTextureMapMode_Mirror")))

(cenum
 ai-texture-mapping
 ((:uv "aiTextureMapping_UV"))
 ((:sphere "aiTextureMapping_SPHERE"))
 ((:cyliner "aiTextureMapping_CYLINDER"))
 ((:box "aiTextureMapping_BOX"))
 ((:plane "aiTextureMapping_PLANE"))
 ((:other "aiTextureMapping_OTHER")))

(cenum
 ai-texture-type
 ((:none "aiTextureType_NONE"))
 ((:diffuse "aiTextureType_DIFFUSE"))
 ((:specular "aiTextureType_SPECULAR"))
 ((:ambient "aiTextureType_AMBIENT"))
 ((:emissive "aiTextureType_EMISSIVE"))
 ((:height "aiTextureType_HEIGHT"))
 ((:normals "aiTextureType_NORMALS"))
 ((:shininess "aiTextureType_SHININESS"))
 ((:opacity "aiTextureType_OPACITY"))
 ((:displacement "aiTextureType_DISPLACEMENT"))
 ((:lightmap "aiTextureType_LIGHTMAP"))
 ((:reflection "aiTextureType_REFLECTION"))
 ((:unknown "aiTextureType_UNKNOWN")))

(cenum
 ai-property-type-info
 ((:float "aiPTI_Float"))
 ((:string "aiPTI_String"))
 ((:integer "aiPTI_Integer"))
 ((:buffer "aiPTI_Buffer")))

(cstruct
 ai-vector-3d "aiVector3D"
 (x "x" :type :float)
 (y "y" :type :float)
 (z "z" :type :float))

(cstruct
 ai-matrix-4x4 "aiMatrix4x4"
 (a1 "a1" :type :float)
 (a2 "a2" :type :float)
 (a3 "a3" :type :float)
 (a4 "a4" :type :float)
 (b1 "b1" :type :float)
 (b2 "b2" :type :float)
 (b3 "b3" :type :float)
 (b4 "b4" :type :float)
 (c1 "c1" :type :float)
 (c2 "c2" :type :float)
 (c3 "c3" :type :float)
 (c4 "c4" :type :float)
 (d1 "d1" :type :float)
 (d2 "d2" :type :float)
 (d3 "d3" :type :float)
 (d4 "d4" :type :float))

(cstruct
 ai-string "aiString"
  (length "length" :type :unsigned-int)
  (data "data" :type :char :count 1024))

(cstruct
 ai-material-property "aiMaterialProperty"
 (key "mKey" :type (:struct ai-string))
 (semantic "mSemantic" :type ai-texture-type)
 (index "mIndex" :type :unsigned-int)
 (data-length "mDataLength" :type :unsigned-int)
 (type "mType" :type ai-property-type-info)
 (data "mData" :type :pointer))

(cstruct
 ai-material "aiMaterial"
 (properties "mProperties" :type :pointer)
 (num-properties "mNumProperties" :type :unsigned-int)
 (num-allocated "mNumAllocated" :type :unsigned-int))

(cstruct
 ai-texel "aiTexel"
 (b "b" :type :unsigned-char)
 (g "g" :type :unsigned-char)
 (r "r" :type :unsigned-char)
 (a "a" :type :unsigned-char))

(cstruct
 ai-texture "aiTexture"
  (width "mWidth" :type :unsigned-int)
  (height "mHeight" :type :unsigned-int)
  (ach-format-hint "achFormatHint" :type :char :count 4)
  (data "pcData" :type :pointer))

(cstruct
 ai-face "aiFace"
  (num-indices "mNumIndices" :type :unsigned-int)
  (indices "mIndices" :type :pointer))

(cstruct
 ai-mesh "aiMesh"
 (primitive-types "mPrimitiveTypes" :type ai-primitive-type)
 (num-vertices "mNumVertices" :type :unsigned-int)
 (num-faces "mNumFaces" :type :unsigned-int)
 (vertices "mVertices" :type :pointer)
 (normals "mNormals" :type :pointer)
 (tangents "mTangents" :type :pointer)
 (bitangents "mBitangents" :type :pointer)
 (colors "mColors" :type :pointer)
 (texture-coords "mTextureCoords" :type :pointer)
 (num-uv-components "mNumUVComponents" :type :unsigned-int)
 (faces "mFaces" :type :pointer)
 (num-bones "mNumBones" :type :unsigned-int)
 (bones "mBones" :type :pointer)
 (material-index "mMaterialIndex" :type :unsigned-int)
 (name "mName" :type (:struct ai-string)))

(cstruct
 ai-node "aiNode"
 (name "mName" :type (:struct ai-string))
 (transform "mTransformation" :type (:struct ai-matrix-4x4))
 (parent "mParent" :type :pointer)
 (num-children "mNumChildren" :type :unsigned-int)
 (children "mChildren" :type :pointer)
 (num-meshes "mNumMeshes" :type :unsigned-int)
 (meshes "mMeshes" :type :pointer))

(cstruct
 ai-scene "aiScene"
 (flags "mFlags" :type :unsigned-int)
 (root-node "mRootNode" :type :pointer)
 (num-meshes "mNumMeshes" :type :unsigned-int)
 (meshes "mMeshes" :type :pointer)
 (num-materials "mNumMaterials" :type :unsigned-int)
 (materials "mMaterials" :type :pointer)
 (num-animations "mNumAnimations" :type :unsigned-int)
 (animations "mAnimations" :type :pointer)
 (num-textures "mNumTextures" :type :unsigned-int)
 (textures "mTextures" :type :pointer)
 (num-lights "mNumLights" :type :unsigned-int)
 (lights "mLights" :type :pointer)
 (num-cameras "mNumCameras" :type :unsigned-int)
 (cameras "mCameras" :type :pointer))
