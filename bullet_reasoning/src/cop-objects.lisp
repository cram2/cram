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

(in-package :btr)

(defmethod add-object ((world bt-world) (type (eql 'cop-object)) name pose
                       &key object-id)
  (let ((pose (ensure-pose pose)))
    (roslisp:with-fields ((type (type shape))
                          (shape shape))
        (roslisp:call-service "/cop_geometric_shape" 'vision_srvs-srv:cop_get_object_shape
                              :object_id object-id)
      (ecase type
        (0 ;; Sphere
         (roslisp:with-fields (dimensions) shape
           (add-object world 'sphere name pose
                       :mass (physics-utils:calculate-mass
                              :sphere :radius (/ (elt dimensions 0) 2))
                       :radius (/ (elt dimensions 0) 2))))
        (1 ;; Box
         (roslisp:with-fields (dimensions) shape
           (add-object world 'box name pose
                       :mass (physics-utils:calculate-mass
                              :box
                              :size-x (elt dimensions 0)
                              :size-y (elt dimensions 1)
                              :size-z (elt dimensions 2))
                       :size (cl-transforms:make-3d-vector
                              (elt dimensions 0)
                              (elt dimensions 1)
                              (elt dimensions 2)))))
        (2 ;; Cylinder
         (roslisp:with-fields (dimensions) shape
           (add-object world 'cylinder name pose
                       :mass (physics-utils:calculate-mass
                              :cylinder
                              :radius (/ (elt dimensions 0) 2)
                              :height (elt dimensions 2))
                       :size (cl-transforms:make-3d-vector
                              (elt dimensions 0)
                              (elt dimensions 1)
                              (elt dimensions 2)))))
        (3 ;; Mesh
         (let ((mesh (physics-utils:shape-msg->mesh shape)))
           (add-object world 'mesh name pose
                       :mass (physics-utils:calculate-mass
                              :mesh :points (physics-utils:3d-model-vertices mesh))
                       :mesh mesh)))
        (4 ;; Point cloud (inofficial)
           (roslisp:with-fields (mesh)
               (roslisp:call-service
                "/triangulate_point_cloud/triangulate"
                "triangulate_point_cloud/TriangulatePCL"
                :points (physics-utils:points->point-cloud
                         (cl-transforms:make-identity-pose)
                         (physics-utils:shape-msg->points shape)))
             (add-object world 'mesh name pose
                         :mass 0.0
                         :mesh (physics-utils:shape-msg->mesh mesh)
                         :disable-face-culling t))
         ;; (make-object
         ;;  world name
         ;;  (list
         ;;   (make-instance 'rigid-body
         ;;     :name name
         ;;     :mass 0.0 ;; We cannot simulate dynamics for point clouds,
         ;;     ;; so we make them static objects
         ;;     :collision-shape (make-instance 'convex-hull-shape
         ;;                        :points (physics-utils:shape-msg->points shape)))))
           )))))
