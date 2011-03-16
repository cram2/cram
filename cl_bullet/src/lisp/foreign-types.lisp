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

(in-package :cl-bullet)

(defcenum activation-state
  (:active-tag 1)
  (:island-sleeping 2)
  (:wants-deactivation 3)
  (:disable-deactivation 4)
  (:disable-simulation 5))

(defbitfield collision-flags
  (:cf-default 0)
  (:cf-static-object 1)
  (:cf-kinematic-object 2)
  (:cf-no-contact-response 4)
  (:cf-custom-material-callback 8)
  (:cf-character-object 16)
  (:cf-disable-visualize-object 32))

(defbitfield (collision-filters :ushort)
  (:no-filter        #x0000)
  (:default-filter   #x0001)
  (:static-filter    #x0002)
  (:kinematic-filter #x0004)
  (:debris-filter    #x0008)
  (:sensor-trigger   #x0010)
  (:character-filter #x0020)
  (:user-filter-1    #x0040)
  (:user-filter-2    #x0080)
  (:user-filter-3    #x0100)
  (:user-filter-4    #x0200)
  (:user-filter-5    #x0400)
  (:user-filter-6    #x0800)
  (:user-filter-7    #x1000)
  (:user-filter-8    #x2000)
  (:user-filter-9    #x4000)
  (:user-filter-10   #x8000)
  (:all-filter       #xffff))

(defcenum debug-modes
  (:dbg-no-debug 0)
  (:dbg-draw-wireframe 1)
  (:dbg-draw-aabb 2)
  (:dbg-draw-features-text 4)
  (:dbg-draw-contanct-points 8)
  (:dbg-no-deactivation 16)
  (:dbg-no-help-text 32)
  (:dbg-draw-text 64)
  (:dbg-profile-timings 128)
  (:dbg-enable-stat-comparison 256)
  (:dbg-disable-bullet-lcp 512)
  (:dbg-enable-ccd 1024)
  (:dbg-draw-constraints 2048)
  (:dbg-draw-constraint-limits 4096)
  (:dbg-fast-wireframe 8192)
  (:dbg-max-debug-draw-mode 8193))

(defcenum broadphase-native-type
  :box-shape-proxytype
  :triangle-shape-proxytype
  :tetrahedral-shape-proxytype
  :convex-trianglemesh-shape-proxytype
  :convex-hull-shape-proxytype
  :convex-point-cloud-shape-proxytype
  :custom-polyhedral-shape-type
  :implicit-convex-shapes-start-here
  :sphere-shape-proxytype
  :multi-sphere-shape-proxytype
  :capsule-shape-proxytype
  :cone-shape-proxytype
  :convex-shape-proxytype
  :cylinder-shape-proxytype
  :uniform-scaling-shape-proxytype
  :minkowski-sum-shape-proxytype
  :minkowski-difference-shape-proxytype
  :box-2d-shape-proxytype
  :convex-2d-shape-tproxyype
  :custom-convex-shape-type
  :concave-shapes-start-here
  :triangle-mesh-shape-proxytype
  :scaled-triangle-mesh-shape-proxytype
  :fast-concave-mesh-proxytype
  :terrain-shape-proxytype
  :gimpact-shape-proxytype
  :multimaterial-triangle-mesh-proxytype
  :empty-shape-proxytype
  :static-plane-proxytype
  :custom-concave-shape-type
  :concave-shapes-end-here
  :compound-shape-proxytype
  :softbody-shape-proxytype
  :hffluid-shape-proxytype
  :hffluid-buoyant-convex-shape-proxytype
  :invalid-shape-proxytype
  :max-broadphase-collision-types)

(defcstruct debug-draw-callbacks
  (draw-line :pointer)
  (draw-sphere :pointer)
  (draw-triangle :pointer)
  (draw-box :pointer)
  (draw-aabb :pointer)
  (draw-transform :pointer)
  (draw-arc :pointer)
  (draw-sphere-patch :pointer)
  (draw-contact-point :pointer)
  (report-error-warning :pointer)
  (draw-3d-text :pointer))

(define-foreign-type bt-3d-vector ()
  () (:actual-type :pointer :double))

(define-parse-method bt-3d-vector (&key)
  (make-instance 'bt-3d-vector))

(defmethod translate-to-foreign ((value cl-transforms:3d-vector) (type bt-3d-vector))
  (let ((native-vector (foreign-alloc :double :count 3)))
    (setf (mem-aref native-vector :double 0) (coerce (cl-transforms:x value) 'double-float))
    (setf (mem-aref native-vector :double 1) (coerce (cl-transforms:y value) 'double-float))
    (setf (mem-aref native-vector :double 2) (coerce (cl-transforms:z value) 'double-float))
    native-vector))

(defmethod translate-from-foreign (pointer (type bt-3d-vector))
  (cl-transforms:make-3d-vector
   (mem-aref pointer :double 0)
   (mem-aref pointer :double 1)
   (mem-aref pointer :double 2)))

(defmethod free-translated-object (value (type bt-3d-vector) param)
  (declare (ignore param))
  (foreign-free value))

(define-foreign-type bt-quaternion ()
  () (:actual-type :pointer :double))

(define-parse-method bt-quaternion (&key)
  (make-instance 'bt-quaternion))

(defmethod translate-to-foreign ((value cl-transforms:quaternion) (type bt-quaternion))
  (let ((native-vector (foreign-alloc :double :count 4)))
    (setf (mem-aref native-vector :double 0) (coerce (cl-transforms:x value) 'double-float))
    (setf (mem-aref native-vector :double 1) (coerce (cl-transforms:y value) 'double-float))
    (setf (mem-aref native-vector :double 2) (coerce (cl-transforms:z value) 'double-float))
    (setf (mem-aref native-vector :double 3) (coerce (cl-transforms:w value) 'double-float))
    native-vector))

(defmethod translate-from-foreign (pointer (type bt-quaternion))
  (cl-transforms:make-quaternion
   (mem-aref pointer :double 0)
   (mem-aref pointer :double 1)
   (mem-aref pointer :double 2)
   (mem-aref pointer :double 3)))

(defmethod free-translated-object (value (type bt-quaternion) param)
  (declare (ignore param))
  (foreign-free value))

(define-foreign-type bt-transform ()
  () (:actual-type :pointer :double))

(define-parse-method bt-transform (&key)
  (make-instance 'bt-transform))

(defmethod translate-to-foreign ((value cl-transforms:transform) (type bt-transform))
  (let ((native-vector (foreign-alloc :double :count 7))
        (translation (cl-transforms:translation value))
        (rotation (cl-transforms:rotation value)))
    (setf (mem-aref native-vector :double 0) (coerce (cl-transforms:x translation) 'double-float))
    (setf (mem-aref native-vector :double 1) (coerce (cl-transforms:y translation) 'double-float))
    (setf (mem-aref native-vector :double 2) (coerce (cl-transforms:z translation) 'double-float))
    (setf (mem-aref native-vector :double 3) (coerce (cl-transforms:x rotation) 'double-float))
    (setf (mem-aref native-vector :double 4) (coerce (cl-transforms:y rotation) 'double-float))
    (setf (mem-aref native-vector :double 5) (coerce (cl-transforms:z rotation) 'double-float))
    (setf (mem-aref native-vector :double 6) (coerce (cl-transforms:w rotation) 'double-float))
    native-vector))

(defmethod translate-to-foreign ((value cl-transforms:pose) (type bt-transform))
  (let ((native-vector (foreign-alloc :double :count 7))
        (translation (cl-transforms:origin value))
        (rotation (cl-transforms:orientation value)))
    (setf (mem-aref native-vector :double 0) (coerce (cl-transforms:x translation) 'double-float))
    (setf (mem-aref native-vector :double 1) (coerce (cl-transforms:y translation) 'double-float))
    (setf (mem-aref native-vector :double 2) (coerce (cl-transforms:z translation) 'double-float))
    (setf (mem-aref native-vector :double 3) (coerce (cl-transforms:x rotation) 'double-float))
    (setf (mem-aref native-vector :double 4) (coerce (cl-transforms:y rotation) 'double-float))
    (setf (mem-aref native-vector :double 5) (coerce (cl-transforms:z rotation) 'double-float))
    (setf (mem-aref native-vector :double 6) (coerce (cl-transforms:w rotation) 'double-float))
    native-vector))

(defmethod translate-from-foreign (pointer (type bt-transform))
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector
    (mem-aref pointer :double 0)
    (mem-aref pointer :double 1)
    (mem-aref pointer :double 2))
   (cl-transforms:make-quaternion
    (mem-aref pointer :double 3)
    (mem-aref pointer :double 4)
    (mem-aref pointer :double 5)
    (mem-aref pointer :double 6))))

(defmethod free-translated-object (value (type bt-transform) param)
  (declare (ignore param))
  (foreign-free value))