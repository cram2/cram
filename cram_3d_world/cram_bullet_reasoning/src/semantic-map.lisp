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
  ((semantic-map :reader semantic-map
                 :initform (sem-map-utils:get-semantic-map)
                 :initarg :semantic-map)))

(defgeneric semantic-map-part-pose (semantic-map part-name)
  (:documentation "Returns the pose of the part `part-name'.")
  (:method ((semantic-map semantic-map-object) part-name)
    (let ((part (lazy-car (sem-map-utils:sub-parts-with-name
                           (semantic-map semantic-map) part-name :recursive t))))
      (unless part
        (error 'simple-error
               :format-control "Unable to find semantic map object `~a'."
               :format-arguments (list part-name)))
      (unless (typep part 'sem-map-utils:semantic-map-geom)
        (error 'simple-error
               :format-control "Semantic map object `~a' is not a geometric object."
               :format-arguments (list part-name)))
      (sem-map-utils:pose part))))

;; (defmethod copy-object ((obj semantic-map-object) (world bt-reasoning-world))
;;   (with-slots (semantic-map) obj
;;     (change-class
;;      (call-next-method) 'semantic-map-object
;;      :semantic-map (sem-map-utils:copy-semantic-map-object semantic-map))))
;; For multiple inheritance objects of class URDF-SEMANTIC-MAP-OBJECT this function
;; discards all the slots of its ROBOT-OBJECT when changing the class to
;; SEMANTIC-MAP-OBJECT. Therefore, and considering that SEMANTIC-MAP-OBJECT
;; only appears in bullet world as a URDF-SEMANTIC-MAP-OBJECT
;; or maybe SIMPLE-SEMANTIC-MAP-OBJECT and it is itself
;; only a mixin, it makes more sense to define COPY-OBJECT directly on the
;; URDF-SEMANTIC-MAP-OBJECT and SIMPLE-SEMANTIC-MAP-OBJECT classes.

 (defmethod add-object ((world bt-world) (type (eql :semantic-map)) name pose &key urdf)
   (if urdf
       (make-instance 'urdf-semantic-map-object
         :name name
         :world world
         :pose (ensure-pose pose)
         :urdf (etypecase urdf
                 (cl-urdf:robot urdf)
                 (string (handler-bind ((cl-urdf:urdf-type-not-supported #'muffle-warning))
                           (cl-urdf:parse-urdf urdf))))
         :collision-group :static-filter
         :collision-mask '(:default-filter :character-filter))
       (make-instance 'simple-semantic-map-object
         :name name
         :world world
         :pose (ensure-pose pose)
         :collision-group :static-filter
         :collision-mask '(:default-filter :character-filter))))
