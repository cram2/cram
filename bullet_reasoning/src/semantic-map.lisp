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

(defclass semantic-map-object (robot-object sem-map-utils:semantic-map) ())

(defmethod (setf joint-state) :before (new-value (sem-map semantic-map-object) name)
  (attach-contacting-objects
   sem-map :detach-invalid t
   :test (lambda (obj link-name)
           (declare (ignore link-name))
           (typep obj 'household-object))))

(defmethod (setf joint-state) :around (new-value (sem-map semantic-map-object) name)
  (with-slots (urdf links) sem-map
    ;; We shouldn't use parent here but child for drawers. The problem
    ;; is that in the current semantic map, the drawer is connected to
    ;; the door by the joint. We will keep this code until the
    ;; semantic map is fixed.
    (let* ((child-link-name (cl-urdf:name (cl-urdf:child (gethash name (cl-urdf:joints urdf)))))
           (parent-link-name (cl-urdf:name (cl-urdf:parent (gethash name (cl-urdf:joints urdf)))))
           (child-link (gethash child-link-name links))
           (parent-sem-map-obj (lazy-car
                                (sem-map-utils:sub-parts-with-name
                                 sem-map (owl-name-from-urdf-name sem-map parent-link-name))))
           (child-sem-map-obj (lazy-car
                                (sem-map-utils:sub-parts-with-name
                                 sem-map  (owl-name-from-urdf-name sem-map child-link-name))))
           (sem-map-obj (string-case (sem-map-utils:obj-type parent-sem-map-obj)
                          ("Drawer" parent-sem-map-obj)
                          (t child-sem-map-obj)))
           (original-link-pose (pose child-link)))
      (call-next-method)
      (let* ((new-link-pose (pose child-link))
             (diff (cl-transforms:transform*
                    (cl-transforms:reference-transform new-link-pose)
                    (cl-transforms:transform-inv
                     (cl-transforms:reference-transform original-link-pose)))))
        (sem-map-utils:update-pose sem-map-obj diff :relative t :recursive t)))))

(defmethod (setf link-pose) :before (new-value (sem-map semantic-map-object) name)
  (attach-contacting-objects
   sem-map :detach-invalid t
   :test (lambda (obj link-name)
           (declare (ignore link-name))
           (typep obj 'household-object))))

(defmethod (setf link-pose) :around (new-value (sem-map semantic-map-object) name)
  (with-slots (urdf links) sem-map
    (let* ((link (gethash name links))
           (sem-map-obj (lazy-car
                         (sem-map-utils:sub-parts-with-name
                          sem-map (owl-name-from-urdf-name sem-map name))))
           (original-link-pose (pose link)))
      (call-next-method)
      (let* ((new-link-pose (pose link))
             (diff (cl-transforms:transform*
                    (cl-transforms:reference-transform new-link-pose)
                    (cl-transforms:transform-inv
                     (cl-transforms:reference-transform original-link-pose)))))
        (sem-map-utils:update-pose sem-map-obj diff :relative t :recursive t)))))

(defmethod copy-object ((obj semantic-map-object) (world bt-reasoning-world))
  (with-slots (pose parts) obj
    (change-class (call-next-method) 'semantic-map-object :parts parts)))

(defmethod add-object ((world bt-world) (type (eql 'semantic-map)) name pose &key urdf)
  (let ((sem-map (sem-map-utils:get-semantic-map)))
    (change-class (add-object world 'urdf name pose :urdf urdf) 'semantic-map-object
                  :parts (slot-value sem-map 'sem-map-utils::parts))))

