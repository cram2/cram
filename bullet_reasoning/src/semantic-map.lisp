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

(defclass semantic-map-object (robot-object sem-map-utils:semantic-map)
  ((link-offsets :initform (make-hash-table :test 'equal)
                 :reader link-offsets
                 :documentation "Mapping of link names to pose offsets
                 between the body that corresponds to the link and the
                 corresponding object in the semantic map.")))

(defmethod initialize-instance :after ((semantic-map semantic-map-object)
                                       &key)
  (with-slots (link-offsets) semantic-map
    (dolist (part (sem-map-utils:semantic-map-parts semantic-map :recursive t))
      (when (and (sem-map-utils:urdf-name part)
                 (typep part 'sem-map-utils:semantic-map-geom))
        (let ((link-pose (link-pose semantic-map (sem-map-utils:urdf-name part))))
          (assert link-pose)
          (setf (gethash (sem-map-utils:urdf-name part) link-offsets)
                (cl-transforms:transform*
                 (cl-transforms:transform-inv
                  (cl-transforms:reference-transform link-pose))                   
                 (cl-transforms:reference-transform (sem-map-utils:pose part)))))))))

(defmethod invalidate-object :around ((obj semantic-map-object))
  (with-slots (joint-states) obj
    (let ((old-joint-states (alexandria:copy-hash-table joint-states)))
      (call-next-method)
      (update-semantic-map-poses
       obj
       (loop for name being the hash-keys in joint-states
             using (hash-value new-state)
             for old-state being the hash-values in old-joint-states
             unless (eql new-state old-state) collect name)))))

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

(defmethod (setf joint-state) :after (new-value (sem-map semantic-map-object) name)
  (update-semantic-map-joint sem-map name))

(defmethod (setf link-pose) :before (new-value (sem-map semantic-map-object) name)
  (attach-contacting-objects
   sem-map :detach-invalid t
   :test (lambda (obj link-name)
           (declare (ignore link-name))
           (typep obj 'household-object))))

(defmethod (setf link-pose) :around (new-value (sem-map semantic-map-object) name)
  (with-slots (urdf links link-offsets) sem-map
    (let* ((link (gethash name links))
           (sem-map-obj (lazy-car
                         (sem-map-utils:sub-parts-with-name
                          sem-map (owl-name-from-urdf-name sem-map name)))))
      (sem-map-utils:update-pose
       sem-map-obj (cl-transforms:transform->pose
                    (cl-transforms:transform*
                     (cl-transforms:reference-transform (pose link))
                     (gethash (sem-map-utils:urdf-name sem-map-obj)
                              link-offsets)))
       :relative nil :recursive t))))

(defmethod copy-object ((obj semantic-map-object) (world bt-reasoning-world))
  (with-slots (pose sem-map-utils:parts) obj
    (change-class
     (call-next-method) 'semantic-map-object
     :parts sem-map-utils:parts)))

(defmethod add-object ((world bt-world) (type (eql 'semantic-map)) name pose &key urdf)
  (make-instance 'semantic-map-object
    :name name
    :world world
    :pose (ensure-pose pose)
    :urdf (etypecase urdf
            (cl-urdf:robot urdf)
            (string (handler-bind ((cl-urdf:urdf-type-not-supported #'muffle-warning))
                      (cl-urdf:parse-urdf urdf))))
    :parts (slot-value (sem-map-utils:get-semantic-map) 'sem-map-utils::parts)
    :collision-group :static-filter
    :collision-mask '(:default-filter :character-filter)))

(defun update-semantic-map-joint (sem-map joint-name)
  (with-slots (urdf links link-offsets) sem-map
    ;; We shouldn't use parent here but child for drawers. The problem
    ;; is that in the current semantic map, the drawer is connected to
    ;; the door by the joint. We will keep this code until the
    ;; semantic map is fixed.
    (let* ((child-link-name (cl-urdf:name (cl-urdf:child (gethash joint-name (cl-urdf:joints urdf)))))
           (parent-link-name (cl-urdf:name (cl-urdf:parent (gethash joint-name (cl-urdf:joints urdf)))))
           (child-link (gethash child-link-name links))
           (parent-sem-map-obj (lazy-car
                                (sem-map-utils:sub-parts-with-name
                                 sem-map (owl-name-from-urdf-name sem-map parent-link-name))))
           (child-sem-map-obj (lazy-car
                               (sem-map-utils:sub-parts-with-name
                                sem-map  (owl-name-from-urdf-name sem-map child-link-name))))
           (sem-map-obj  (string-case (if parent-sem-map-obj
                                          (sem-map-utils:obj-type parent-sem-map-obj)
                                          "")
                          ("Drawer" parent-sem-map-obj)
                          (t child-sem-map-obj))))
      (when sem-map-obj
        (sem-map-utils:update-pose
         sem-map-obj (cl-transforms:transform->pose
                      (cl-transforms:transform*
                       (cl-transforms:reference-transform (pose child-link))
                       (gethash (sem-map-utils:urdf-name sem-map-obj)
                                link-offsets)))
         :relative nil :recursive t)))))

(defun update-semantic-map-poses (sem-map &optional (joint-names nil joint-names-p))
  ;; Instead of iterating over all links, just iterate over all
  ;; non-fixed joints and use code similar to (setf joint-state) The
  ;; problem with this code is that the mapping between urdf links and
  ;; sem-map parts somehow seems to be broken
  (with-slots (urdf) sem-map
    (let ((joint-names (if joint-names-p
                           joint-names
                           (alexandria:hash-table-keys
                            (cl-urdf:joints urdf)))))
      (dolist (joint-name joint-names)
        (let ((joint (gethash joint-name (cl-urdf:joints urdf))))
          (unless (eq (cl-urdf:joint-type joint)
                      :fixed)
            (update-semantic-map-joint sem-map joint-name)))))))
