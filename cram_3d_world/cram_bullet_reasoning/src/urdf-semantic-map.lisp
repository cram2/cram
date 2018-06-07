;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :btr)

(defclass urdf-semantic-map-object (semantic-map-object robot-object)
  ((link-offsets :initarg :link-offsets
                 :initform (make-hash-table :test 'equal)
                 :reader link-offsets
                 :documentation "Mapping of link names to pose offsets
                 between the body that corresponds to the link and the
                 corresponding object in the semantic map."))
  (:documentation "Multiple inheritance is a pain.
Make sure you know what you're doing before changing anything.
Be especially careful with BTR::COPY-OBJECT."))

(defmethod copy-object ((obj urdf-semantic-map-object) (world bt-reasoning-world))
  (with-slots (link-offsets semantic-map) obj
    (change-class
     (call-next-method) 'urdf-semantic-map-object
     :link-offsets (copy-hash-table link-offsets)
     :semantic-map (sem-map-utils:copy-semantic-map-object semantic-map))))

(defmethod initialize-instance :after ((semantic-map urdf-semantic-map-object)
                                       &key)
  (with-slots (link-offsets) semantic-map
    (dolist (part (sem-map-utils:semantic-map-parts
                   (semantic-map semantic-map) :recursive t))
      (when (and (sem-map-utils:urdf-name part)
                 (typep part 'sem-map-utils:semantic-map-geom))
        (let ((link-pose (link-pose semantic-map (sem-map-utils:urdf-name part))))
          (assert link-pose)
          (setf (gethash (sem-map-utils:urdf-name part) link-offsets)
                (cl-transforms:transform*
                 (cl-transforms:transform-inv
                  (cl-transforms:reference-transform link-pose))
                 (cl-transforms:reference-transform (sem-map-utils:pose part)))))))))

(defmethod invalidate-object :around ((obj urdf-semantic-map-object))
  (with-slots (joint-states) obj
    (let ((old-joint-states (alexandria:copy-hash-table joint-states)))
      (call-next-method)
      (update-semantic-map-poses
       obj
       (loop for name being the hash-keys in joint-states
             using (hash-value new-state)
             for old-state being the hash-values in old-joint-states
             unless (eql new-state old-state) collect name)))))

(defmethod (setf joint-state) :before (new-value (sem-map urdf-semantic-map-object) name)
  ;; (attach-contacting-objects
  ;;  sem-map :detach-invalid t
  ;;  :test (lambda (obj link-name)
  ;;          (declare (ignore link-name))
  ;;          (typep obj 'item)))
  )

(defmethod (setf joint-state) :after (new-value (sem-map urdf-semantic-map-object) name)
  (update-semantic-map-joint sem-map name))

(defmethod (setf link-pose) :before (new-value (sem-map urdf-semantic-map-object) name)
  ;; (attach-contacting-objects
  ;;  sem-map :detach-invalid t
  ;;  :test (lambda (obj link-name)
  ;;          (declare (ignore link-name))
  ;;          (typep obj 'item)))
  )

(defmethod (setf link-pose) :around (new-value (sem-map urdf-semantic-map-object) name)
  (with-slots (urdf links link-offsets semantic-map) sem-map
    (let* ((link (gethash name links))
           (sem-map-obj (lazy-car
                         (sem-map-utils:sub-parts-with-name
                          semantic-map (owl-name-from-urdf-name sem-map name)))))
      (if link
          (sem-map-utils:update-pose
           sem-map-obj (cl-transforms:transform->pose
                        (cl-transforms:transform*
                         (cl-transforms:reference-transform (pose link))
                         (gethash (sem-map-utils:urdf-name sem-map-obj)
                                  link-offsets)))
           :relative nil :recursive t)
          (warn "URDF semantic map link ~s is not physical. Cannot move, sorry." name)))))

(defun update-semantic-map-joint (sem-map joint-name)
  (with-slots (urdf links link-offsets semantic-map joint-states)
      sem-map
    ;; We shouldn't use parent here but child for drawers. The problem
    ;; is that in the current semantic map, the drawer is connected to
    ;; the door by the joint. We will keep this code until the
    ;; semantic map is fixed.
    (let* ((child-link-name
             (cl-urdf:name (cl-urdf:child (gethash joint-name (cl-urdf:joints urdf)))))
           (parent-link-name
             (cl-urdf:name (cl-urdf:parent (gethash joint-name (cl-urdf:joints urdf)))))
           (child-link (gethash child-link-name links))
           (parent-sem-map-obj
             (lazy-car
              (sem-map-utils:sub-parts-with-name
               semantic-map (owl-name-from-urdf-name sem-map parent-link-name))))
           (child-sem-map-obj
             (lazy-car
              (sem-map-utils:sub-parts-with-name
               semantic-map (owl-name-from-urdf-name sem-map child-link-name))))
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
                       (or
                        (gethash (sem-map-utils:urdf-name sem-map-obj)
                                 link-offsets)
                        (cl-transforms:make-identity-transform))))
         :relative nil :recursive t)
        (let ((joint (find-if (lambda (object)
                                (typep object 'sem-map-utils:semantic-map-joint))
                              (sem-map-utils:sub-parts sem-map-obj))))
          (when joint
            (setf (sem-map-utils:joint-position joint)
                  (gethash joint-name joint-states))))))))

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
