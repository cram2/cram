;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;               2014, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;               2019, Vanessa Hassouna <hassouna@uni-bremen.de>
;;;               2019, Thomas Lipps <tlipps@uni-bremen.de>
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

(defvar *robot-model-alpha* nil)

(defgeneric urdf-make-collision-shape (geometry &optional color compound))

(defmethod urdf-make-collision-shape ((box cl-urdf:box)
                                      &optional (color '(0.8 0.8 0.8 1.0)) (compound nil))
  (declare (ignore compound))
  (make-instance 'colored-box-shape
    :half-extents (cl-transforms:v*
                   (cl-urdf:size box) 0.5)
    :color (apply-alpha-value color)))

(defmethod urdf-make-collision-shape ((cylinder cl-urdf:cylinder)
                                      &optional (color '(0.8 0.8 0.8 1.0)) (compound nil))
  (declare (ignore compound))
  (make-instance 'colored-cylinder-shape
    :half-extents (cl-transforms:make-3d-vector
                   (cl-urdf:radius cylinder)
                   (cl-urdf:radius cylinder)
                   (* 0.5 (cl-urdf:cylinder-length cylinder)))
    :color (apply-alpha-value color)))

(defmethod urdf-make-collision-shape ((sphere cl-urdf:sphere)
                                      &optional (color '(0.8 0.8 0.8 1.0)) (compound nil))
  (declare (ignore compound))
  (make-instance 'colored-sphere-shape :radius (cl-urdf:radius sphere)
    :color (apply-alpha-value color)))

(defmethod urdf-make-collision-shape ((mesh cl-urdf:mesh)
                                      &optional (color '(0.8 0.8 0.8 1.0)) (compound nil))
  "Loads the meshes from the specified filename in `mesh' and creates either:
  a `convex-hull-shape' if `compound' is NIL or
  a `compound-shape' if compound it T.
  The former combines all meshes and faces into one convex-hull-shape, while the latter
  contains every single mesh as a seperate convex-hull-shape as children in a compound-shape."
  (let* ((mesh-filename
           (cl-urdf:filename mesh))
         (scale
           (cl-urdf:scale mesh))
         (size
           (cl-urdf:size mesh))
         (collision-shape
           (make-collision-shape-from-mesh
            mesh-filename :color color :scale scale :size size :compound compound)))
    collision-shape))

(defclass robot-object (object)
  ((links :initarg :links :initform (make-hash-table :test 'equal) :reader links)
   (joint-states :initarg :joint-states :initform (make-hash-table :test 'equal)
                 :reader joint-states)
   (urdf :initarg :urdf :reader urdf)
   (attached-objects
    :initarg :attached-objects :initform nil
    :reader attached-objects
    :documentation "An alist that maps object
                     instances to a list of instances of the struct
                     `attachment' and an instance of
                     `collision-information'.")
   (initial-pose :initarg :pose
                 :documentation "Pose that got passed in initially. It
                 is returned by the `pose' method if `reference-body'
                 is invalid.")
   (collision-group :initarg :collision-group
                    :initform :character-filter
                    :reader collision-group
                    :documentation "Contains the collision group of the robot.
                    Defaults to :character-filter for PR2, should be set to
                    :static-filter when used on the kitchen URDF.")
   (collision-mask :initarg :collision-mask
                   :initform '(:default-filter :static-filter)
                   :reader collision-mask
                   :documentation "List of filters, with whom the robot detects
                    collisions with. Contains the :default-filter for regular objects
                    and :static-filter for the kitchen. Swap to :character-filter if
                    this objects is a kitchen.")
   (compound :initarg :compound
             :initform nil
             :reader compound
             :documentation "Determines, if the URDFs meshes are build as
             a compound of meshes, or one single convex hull shape.")))

(defgeneric joint-names (robot-object)
  (:documentation "Returns the list of joints")
  (:method ((robot-object robot-object))
    (loop for name being the hash-keys in (joint-states robot-object)
          collecting name)))

(defgeneric joint-state (robot-object name)
  (:documentation "Returns the value of the joint named `name'"))

(defgeneric (setf joint-state) (new-value robot-object name)
  (:documentation "Sets the specific joint to a new value and updates
  all child-link positions"))

(defgeneric link-names (robot-object)
  (:documentation "Returns a list of link names")
  (:method ((robot-object robot-object))
    (loop for name being the hash-keys in (links robot-object)
          collecting name)))

(defgeneric link-pose (robot-object name)
  (:documentation "Gets the pose of a link"))

(defgeneric (setf link-pose) (new-value robot-object name)
  (:documentation "Sets the pose of a link and all its children"))


(defgeneric object-attached (robot-object object)
  (:documentation "Returns the list of links `object' has been
  attached to.")
  (:method ((robot-object robot-object) (object object))
    (with-slots (attached-objects) robot-object
      (values (mapcar #'attachment-link (car (cdr (assoc (name object) attached-objects
                                                         :test #'equal))))
              (mapcar #'attachment-grasp (car (cdr (assoc (name object) attached-objects
                                                          :test #'equal))))))))

(defmethod attach-object ((robot-object robot-object) (obj object)
                          &key link loose grasp &allow-other-keys)
  "Adds `obj' to the set of attached objects of `robot-object'.
`link' specifies to which link of `robot-object' to attach the `obj'.
If `loose' is set to NIL and the link the object is attached to is moved,
the object moves accordingly.
Otherwise, the attachment is only used as information but does not affect the world."
  (unless (gethash link (links robot-object))
    (error 'simple-error :format-control "Link ~a unknown"
                         :format-arguments (list link)))
  (with-slots (attached-objects) robot-object
    (let* ((new-attachment
             (make-attachment
              :object (name obj) :link link :loose loose :grasp grasp))
           (obj-attachment
             (assoc (name obj) attached-objects :test #'equal))
           (attachment-struct
             (caadr obj-attachment)))
      (if obj-attachment
          (cond
            ((and (string-equal link (attachment-link attachment-struct))
                  (eql loose (attachment-loose attachment-struct)))
             (warn "Object ~a already attached to ~a. Ignoring new attachment."
                   (name obj) (name robot-object))
             (return-from attach-object))
            ((and (string-equal link (attachment-link attachment-struct))
                  (eql loose T))
             (warn "Object ~a already attached to ~a's link ~a but not loosely. ~
                    Ignoring new loose attachment."
                   (name obj) (name robot-object) link)
             (return-from attach-object))
            ((and (string-equal link (attachment-link attachment-struct))
                  (eql loose NIL))
             (warn "Object ~a already attached to ~a's link ~a but loosely. ~
                    Overwriting with new attachment."
                   (name obj) (name robot-object) link)
             (setf (attachment-loose attachment-struct) NIL))
            (t
             (pushnew new-attachment (car (cdr obj-attachment))
                      :test #'equal :key #'attachment-link)))
          (push (cons (name obj)
                      (cons
                       (list new-attachment)
                       (create-static-collision-information obj)))
                attached-objects)))))

(defmethod detach-object ((robot-object robot-object) (object object) &key link)
  "Detaches `object' from the set of attached objects.
 If `link' is specified, detaches `object' only from
 `link'. Otherwise, detaches `object' from all links."
  (with-slots (attached-objects) robot-object
    (let ((attachment (assoc (name object) attached-objects)))
      (when attachment
        (cond (link
               (setf (second attachment)
                     (remove link (second attachment)
                             :test #'equal :key #'attachment-link))
               (unless (second attachment)
                 (setf attached-objects (remove (name object) attached-objects
                                                :key #'car))
                 (reset-collision-information object (cdr (cdr attachment)))))
              (t (setf attached-objects (remove (name object) attached-objects
                                                :key #'car))
                 (reset-collision-information object (cdr (cdr attachment)))))))))

(defmethod detach-all-from-link ((robot-object robot-object) link)
  "Removes all objects form the given `link' of `robot-object'."
  (with-slots (attached-objects) robot-object
       (dolist (attachment attached-objects)
         (let* ((object-name (car attachment))
                (object-instance (object *current-bullet-world* object-name)))
           (if object-instance
               (let ((attached-to-links (object-attached robot-object object-instance)))
                 (when (find link attached-to-links :test #'equalp)
                   (detach-object robot-object object-instance :link link)))
               (setf attached-objects (remove object-name attached-objects :key #'car)))))))

(defmethod detach-all-objects ((robot-object robot-object))
  "Removes all objects form the list of attached objects."
  (with-slots (attached-objects) robot-object
    (dolist (attached-object attached-objects)
      (let ((object-name (car attached-object)))
        (if (object *current-bullet-world* object-name)
            (detach-object robot-object (object *current-bullet-world* object-name))
            (setf attached-objects (remove object-name attached-objects :key #'car)))))))

(defgeneric gc-attached-objects (robot-object)
  (:documentation "Removes all attached objects with an invalid world
  instance")
  (:method ((robot-object robot-object))
    (with-slots (attached-objects) robot-object
      (setf attached-objects
            (remove-if-not (lambda (obj)
                             (slot-value (car obj) 'world))
                           attached-objects)))))

(defgeneric attach-contacting-objects (robot-object
                                       &key blacklist test
                                         detach-invalid)
  (:documentation "Attaches all objects of `world' that are in contact
  to `robot-object' and not a member of the list `blacklist' or `test'
  fails. `test' must be a function with exactly two parameters, the
  object to be attached and the link name it is contacting, and
  returns a generalized boolean indicating if the object should be
  attached. If `detach-invalid' is non-NIL, it is asured that a
  contacting object is only attached to its contacting links.")
  (:method ((robot-object robot-object)
            &key blacklist (test (constantly t))
              detach-invalid)
    (let ((detached-cache (make-hash-table)))
      (gc-attached-objects robot-object)
      (loop for (obj . link-name) in (link-contacts robot-object)
            when (and (not (member obj blacklist)) (funcall test obj link-name)) do
              (when (and detach-invalid (not (gethash obj detached-cache)))
                (setf (gethash obj detached-cache) t)
                (detach-object robot-object obj))
              (attach-object robot-object obj :link link-name)))))

(defmethod initialize-instance :after ((robot-object robot-object)
                                       &key color name pose
                                         (collision-group :character-filter)
                                         (collision-mask '(:default-filter :static-filter))
                                         (compound nil))
  "Translates the rigid bodies, links and joints of the `robot-object' to create and spawn
  the robot. Is usually called by `add-object' to add a URDF object. The `collision-mask'
  contains all `collision-groups' that this robot object will detect collision with.
  Set `compound' T to spawn the robot as compound-shapes, instead of colored-boxes
  or convex-hull-shapes."
  (with-slots (rigid-bodies links urdf pose-reference-body) robot-object
    (labels ((make-link-bodies (pose link)
               "Returns the list of rigid bodies of `link' and all its sub-links"
               (let* ((pose-transform (cl-transforms:reference-transform pose))
                      (collision-elem (cl-urdf:collision link))
                      (bodies (mapcan (lambda (joint)
                                        (make-link-bodies (cl-transforms:transform-pose
                                                           pose-transform (cl-urdf:origin joint))
                                                          (cl-urdf:child joint)))
                                      (cl-urdf:to-joints link))))
                 (if collision-elem
                     (cons (cons
                            (cl-urdf:name link)
                            (make-instance
                                'rigid-body
                              :name (make-rigid-body-name name (cl-urdf:name link))
                              :mass 0
                              :pose (cl-transforms:transform-pose
                                     pose-transform (cl-urdf:origin collision-elem))
                              :collision-shape (urdf-make-collision-shape
                                                (cl-urdf:geometry collision-elem)
                                                (apply-alpha-value
                                                 (or color
                                                     (when (and (cl-urdf:visual link)
                                                                (cl-urdf:material
                                                                 (cl-urdf:visual link)))
                                                       (cl-urdf:color
                                                        (cl-urdf:material
                                                         (cl-urdf:visual link))))
                                                     (let ((some-gray (/ (+ (random 5) 3) 10.0)))
                                                       (list some-gray some-gray some-gray
                                                             (or *robot-model-alpha* 1.0)))))
                                                compound)
                              :collision-flags :cf-default
                              :group collision-group
                              :mask collision-mask))
                           bodies)
                     bodies))))
      (let ((bodies (make-link-bodies pose (cl-urdf:root-link urdf))))
        (initialize-rigid-bodies robot-object (mapcar #'cdr bodies))
        (setf pose-reference-body (make-rigid-body-name name (cl-urdf:name (cl-urdf:root-link urdf))))
        (loop for (name . body) in bodies do
          (setf (gethash name links) body))
        (loop for name being the hash-keys in (cl-urdf:joints urdf) do
          (setf (gethash name (joint-states robot-object)) 0.0d0))))))

(defgeneric link-contacts (robot-object)
  (:method ((robot-object robot-object))
    (flet ((find-link-name (body)
             (loop for name being the hash-keys in (links robot-object)
                     using (hash-value rb) do
                       (when (eq body rb)
                         (return name)))))
      (with-slots (world) robot-object
        (let ((objects (objects world))
              (contacts nil))
          (perform-collision-detection world)
          (dolist (manifold (contact-manifolds world) contacts)
            (let ((obj (loop for obj in objects
                             when (and (rigid-body obj (name (body-1 manifold)))
                                       (rigid-body robot-object (name (body-2 manifold))))
                               do (return (cons obj (body-2 manifold)))
                             when (and (rigid-body obj (name (body-2 manifold)))
                                       (rigid-body robot-object (name (body-1 manifold))))
                               do (return (cons obj (body-1 manifold)))
                             finally (return nil))))
              (when obj
                (push (cons (car obj) (find-link-name (cdr obj)))
                      contacts)))))))))

(defmethod copy-object ((obj robot-object) (world bt-reasoning-world))
  (with-slots (links joint-states urdf) obj
    (change-class
     (call-next-method) 'robot-object
     :links (copy-hash-table links)
     :joint-states (copy-hash-table joint-states)
     :urdf urdf
     :pose (slot-value obj 'initial-pose)
     :attached-objects (copy-list (attached-objects obj)))))

(defmethod add-object ((world bt-world) (type (eql :urdf)) name pose
                       &key urdf color
                         (collision-group :character-filter)
                         (collision-mask  '(:default-filter :static-filter))
                         compound)
  (make-instance 'robot-object
    :name name
    :world world
    :pose (ensure-pose pose)
    :urdf (etypecase urdf
            (cl-urdf:robot urdf)
            (string (handler-bind ((cl-urdf:urdf-type-not-supported #'muffle-warning))
                      (cl-urdf:parse-urdf urdf))))
    :color color
    :collision-group collision-group
    :collision-mask collision-mask
    :compound compound))

(let ((updated-attachments (make-hash-table)))
  ;; Stores the already updated attached objects and the corresponding traversed links

  (defun get-updated-attachments ()
    "This getter exists only for testing `updated-link-in-attachment'
in the unit tests."
    updated-attachments)

  (defun updated-link-in-attachment (link attachment)
    "Returns if the pose of the attached object behind the `attachment'
was updated by checking if `link' was already updated. The already
updated links are saved under the attachment name in `updated-attachments'.
If all links of an attachment were updated the entry under the attachment
name in `updated-attachments' gets deleted."
    (let ((links-to-update (mapcar #'attachment-link 
                                   (remove-if #'attachment-loose
                                              (car (cdr attachment)))))
          (ret T))
      (when (and link (member (cl-urdf:name link) links-to-update :test #'string-equal))
        (if (gethash (car attachment) updated-attachments)
            (setf (gethash (car attachment) updated-attachments)
                  (push (cl-urdf:name link) (gethash (car attachment) updated-attachments)))
            (setf (gethash (car attachment) updated-attachments)
                  (list (cl-urdf:name link))
                  ret
                  NIL))
        ;; checks if the list of links in attachment and the already visited links are equal
        (when (equal
               (length links-to-update)
               (length
                (intersection
                 (gethash (car attachment) updated-attachments)
                 links-to-update :test #'string-equal)))
          (remhash (car attachment) updated-attachments))
        (return-from updated-link-in-attachment ret)))))

(defun update-attached-object-poses (robot-object link pose)
  "Updates the poses of all objects that are attached to
`link'. `pose' is the new pose of `link'"
  (with-slots (attached-objects links) robot-object
    (let ((body (gethash (cl-urdf:name link) links)))
      (when body
        (let* ((attachments
                 (mapcar
                  #'car
                  (remove-if-not
                   (lambda (attachment)
                     (let ((link-attachment
                             (find (cl-urdf:name link) (car (cdr attachment))
                                   :key #'attachment-link :test #'equal)))
                       (and link-attachment
                            (not (attachment-loose link-attachment))
                            (not (updated-link-in-attachment link attachment)))))
                   attached-objects)))
               (body-transform (cl-transforms:reference-transform (pose body)))
               (pose-transform (cl-transforms:reference-transform pose))
               (pose-delta (cl-transforms:transform*
                            (cl-transforms:transform*
                             pose-transform
                             (cl-urdf:origin (cl-urdf:collision link)))
                            (cl-transforms:transform-inv body-transform))))
          (dolist (attachment attachments)
            (let ((attached-object (object (world robot-object) attachment)))
              (if attached-object
                  (setf (pose attached-object)
                        (cl-transforms:transform-pose pose-delta (pose attached-object)))
                  (setf attached-objects (remove attachment attached-objects
                                                 :key #'car))))))))))

(defun update-link-poses (robot-object link pose)
  (declare (type cl-urdf:link link)
           (type robot-object robot-object)
           (type (or cl-transforms:transform cl-transforms:pose) pose))
  "Updates the pose of `link' and all its children according to
current joint states"
  (let ((links (links robot-object))
        (joint-states (joint-states robot-object)))
    (let ((body (gethash (cl-urdf:name link) links))
          (pose-transform (cl-transforms:reference-transform pose)))
      (when body
        (update-attached-object-poses robot-object link pose)
        (setf (pose body) (cl-transforms:transform-pose
                           pose-transform
                           (cl-urdf:origin (cl-urdf:collision link)))))
      (dolist (to-joint (cl-urdf:to-joints link))
        (update-link-poses robot-object
                           (cl-urdf:child to-joint)
                           (cl-transforms:transform-pose
                            pose-transform
                            (cl-transforms:transform-pose
                             (cl-urdf:origin to-joint)
                             (joint-transform to-joint (gethash
                                                        (cl-urdf:name to-joint)
                                                        joint-states)))))))))

(defun joint-transform (joint value)
  (case (cl-urdf:joint-type joint)
    ((:revolute :continuous)
     (cl-transforms:make-transform
      (cl-transforms:make-3d-vector 0 0 0)
      (cl-transforms:axis-angle->quaternion
       (cl-urdf:axis joint)
       (* value 1.0d0))))
    (:prismatic
     (cl-transforms:make-transform
      (cl-transforms:v* (cl-urdf:axis joint) value)
      (cl-transforms:make-quaternion 0 0 0 1)))
    (t (cl-transforms:make-transform
        (cl-transforms:make-3d-vector 0 0 0)
        (cl-transforms:make-quaternion 0 0 0 1)))))

(defun calculate-joint-state (obj name)
  (with-slots (urdf) obj
    (let ((links (links obj)) ; hash tbl of name <-> ridig body
          (urdf-joint (gethash name (cl-urdf:joints urdf)))) ; urdf-joint
      (when urdf-joint
        (let* ((parent-link
                 (cl-urdf:parent urdf-joint))
               (parent-body-pose-in-map
                 (find-parent-pose obj name))
               (map-to-parent-body-transform
                 (cl-transforms:reference-transform parent-body-pose-in-map))
               (child-link
                 (cl-urdf:child urdf-joint))
               (child-body
                 (gethash (cl-urdf:name child-link) links)))
          (when child-body
            (let* ((parent-body-to-its-link-transform
                     (if (cl-urdf:collision parent-link)
                         (cl-transforms:transform-inv
                          (cl-transforms:reference-transform
                           (cl-urdf:origin (cl-urdf:collision parent-link))))
                         (cl-transforms:make-identity-transform)))
                   (parent-link-to-urdf-joint-transform
                     (cl-urdf:origin urdf-joint))
                   (map-to-urdf-joint-transform
                     (cl-transforms:transform*
                      map-to-parent-body-transform
                      parent-body-to-its-link-transform
                      parent-link-to-urdf-joint-transform))
                   (map-to-child-body-transform
                     (cl-transforms:reference-transform
                      (pose child-body)))
                   (child-body-to-its-link-transform
                     (cl-transforms:transform-inv
                      (cl-transforms:reference-transform
                       (cl-urdf:origin (cl-urdf:collision child-link)))))
                   (map-to-child-link-transform
                     (cl-transforms:transform*
                      map-to-child-body-transform
                      child-body-to-its-link-transform)))
              (case (cl-urdf:joint-type urdf-joint)
                ((:revolute :continuous)
                 (cl-transforms:normalize-angle
                  (multiple-value-bind (angle axis)
                      (cl-transforms:angle-between-quaternions
                       (cl-transforms:rotation map-to-urdf-joint-transform)
                       (cl-transforms:rotation map-to-child-link-transform))
                    (if (< (cl-transforms:dot-product
                            axis (cl-urdf:axis urdf-joint))
                           0)
                        (* angle -1)
                        angle))))
                (:prismatic
                 (let ((urdf-joint-to-child-link-transform
                         (cl-transforms:transform*
                          (cl-transforms:transform-inv
                           map-to-urdf-joint-transform)
                          map-to-child-link-transform)))
                   (cl-transforms:dot-product
                    (cl-transforms:translation urdf-joint-to-child-link-transform)
                    (cl-urdf:axis urdf-joint))
                   ;; (* (cl-transforms:v-dist
                   ;;     (cl-transforms:translation map-to-urdf-joint-transform)
                   ;;     (cl-transforms:translation map-to-child-link-transform))
                   ;;    (if (< (cl-transforms:dot-product
                   ;;            (cl-transforms:v-
                   ;;             (cl-transforms:translation map-to-child-link-transform)
                   ;;             (cl-transforms:translation map-to-urdf-joint-transform))
                   ;;            (cl-urdf:axis urdf-joint))
                   ;;           0)
                   ;;        -1 1))
                   ))
                (t 0.0d0)))))))))

(defmethod invalidate-object :after ((obj robot-object))
  (with-slots (world links joint-states) obj
    (loop for name being the hash-keys in links
            using (hash-value body) do
              (setf (gethash name links)
                    (rigid-body obj (name body))))
    (loop for name being the hash-keys in joint-states do
      (setf (gethash name joint-states)
            (or (calculate-joint-state obj name) 0.0d0)))))

(defmethod joint-state ((obj robot-object) name)
  (nth-value 0 (gethash name (joint-states obj))))

(defmethod (setf joint-state) (new-value (obj robot-object) name)
  (with-slots (urdf) obj
    (assert (gethash name (cl-urdf:joints urdf)) ()
            "Joint ~a unknown" name)
    (let* ((joint-states (joint-states obj))
           (joint (gethash name (cl-urdf:joints urdf)))
           (parent (cl-urdf:parent joint))
           (parent-pose (find-parent-pose obj name))
           (limits (when (slot-boundp joint 'cl-urdf:limits)
                     (cl-urdf:limits joint)))
           (joint-type (cl-urdf:joint-type joint)))
      (when (and limits (not (eq joint-type :continuous)))
        (when (eq joint-type :revolute)
          (setf new-value (cl-transforms:normalize-angle new-value)))
        (unless (and (<= new-value (+ (cl-urdf:upper limits) 0.0000001))
                     (>= new-value (- (cl-urdf:lower limits) 0.0000001)))
          (setf new-value (min (max new-value (cl-urdf:lower limits))
                               (cl-urdf:upper limits)))
          (warn "Trying to assert joint value for ~a to ~a but limits are (~a; ~a)"
                name new-value (cl-urdf:lower limits) (cl-urdf:upper limits))))
      (let ((joint-transform
              (cl-transforms:transform*
               (cl-transforms:reference-transform
                (cl-urdf:origin joint))
               (joint-transform joint new-value))))
        (setf (gethash name joint-states) new-value)
        (update-link-poses
         obj (cl-urdf:child joint)
         (cl-transforms:transform*
          (cl-transforms:reference-transform parent-pose)
          (if (cl-urdf:collision parent)
              (cl-transforms:transform-inv
               (cl-transforms:reference-transform
                (cl-urdf:origin (cl-urdf:collision parent))))
              (cl-transforms:make-identity-transform))
          joint-transform)))
      ;; also update state of joints that mimic the joint called `name'
      (loop for joint being the hash-values of (cl-urdf:joints urdf)
            ;; store the mimic slot of JOINT into a variable if it is bound
            for joints-mimic-slot = (and (slot-boundp joint 'cl-urdf:mimics)
                                         (cl-urdf:mimics joint))
            ;; if the joint mimics some other joint (i.e. mimics slot is bound)
            when (and joints-mimic-slot
                      ;; and that other joint happens to be our `name' joint
                      (string-equal (cl-urdf:joint joints-mimic-slot)
                                    name))
              do (let ((multiplier (cl-urdf:multiplier joints-mimic-slot))
                       (offset (cl-urdf:offset joints-mimic-slot)))
                   ;; from ROS wiki: value = multiplier * other_joint_value + offset
                   (setf (joint-state obj (cl-urdf:name joint))
                         (+ (* multiplier new-value) offset)))))))

(defun set-joint-state (robot name new-state)
  (setf (joint-state robot name) new-state))

(defun find-parent-pose (obj joint-name
                         &optional
                           (current-pose (cl-transforms:make-identity-pose)))
  "Tries to find the pose of the parent link of the joint named
`joint-name'. The pose is absolute (fixed frame).
The algorithm works as follows: if the parent link has
a rigid body, use its pose. If not, walk the tree up, apply the
inverse joint transform of parent's from-joint and try again."
  (with-slots (urdf links joint-states) obj
    (let* ((joint (gethash joint-name (cl-urdf:joints urdf)))
           (parent (and joint (cl-urdf:parent joint))))
      (cond ((and parent (gethash (cl-urdf:name parent) links))
             (cl-transforms:transform->pose
              (cl-transforms:transform*
               (cl-transforms:reference-transform current-pose)
               (cl-transforms:reference-transform (pose (gethash (cl-urdf:name parent) links))))))
            ((and parent (cl-urdf:from-joint parent))
             ;; walk the tree up
             (let* ((parent-joint (cl-urdf:from-joint parent))
                    (parent-joint-name (cl-urdf:name parent-joint)))
               (find-parent-pose
                obj parent-joint-name
                (cl-transforms:transform->pose
                 (cl-transforms:transform*
                  (cl-transforms:reference-transform
                   (cl-urdf:origin parent-joint))
                  (joint-transform
                   parent-joint
                   (or (joint-state obj parent-joint-name)
                       0.0))
                  (cl-transforms:reference-transform current-pose))))))
            (t ;; We are at the root. Return the object's inverse pose
             ;; multiplied with current-pose
             (cl-transforms:transform*
              (cl-transforms:reference-transform (pose obj))
              (cl-transforms:reference-transform current-pose)))))))

(defmethod link-pose ((obj robot-object) name)
  ;; We need to handle two different cases here. One is when we have a
  ;; rigid body for a specific link. Then reading the pose is just
  ;; returning the pose of the body minus the pose of the collision
  ;; object in the body. The second case is a link without a rigid
  ;; body, i.e. without a collision model. We need to walk up the tree
  ;; until we reach a body, collecting the transforms along that path.
  (with-slots (links urdf) obj
    (let ((link (gethash name (cl-urdf:links urdf)))
          (body (gethash name links)))
      (cond (body
             (cl-transforms:transform->pose
              (cl-transforms:transform*
               (cl-transforms:reference-transform
                (pose body))
               (cl-transforms:transform-inv
                (cl-transforms:reference-transform
                 (cl-urdf:origin (cl-urdf:collision link)))))))
            ((and link (cl-urdf:from-joint link)
                  (cl-urdf:parent (cl-urdf:from-joint link)))
             (cl-transforms:transform-pose
              (cl-transforms:reference-transform
               (link-pose obj (cl-urdf:name (cl-urdf:parent
                                             (cl-urdf:from-joint link)))))
              (cl-urdf:origin (cl-urdf:from-joint link))))
            (;; We are at the root of the tree
             link (slot-value obj 'initial-pose))
            (t nil)))))

(defmethod (setf link-pose) (new-value (obj robot-object) name)
  (declare (type cl-transforms:pose new-value)
           (type string name))
  "Gets the `name' of the link and sets its new pose.
`new-value' is a pose in map frame.
Updates poses of links in the bullet world and sets the new joint angle of the parent of link.
Only one joint state changes in this situation, so only one joint state is updated."
  (with-slots (urdf) obj
    (let* ((links (links obj))
           (link (gethash name (cl-urdf:links urdf)))
           (body (gethash name links)))
      ;; `link' is a cl-urdf object, `body' is a bullet rigid body
      ;; only uses `body' to check if it exists, otherwise setting pose is impossible
      ;; Note(Lorenz): Should we throw an error here?
      (when body
        (update-link-poses obj link new-value)
        (let ((joint (cl-urdf:from-joint link)))
          (when joint
            (setf (gethash (cl-urdf:name joint) (joint-states obj))
                  (calculate-joint-state obj (cl-urdf:name joint)))))))))

(defmethod pose ((obj robot-object))
  "Gives the pose of root link rigid body"
  (with-slots (urdf) obj
    (link-pose obj (cl-urdf:name (cl-urdf:root-link urdf)))))

(defmethod (setf pose) (new-value (obj robot-object))
  (declare (type cl-transforms:pose new-value))
  "Uses (SETF LINK-POSE) to set the pose of root link and recursively all children"
  (with-slots (urdf) obj
    (setf (link-pose obj (cl-urdf:name (cl-urdf:root-link urdf)))
          new-value)))

(defmacro with-alpha (alpha &body body)
  `(let ((*robot-model-alpha* ,alpha))
     ,@body))

(defun apply-alpha-value (color)
  (if (= (length color) 4)
      (destructuring-bind (r g b a) color
        (list r g b (or *robot-model-alpha* a)))
      (if (= (length color) 3)
          (destructuring-bind (r g b) color
            (list r g b (or *robot-model-alpha* 1.0)))
          (error "Color of an object has to be a list of 3 or 4 values"))))
