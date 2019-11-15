;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Thomas Lipps <tlipps@uni-bremen.de>
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

(defparameter *mesh-files*
  '((:mug "package://cram_bullet_reasoning/resource/mug.stl" t)
    (:mug-compound "package://cram_bullet_reasoning/resource/mug_compound.dae" t)
    (:plate "package://cram_bullet_reasoning/resource/plate.stl" nil)
    (:plate-compound "package://cram_bullet_reasoning/resource/plate_compound.dae" nil)
    (:tray "package://cram_bullet_reasoning/resource/tray.stl" nil)
    (:tray-compound "package://cram_bullet_reasoning/resource/tray_compound.dae" nil)
    (:cup "package://cram_bullet_reasoning/resource/cup.stl" nil)
    (:cup-compound "package://cram_bullet_reasoning/resource/cup_compound.dae" nil)
    (:mondamin "package://cram_bullet_reasoning/resource/mondamin.stl" nil)
    (:pot "package://cram_bullet_reasoning/resource/pot-ww.stl" nil)
    (:weisswurst "package://cram_bullet_reasoning/resource/ww.stl" nil)
    (:bowl "package://cram_bullet_reasoning/resource/bowl.stl" nil)
    (:bowl-compound "package://cram_bullet_reasoning/resource/bowl_compound.dae" nil)
    (:fork "package://cram_bullet_reasoning/resource/fork.stl" nil)
    (:knife "package://cram_bullet_reasoning/resource/knife.stl" nil)
    (:spatula "package://cram_bullet_reasoning/resource/spatula.stl" nil)
    (:cap "package://cram_bullet_reasoning/resource/cap.stl" t)
    (:glasses "package://cram_bullet_reasoning/resource/glasses.stl" nil)
    (:glove "package://cram_bullet_reasoning/resource/glove.stl" nil)
    (:shoe "package://cram_bullet_reasoning/resource/shoe.stl" nil)))


(defun add-objects-to-mesh-list (ros-package &key (directory "resource") extension)
  "Adds all meshes from `ros-package' resource directory into *mesh-files* list.
The name in the list is a keyword that is created by lispifying the filename."
  (mapcar (lambda (object-filename-and-object-extension)
            (declare (type list object-filename-and-object-extension))
            (destructuring-bind (object-filename object-extension)
                object-filename-and-object-extension
              (if (if extension
                      (string-equal object-extension extension)
                      (or (string-equal object-extension "stl")
                          (string-equal object-extension "dae")))
                  (let* ((lisp-name (roslisp-utilities:lispify-ros-underscore-name
                                     object-filename :keyword))
                         (new-entry (list lisp-name
                                          (format nil "package://~a/~a/~a.~a"
                                                  ros-package directory
                                                  object-filename object-extension)
                                          nil))
                         (position-of-entry (position lisp-name *mesh-files* :key #'car)))
                    (if position-of-entry
                        (setf (nth position-of-entry *mesh-files*) new-entry)
                        (push new-entry *mesh-files*))
                    lisp-name))))
          (mapcar (lambda (pathname)
                    (list (pathname-name pathname) (pathname-type pathname)))
                  (directory (physics-utils:parse-uri
                              (format nil "package://~a/~a/*.*" ros-package directory))))))

(defclass item (object)
  ((types :reader item-types :initarg :types)
   (attached-objects :reader attached-objects :initarg :attached-objects
                     :type 'list :initform nil)))

(defmethod copy-object ((object item) (world bt-reasoning-world))
  (change-class (call-next-method) 'item
                :types (item-types object)))

(defgeneric item-dimensions (object)
  (:method ((object item))
    (bounding-box-dimensions (aabb object)))
  (:method ((object-type symbol))
    (or (cutlery-dimensions object-type)
        (let ((mesh-specification (assoc object-type *mesh-files*)))
          (assert
           mesh-specification ()
           "Couldn't find a mesh for object type ~a." object-type)
          (destructuring-bind (type uri &optional flip-winding-order)
              mesh-specification
            (declare (ignore type))
            (let ((model-filename (physics-utils:parse-uri uri)))
              (with-file-cache
                  model model-filename (physics-utils:load-3d-model
                                        model-filename
                                        :flip-winding-order flip-winding-order)
                (values
                 (physics-utils:calculate-aabb
                  (physics-utils:3d-model-vertices model))))))))))

(defgeneric cutlery-dimensions (type)
  (:method ((type t))
    nil)
  (:method ((type (eql :knife)))
    (cl-transforms:make-3d-vector 0.1 0.01 0.005))
  (:method ((type (eql :fork)))
    (cl-transforms:make-3d-vector 0.1 0.015 0.005)))

(defun make-item (world name types &optional bodies (add-to-world t))
  (make-instance 'item
    :name name
    :world world
    :rigid-bodies bodies
    :add add-to-world
    :types types))

(defun make-octagon-prism-shape (radius height)
  "Returns a collision shape that is a octagon prism, i.e. that has an
  octagon at its base."
  (let ((compound-shape (make-instance 'compound-shape)))
    (dotimes (i 4)
      (add-child-shape compound-shape
                       (cl-transforms:make-pose
                        (cl-transforms:make-3d-vector 0 0 0)
                        (cl-transforms:axis-angle->quaternion
                         (cl-transforms:make-3d-vector 0 0 1)
                         (/ (* i pi)
                            4)))
                       (make-instance
                           'box-shape
                         :half-extents (cl-transforms:make-3d-vector
                                        radius (* radius (sin (/ pi 8)))
                                        (/ height 2)))))
    compound-shape))

(defun make-cup-shape (radius height handle-size)
  (let ((collision-shape (make-octagon-prism-shape radius height)))
    (add-child-shape collision-shape
                     (cl-transforms:make-pose
                      (cl-transforms:make-3d-vector
                       (+ (* radius (cos (/ pi 8)))
                          (/ (cl-transforms:x handle-size)
                             2))
                       0 0)
                      (cl-transforms:make-quaternion 0 0 0 1))
                     (make-instance
                         'box-shape
                       :half-extents (cl-transforms:v* handle-size 0.5)))
    collision-shape))

(defmethod add-object ((world bt-world) (type (eql :generic-cup)) name pose
                       &key
                         mass radius height
                         (handle-size (cl-transforms:make-3d-vector
                                       0.03 0.01 (* height 0.8))))
  (make-item world name '(generic-cup)
             (list
              (make-instance
                  'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-cup-shape radius height handle-size)))))

(defmethod add-object ((world bt-world) (type (eql :mug)) name pose &key
                                                                      mass)
  (add-object world :mesh name pose :mass mass :mesh :mug))

(defmethod add-object ((world bt-world) (type (eql :mesh)) name pose
                       &key mass mesh (color '(0.5 0.5 0.5 1.0)) types (scale 1.0)
                         disable-face-culling (compound *all-meshes-as-compound*))
  (let ((mesh-desc
           (etypecase mesh
             (symbol (if (assoc mesh *mesh-files*)
                         (cdr (assoc mesh *mesh-files*))
                         (error "(btr add-object) Item of type ~a is unknown." mesh)))
             (string (list mesh nil))
             (physics-utils:3d-model (physics-utils:scale-3d-model mesh scale)))))
    (let ((collision-shape (if (listp mesh-desc)
                               (make-collision-shape-from-mesh (car mesh-desc)
                                                               :scale scale
                                                               :compound compound
                                                               :color color
                                                               :disable-face-culling disable-face-culling
                                                               :flip-winding-order (cadr mesh-desc))
                               (make-instance 'convex-hull-mesh-shape
                                              :points (physics-utils:3d-model-vertices mesh-desc)
                                              :faces (physics-utils:3d-model-faces mesh-desc)
                                              :color color
                                              :disable-face-culling disable-face-culling))))
          (make-item world name (or types (list mesh))
                     (list
                      (make-instance 'rigid-body
                                     :name name :mass mass :pose (ensure-pose pose)
                                     :collision-shape
                                     collision-shape))))))

(defmethod add-object ((world bt-world) (type (eql :cutlery)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) cutlery-type)
  (let ((size (cutlery-dimensions cutlery-type)))
    (assert size)
    (make-item world name (list type cutlery-type)
               (list
                (make-instance 'rigid-body
                  :name name :mass mass :pose (ensure-pose pose)
                  :collision-shape (make-instance 'colored-box-shape
                                     :half-extents size
                                     :color color))))))

(defmethod add-object ((world bt-world) (type (eql :pancake-maker)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) size)
  (assert size)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-cylinder-shape
                                   :half-extents (ensure-vector size)
                                   :color color)))))

(defmethod add-object ((world bt-world) (type (eql :pancake)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) size)
  (assert size)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-cylinder-shape
                                   :half-extents (ensure-vector size)
                                   :color color)))))

(defmethod add-object ((world bt-world) (type (eql :orange)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) radius)
  (assert radius)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-sphere-shape
                                   :radius radius
                                   :color color)))))

(defmethod add-object ((world bt-world) (type (eql :apple)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) radius)
  (assert radius)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-sphere-shape
                                   :radius radius
                                   :color color)))))

(defmethod add-object ((world bt-world) (type (eql :sugar-box)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) size)
  (assert size)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-box-shape
                                   :half-extents (ensure-vector size)
                                   :color color)))))

(defmethod add-object ((world bt-world) (type (eql :cereal)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) size)
  (assert size)
  (make-item world name (list type)
             (list
              (make-instance 'rigid-body
                :name name :mass mass :pose (ensure-pose pose)
                :collision-shape (make-instance 'colored-box-shape
                                   :half-extents (ensure-vector size)
                                   :color color)))))



(defmethod get-loose-attached-objects ((object item))
  "Returns all objects attached to `object',
where ATTACHMENTs have the keyword LOOSE as not NIL."
  (mapcar #'car
          (remove-if-not
           (alexandria:compose #'attachment-loose #'car #'car #'cdr)
           (attached-objects object))))

(let ((already-visited '()))
  (defmethod remove-loose-attachment-for ((object item))
    "Searches if the `object' was connected unidirectional/loosly to other
objects and removes ALL corresponding attachments if so.
To search through the attached objects ALREADY-VISITED will help to check
if we already checked this object, as this is a recursive function.
In bidirectional attachments both objects are attached to each other.
In unidirectional attachments, one object is properly attached,
and the other one is loosly attached. "
    (let ((loose-attached-objects (get-loose-attached-objects object)))
      (when loose-attached-objects
        ;; Map the following: (detach-object object loosly-attached-object)
        (mapcar (alexandria:curry #'detach-object object)
                (mapcar (alexandria:curry #'object *current-bullet-world*)
                        loose-attached-objects))))
    ;; searching recrusivly, if an object with attachments was attached to something
    ;; to remove unidirectional/loose objects attachments if they were attached too
    (when (and (slot-boundp object 'attached-objects)
               (> (length (attached-objects object)) 0))
      (push (name object) already-visited)
      (loop for attached-object in (mapcar (lambda (attach)
                                             (object *current-bullet-world* (car attach)))
                                           (attached-objects object))
            do (unless (member (name attached-object) already-visited)
                 (remove-loose-attachment-for attached-object)))
      (if (equal (car (last already-visited)) (name object))
          (setf already-visited '())))))

(defmethod attach-object ((other-object item) (object item)
                          &key attachment-type loose skip-removing-loose link grasp)
  "Attaches `object' to `other-object': adds an attachment to the
attached-objects lists of each other. `attachment-type' is a keyword
that specifies the type of attachment. `loose' specifies if the attachment
is bidirectional (nil) or unidirectional (t). `skip-removing-loose' is for
attaching more objects unidirectional and should be for this T. See
`attach-object' above."
  (declare (ignore link grasp)) ;; used in robot-model.lisp
  (when (equal (name object) (name other-object))
    (warn "Cannot attach an object to itself: ~a" (name object))
    (return-from attach-object))
  (when (member (name object) (attached-objects other-object))
    (warn "Item ~a already attached to ~a. Ignoring new attachment."
          (name object) (name other-object))
    (return-from attach-object))
  (unless skip-removing-loose
    (remove-loose-attachment-for object))
  (push (cons (name object)
              (cons
               (list (make-attachment :object (name object)
                                      :attachment attachment-type))
               (create-static-collision-information object)))
        (slot-value other-object 'attached-objects))
  (push (cons (name other-object)
              (cons
               (list (make-attachment :object (name other-object)
                                      :loose loose :attachment attachment-type))
               (create-static-collision-information other-object)))
        (slot-value object 'attached-objects)))

(defmethod attach-object ((other-objects list) (object item)
                          &key attachment-type loose)
  "Will be used if an attachment should be made from one item to more
than one item. If `loose' T the other attachments have to be made with
`skip-removing-loose' as T to prevent removing loose attachments between
the element before in `other-objects' and `object'."
  (attach-object (first other-objects) object :attachment-type attachment-type :loose loose)
  (mapcar (lambda (obj)
            (attach-object obj object
                           :attachment-type attachment-type :loose loose
                           :skip-removing-loose T))
          (cdr other-objects)))

(defmethod detach-object ((other-object item) (object item) &key)
  "Removes item names from the given arguments in the corresponding `attached-objects' lists
   of the given items."
  (when (equal (name object) (name other-object))
    (warn "Cannot attach an object to itself: ~a" (name object))
    (return-from detach-object))
  (flet ((get-attachment-object (elem)
           (attachment-object (car (second elem))))
         (get-collision-info (attached obj)
           (cdr (cdr (assoc (name attached) (attached-objects obj))))))
    (reset-collision-information object (get-collision-info object other-object))
    (reset-collision-information other-object (get-collision-info other-object object))
    (setf (slot-value other-object 'attached-objects)
          (remove (name object) (attached-objects other-object)
                  :key #'get-attachment-object :test #'equal))
    (setf (slot-value object 'attached-objects)
          (remove (name other-object) (attached-objects object)
                  :key #'get-attachment-object :test #'equal))))

(defmethod detach-all-objects ((object item))
  (with-slots (attached-objects) object
    (dolist (attached-object attached-objects)
      (let ((object-name (car attached-object)))
        (if (object *current-bullet-world* object-name)
            (detach-object (name object) object-name))))))

(let ((already-moved '()))
  (defmethod (setf pose) :around (new-value (object item))
    "Since we save the original pose of the object at the time of attaching,
it is possible to change the pose of its attachments when its pose changes."
    (if (and (slot-boundp object 'attached-objects)
             (> (length (attached-objects object)) 0))
        (let ((carrier-transform
                (cl-transforms:transform-diff
                 (cl-transforms:pose->transform new-value)
                 (cl-transforms:pose->transform (pose object)))))
          ;; If no attached item already moved or wasn't already moved
          (unless (and already-moved
                       (member (name object) already-moved :test #'equal))
            (push (name object) already-moved)
            (call-next-method)
            (dolist (attachment (remove-if #'attachment-loose
                                           (mapcar #'car
                                                   (mapcar #'second
                                                           (attached-objects object)))))
              (let ((current-attachment-pose
                      (pose (object *current-bullet-world* (attachment-object attachment)))))
                (when (and carrier-transform current-attachment-pose)
                  (setf (pose (btr:object btr:*current-bullet-world*
                                          (attachment-object attachment)))
                        (cl-transforms:transform-pose
                         carrier-transform
                         current-attachment-pose)))))
            ;; If all attachments from root head passed, remove all.
            (if (equal (name object) (car (last already-moved)))
                (setf already-moved '()))))
        (call-next-method))))
