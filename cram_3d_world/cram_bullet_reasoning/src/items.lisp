;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Thomas Lipps <tlipps@uni-bremen.de>
;;;                     Jonas Dech <jdech[at]uni-bremen.de>
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
    (:cup-non-compound "package://cram_bullet_reasoning/resource/cup_non_compound.stl" nil)
    (:cup "package://cram_bullet_reasoning/resource/cup.dae" nil)
    (:mondamin "package://cram_bullet_reasoning/resource/mondamin.stl" nil)
    (:pot "package://cram_bullet_reasoning/resource/pot-ww.stl" nil)
    (:weisswurst "package://cram_bullet_reasoning/resource/ww.stl" nil)
    (:bowl-original "package://cram_bullet_reasoning/resource/bowl_original.stl" t)
    (:bowl-non-compound "package://cram_bullet_reasoning/resource/bowl_non_compound.stl" nil)
    (:bowl "package://cram_bullet_reasoning/resource/bowl.dae" nil)
    (:fork "package://cram_bullet_reasoning/resource/fork.stl" nil)
    (:knife "package://cram_bullet_reasoning/resource/knife.stl" nil)
    (:spatula "package://cram_bullet_reasoning/resource/spatula.stl" nil)
    (:cap "package://cram_bullet_reasoning/resource/cap.stl" t)
    (:glasses "package://cram_bullet_reasoning/resource/glasses.stl" nil)
    (:glove "package://cram_bullet_reasoning/resource/glove.stl" nil)
    (:shoe "package://cram_bullet_reasoning/resource/shoe.stl" nil)
    (:arrow "package://cram_bullet_reasoning/resource/arrow.stl" nil))
  "(mesh-name-in-CRAM  mesh-ROS-uri-path  flip-winding-order-of-the-mesh")


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

(defun make-item (world name types &optional bodies (add-to-world t))
  (make-instance 'item
    :name name
    :world world
    :rigid-bodies bodies
    :add add-to-world
    :types types))

(defgeneric item-dimensions (object)
  (:method ((object item))
    (calculate-bb-dims object))
  (:method ((object-type symbol))
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
              (physics-utils:3d-model-vertices model)))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;; ATTACHMENTS ;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod attach-object ((other-object item) (object item)
                          &key attachment-type loose
                          skip-removing-loose link grasp)
  "Attaches `object' to `other-object': adds an attachment to the
attached-objects lists of each other. `attachment-type' is a keyword
that specifies the type of attachment. `loose' specifies if the attachment
is bidirectional (nil) or unidirectional (t). In bidirectional
attachments both objects are attached to each other. In unidirectional/loose
attachments, one object is properly attached, and the other one is
loosely attached. `skip-removing-loose' should be T for attaching more objects
unidirectional. See `attach-object' above."
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
               ;; Since robot objects are not in the attached-objects
               ;; list of items, this has to be copied manuelly:
               (if (and (get-robot-object)
                        (object-attached (get-robot-object) object))
                   (get-collision-information object (get-robot-object))
                   (create-static-collision-information object))))
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
  (if other-objects
      (progn
        (attach-object (first other-objects) object
                       :attachment-type attachment-type :loose loose)
        (mapcar (lambda (obj)
                  (attach-object obj object
                                 :attachment-type attachment-type :loose loose
                                 :skip-removing-loose T))
                (cdr other-objects)))
      (warn "Trying to attach an object to a NIL.")))

(defmethod detach-object ((other-object item) (object item) &key)
  "Removes item names from the given arguments in the corresponding
`attached-objects' lists of the given items."
  (when (equal (name object) (name other-object))
    (warn "Cannot attach an object to itself: ~a" (name object))
    (return-from detach-object))
  (flet ((get-attachment-object (elem)
           (attachment-object (car (second elem)))))
    (let ((object-collision-info
            (get-collision-information object other-object))
          (other-object-collision-info
            (get-collision-information other-object object)))
      (setf (slot-value other-object 'attached-objects)
            (remove (name object) (attached-objects other-object)
                    :key #'get-attachment-object :test #'equal))
      (setf (slot-value object 'attached-objects)
            (remove (name other-object) (attached-objects object)
                    :key #'get-attachment-object :test #'equal))
      (reset-collision-information object object-collision-info)
      (reset-collision-information other-object other-object-collision-info))))

(defmethod detach-all-objects ((object item))
  (with-slots (attached-objects) object
    (dolist (attached-object attached-objects)
      (let ((object-name (car attached-object)))
        (if (object *current-bullet-world* object-name)
            (detach-object (name object) object-name))))))

(let ((already-moved '()))
  (defmethod (setf pose) :around (new-value (object item))
    "Updates the pose of the object and its attached objects: since
the original pose of the object is saved at the time of attaching,
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
                  (setf (pose (btr:object *current-bullet-world*
                                          (attachment-object attachment)))
                        (cl-transforms:transform-pose
                         carrier-transform
                         current-attachment-pose)))))
            ;; If all attachments from root head passed, remove all.
            (if (equal (name object) (car (last already-moved)))
                (setf already-moved '()))))
        (call-next-method))))

;;;;;;;;;;;;;;;;;;;;; SPAWNING MESH AND PRIMITIVE-SHAPED ITEMS ;;;;;;;;;;;;

(defmethod add-object ((world bt-world) (type (eql :mesh)) name pose
                       &key mass mesh (color '(0.5 0.5 0.5 1.0)) types (scale 1.0)
                         disable-face-culling (compound *all-meshes-as-compound*))
  (let ((collision-shape
          (etypecase mesh

            (physics-utils:3d-model
             (let ((scaled-mesh (physics-utils:scale-3d-model mesh scale)))
               (make-instance 'convex-hull-mesh-shape
                 :points (physics-utils:3d-model-vertices scaled-mesh)
                 :faces (physics-utils:3d-model-faces scaled-mesh)
                 :color color
                 :disable-face-culling disable-face-culling)))

            ((or string symbol)
             (let (path flip-winding-order)
               (etypecase mesh
                 (string
                  (setf path mesh
                        flip-winding-order nil))
                 (symbol
                  (let ((mesh-files-entry (assoc mesh *mesh-files*)))
                    (if mesh-files-entry
                        (setf path (second mesh-files-entry)
                              flip-winding-order (third mesh-files-entry))
                        (error "[btr add-object] Item ~a is unknown ~
                                in btr:*mesh-files*." mesh)))))
               (make-collision-shape-from-mesh
                path
                :scale scale
                :compound compound
                :color color
                :disable-face-culling disable-face-culling
                :flip-winding-order flip-winding-order))))))

    (make-item world name (or types (list mesh))
               (list
                (make-instance 'rigid-body
                  :name name :mass mass :pose (ensure-pose pose)
                  :collision-shape collision-shape)))))

(defmethod add-object ((world bt-world) (type (eql :mug)) name pose &key mass)
  (add-object world :mesh name pose :mass mass :mesh :mug))


(defmethod add-object ((world bt-world) (type (eql :generic-cup)) name pose
                       &key
                         mass radius height
                         (handle-size (cl-transforms:make-3d-vector
                                       0.03 0.01 (* height 0.8))))

  (labels ((make-octagon-prism-shape (radius height)
             "Returns a collision shape that is a octagon prism, ~
            i.e. that has an octagon at its base."
             (let ((compound-shape (make-instance 'compound-shape)))
               (dotimes (i 4)
                 (add-child-shape
                  compound-shape
                  (cl-transforms:make-pose
                   (cl-transforms:make-3d-vector 0 0 0)
                   (cl-transforms:axis-angle->quaternion
                    (cl-transforms:make-3d-vector 0 0 1)
                    (/ (* i pi)
                       4)))
                  (make-instance 'box-shape
                    :half-extents (cl-transforms:make-3d-vector
                                   radius (* radius (sin (/ pi 8)))
                                   (/ height 2)))))
               compound-shape))

           (make-cup-shape (radius height handle-size)
             (let ((collision-shape (make-octagon-prism-shape radius height)))
               (add-child-shape
                collision-shape
                (cl-transforms:make-pose
                 (cl-transforms:make-3d-vector
                  (+ (* radius (cos (/ pi 8)))
                     (/ (cl-transforms:x handle-size)
                        2))
                  0 0)
                 (cl-transforms:make-quaternion 0 0 0 1))
                (make-instance 'box-shape
                  :half-extents (cl-transforms:v* handle-size 0.5)))
               collision-shape)))

    (make-item world name '(generic-cup)
               (list
                (make-instance 'rigid-body
                  :name name :mass mass :pose (ensure-pose pose)
                  :collision-shape (make-cup-shape radius height handle-size))))))


(defmethod add-object ((world bt-world) (type (eql :basket)) name pose
                       &key (mass 1.0) length width height (handle-height 0.09))
  "Creates a collision shape in the form of a basket and adds it to the world.
The length, width and height have to be given for the function to work."
  (assert length)
  (assert width)
  (assert height)
  ;; have to check if the object with such a name already exists,
  ;; otherwise rogue objects will be spawned
  (unless (object world name)
    (let ((compound-shape (make-instance 'compound-shape)))
      ;; Walls
      (dotimes (i 2)
        (add-child-shape
         compound-shape
         (cl-transforms:make-pose
          (cl-transforms:make-3d-vector (* i width) 0 0)
          (cl-transforms:make-quaternion 0 0 0 1))
         (make-instance 'box-shape
           :half-extents (cl-transforms:v*
                          (cl-transforms:make-3d-vector 0.01 length height)
                          0.5))))
      (dotimes (i 2)
        (add-child-shape
         compound-shape
         (cl-transforms:make-pose
          (cl-transforms:make-3d-vector (/ width 2)
                                        ;; ((i + 1) * -2 + 3) * length/2
                                        (* (+ (* (+ i 1) -2) 3) (/ length 2))
                                        0)
          (cl-transforms:make-quaternion 0 0 1 0))
         (make-instance 'box-shape
           :half-extents (cl-transforms:v*
                          (cl-transforms:make-3d-vector width 0.01 height)
                          0.5))))
      ;; Bottom
      (add-child-shape
       compound-shape
       (cl-transforms:make-pose
        (cl-transforms:make-3d-vector (/ width 2) 0 (- (/ height 2)))
        (cl-transforms:make-quaternion 0 0 0 1))
       (make-instance 'box-shape
         :half-extents (cl-transforms:v*
                        (cl-transforms:make-3d-vector width length 0.01)
                        0.5)))
    ;;; Handle
      ;; the handle of the basket is formed like this:
      ;; ------------
      ;; |          |
      ;; |          |
      ;; |          |
      ;; The sides
      (dotimes (i 2)
        (add-child-shape
         compound-shape
         (cl-transforms:make-pose
          (cl-transforms:make-3d-vector (* i width) 0 (/ (+ height handle-height) 2))
          (cl-transforms:make-quaternion 0 0 0 1))
         (make-instance 'box-shape
           :half-extents (cl-transforms:v*
                          (cl-transforms:make-3d-vector 0.005 0.005 handle-height)
                          0.5))))
      ;; the top
      (add-child-shape
       compound-shape
       (cl-transforms:make-pose
        (cl-transforms:make-3d-vector (/ width 2) 0 (+ handle-height (/ height 2)))
        (cl-transforms:make-quaternion 0 0 0 1))
       (make-instance 'box-shape
         :half-extents (cl-transforms:v*
                        (cl-transforms:make-3d-vector width 0.005 0.005)
                        0.5)))
      ;; Adds the basket to the specified world
      (make-item world name (list type)
                 (list
                  (make-instance 'rigid-body
                    :name name :mass mass :pose (ensure-pose pose)
                    :collision-shape compound-shape))))))


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

(defmethod add-object ((world bt-world) (type (eql :cylinder-item)) name pose
                       &key mass (color '(0.5 0.5 0.5 1.0)) size item-type)
  (assert size)
  (assert item-type)
  (unless (object world name)
    (make-item world name (list item-type)
               (list
                (make-instance 'rigid-body
                  :name name :mass mass :pose (ensure-pose pose)
                  :collision-shape (make-instance 'bt-vis:colored-cylinder-shape
                                     :half-extents (ensure-vector size)
                                     :color color))))))

(defmethod add-object ((world bt-world) (type (eql :box-item)) name pose
                       &key mass (color '(1.0 0.0 0.0 1.0)) size item-type)
  (assert size)
  (assert item-type)
  (unless (object world name)
    (make-item world name (list item-type)
               (list
                (make-instance 'rigid-body
                  :name name :mass mass :pose (ensure-pose pose)
                  :collision-shape (make-instance 'bt-vis:colored-box-shape
                                     :half-extents (ensure-vector size)
                                     :color color))))))
