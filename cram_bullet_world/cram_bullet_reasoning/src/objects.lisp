;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(defparameter *all-meshes-as-compound* T)

(defclass object ()
  ((name :initarg :name :reader name :type keyword)
   (rigid-bodies :initform (make-hash-table :test 'equal))
   (pose-reference-body :initarg :pose-reference-body
                        :documentation "The name of the rigid-body
                        that is used for returning the pose of the
                        object. It defaults to the first body that got
                        added.")
   (world :initarg :world :reader world)))


(defun make-object (world name &optional
                                 bodies (add-to-world t))
  (make-instance 'object
    :name name
    :world world
    :rigid-bodies bodies
    :add add-to-world))


(defgeneric add-object (world type name pose &key &allow-other-keys)
  (:documentation "Adds an object of `type' named `name' to the bullet
  world `world'. The object is placed at `pose'"))

(defgeneric remove-object (world name)
  (:documentation "Removes the object named `name' from the `world'")
  (:method ((world bt-reasoning-world) name)
    (let ((obj (object world name)))
      (cond (obj
             (mapcar (lambda (body) (remove-rigid-body world body))
                     (rigid-bodies obj))
             (remhash name (slot-value world 'objects))
             (setf (slot-value obj 'world) nil))
            (t (warn 'simple-warning
                     :format-control "Could not find a body named `~a'"
                     :format-arguments (list name)))))))


(defgeneric rigid-bodies (obj)
  (:documentation "Return the list of rigid bodies that belong to the object `obj'")
  (:method ((obj object))
    (loop for name being the hash-keys in (slot-value obj 'rigid-bodies)
            using (hash-value body)
          collecting (or body (rigid-body obj name)))))

(defgeneric rigid-body-names (obj)
  (:documentation "Return the list of rigid bodies that belong to the object `obj'")
  (:method ((obj object))
    (loop for body being the hash-keys in (slot-value obj 'rigid-bodies)
          collecting body)))

(defgeneric rigid-body (obj name)
  (:documentation "Returns the rigid body named `name' or NIL if the body doesn't exist")
  (:method ((obj object) name)
    (with-slots (world rigid-bodies) obj
      (multiple-value-bind (value valid)
          (gethash name rigid-bodies)
        (when valid
          (or value
              (setf (gethash name rigid-bodies)
                    (or
                     (find name (bodies world)
                           :key #'name
                           :test #'equal)
                     (error 'simple-error
                            :format-control "Could not find body with name `~a'"
                            :format-arguments (list name))))))))))


(defgeneric copy-object (obj world)
  (:documentation "Copies the object `obj' and makes it an object of world `world'")
  (:method ((obj object) (world bt-reasoning-world))
    (with-slots (name rigid-bodies pose-reference-body) obj
      (let ((new-instance
              (make-instance
                  'object :name name
                :pose-reference-body pose-reference-body
                :world world)))
        (prog1 new-instance
          (setf (slot-value new-instance 'rigid-bodies)
                (copy-hash-table rigid-bodies)))))))


(defgeneric initialize-rigid-bodies (object rigid-bodies &key add)
  (:method ((object object) rigid-bodies &key (add t))
    (with-slots (world) object
      (declare (type list rigid-bodies))
      (assert (eql (hash-table-count (slot-value object 'rigid-bodies)) 0))
      (assert rigid-bodies)
      (dolist (body rigid-bodies)
        (when (and add world)
          (when (typep world 'bt-reasoning-world)
            (setf (gethash (name object) (slot-value world 'objects))
                  object))
          (add-rigid-body world body))
        (setf (gethash (name body) (slot-value object 'rigid-bodies))
              body)))))


(defmethod initialize-instance :after ((object object)
                                       &key rigid-bodies pose-reference-body
                                         (add t))
  (when rigid-bodies
    (initialize-rigid-bodies object rigid-bodies :add add)
    (unless pose-reference-body
      (setf (slot-value object 'pose-reference-body) (name (car rigid-bodies))))))

(defmethod invalidate-object :after ((obj object))
  (with-slots (rigid-bodies) obj
    ;; Sets each value of the rigid-bodies slot to nil.
    ;; Don't access rigid bodies directly through this hash table, instead
    ;; use btr:rigid-body function.
    ;; When you use the function, the hash table entry will be reset.
    (loop for key being the hash-keys in rigid-bodies do
      (setf (gethash key rigid-bodies) nil))))


(defmethod pose ((object object))
  "Returns the pose of the object, i.e. the pose of the body named by
  the slot `pose-reference-body'"
  (let ((body (rigid-body object (slot-value object 'pose-reference-body))))
    (when body
      (pose body))))

(defmethod (setf pose) (new-value (object object))
  (let ((body (rigid-body object (slot-value object 'pose-reference-body))))
    (when body
      (setf (pose body) new-value))))

(defun set-object-pose (object new-pose)
  (setf (pose object) (ensure-pose new-pose)))


(defmethod draw ((context gl-context) (object object))
  (dolist (body (rigid-bodies object))
    (draw context body)))


(defun make-rigid-body-name (obj-name body-name)
  (flet ((ensure-string (name)
           (etypecase name
             (string name)
             (symbol (symbol-name name)))))
    (intern (concatenate
             'string
             (ensure-string obj-name)
             "."
             (ensure-string body-name))
            :keyword)))


(defmethod add-object ((world bt-reasoning-world) type name pose
                       &key disable-collisions-with)
  (prog1
      (call-next-method)
    (when disable-collisions-with
      (disable-collisions world name disable-collisions-with))))


;;;;;;;;;;;;;;;;;;;;;;;;; SPAWNING PRIMITIVE-SHAPED OBJECTS ;;;;;;;;;;;;;;;

(defmethod add-object ((world bt-world) (type (eql :box)) name pose &key mass size)
  (destructuring-bind (size-x size-y size-z) size
    (make-object world name
                 (list
                  (make-instance
                      'rigid-body
                    :name name :mass mass :pose (ensure-pose pose)
                    :collision-shape (make-instance
                                         'box-shape
                                       :half-extents (cl-transforms:make-3d-vector
                                                      (/ size-x 2)
                                                      (/ size-y 2)
                                                      (/ size-z 2))))))))

(defmethod add-object ((world bt-world) (type (eql :colored-box)) name pose
                       &key mass size color)
  (destructuring-bind (size-x size-y size-z) size
    (make-object world name
                 (list
                  (make-instance
                      'rigid-body
                    :name name :mass mass :pose (ensure-pose pose)
                    :collision-shape (make-instance
                                         'colored-box-shape
                                       :half-extents (cl-transforms:make-3d-vector
                                                      (/ size-x 2)
                                                      (/ size-y 2)
                                                      (/ size-z 2))
                                       :color color))))))

(defmethod add-object ((world bt-world) (type (eql :static-plane)) name pose
                       &key normal constant (collision-mask '(:default-filter)))
  (destructuring-bind (normal-x normal-y normal-z) normal
    (make-object world name
                 (list
                  (make-instance
                      'rigid-body
                    :name name :pose (ensure-pose pose)
                    :group :static-filter
                    :mask collision-mask
                    :collision-shape (make-instance
                                         'textured-static-plane-shape
                                       :normal (cl-transforms:make-3d-vector
                                                normal-x normal-y normal-z)
                                       :constant constant
                                       ;; :width 16 :height 16
                                       ;; :texture (texture-str->bitmap
                                       ;;           *static-plane-texture* #\Space)
                                       :width 32 :height 32
                                       :texture (texture-str->bitmap
                                                 *static-plane-gray-texture*
                                                 #\Space
                                                 nil
                                                 #\g
                                                 '(0.6 0.6 0.6 1.0))))))))

(defmethod add-object ((world bt-world) (type (eql :sphere)) name pose
                       &key mass radius color)
  (make-object world name
               (list
                (make-instance
                    'rigid-body
                  :name name :mass mass :pose (ensure-pose pose)
                  :collision-shape (make-instance 'colored-sphere-shape
                                     :radius radius :color color)))))

(defmethod add-object ((world bt-world) (type (eql :cylinder)) name pose &key mass size)
  (destructuring-bind (size-x size-y size-z) size
    (make-object world name
                 (list
                  (make-instance
                      'rigid-body
                    :name name :mass mass :pose (ensure-pose pose)
                    :collision-shape (make-instance 'cylinder-shape
                                       :half-extents (cl-transforms:make-3d-vector
                                                      (/ size-x 2)
                                                      (/ size-y 2)
                                                      (/ size-z 2))))))))

(defmethod add-object ((world bt-world) (type (eql :cone)) name pose &key
                                                                       mass radius height)
  (make-object world name
               (list
                (make-instance
                    'rigid-body
                  :name name :mass mass :pose (ensure-pose pose)
                  :collision-shape (make-instance 'cone-shape
                                     :radius radius
                                     :height height)))))

(defmethod add-object ((world bt-world) (type (eql :point-cloud)) name pose &key points)
  (make-object world name
               (list
                (make-instance
                    'rigid-body
                  :name name :mass 0.0 :pose (ensure-pose pose)
                  :collision-shape (make-instance 'convex-hull-shape :points points)))))


;;;;;;;;;;;;;;;;;;;;;;; MESH LOADING UTILS ;;;;;;;;;;;;;;;;;;;;;;;;

(defun load-mesh (mesh-filename &key (scale nil) (size nil)
                                  (compound *all-meshes-as-compound*)
                                  (flip-winding-order nil))
  "Loads and resizes the 3d-model. If `compound' is T we have a list of meshes, instead of one."
  (let* ((uri
           (physics-utils:parse-uri mesh-filename))
         (mesh-model
           (cut:with-file-cache model uri
               (multiple-value-list
                (physics-utils:load-3d-model
                 uri :compound compound :flip-winding-order flip-winding-order))
             model)))
    (cond (scale
           (mapcar (lambda (model-part)
                     (physics-utils:scale-3d-model model-part scale))
                   mesh-model))
          (size
           (mapcar (lambda (model-part)
                     (physics-utils:resize-3d-model model-part size))
                   mesh-model))
          (t
           mesh-model))))

(defun make-collision-shape-from-mesh (mesh-filename &key (color '(0.8 0.8 0.8 1.0))
                                                       (scale nil) (size nil)
                                                       (compound *all-meshes-as-compound*)
                                                       (disable-face-culling nil)
                                                       (flip-winding-order nil))
  "Loads the meshes from the specified filename and creates either:
      a `convex-hull-shape' if `compound' is NIL or
      a `compound-shape' if compound it T.
  The former combines all meshes and faces into one convex-hull-shape, while the latter
  contains every single mesh as a seperate convex-hull-shape as children in a compound-shape."
  (flet ((make-ch-mesh-shape (model-part)
           (make-instance 'convex-hull-mesh-shape
             :color color
             :disable-face-culling disable-face-culling
             :faces (physics-utils:3d-model-faces model-part)
             :points (physics-utils:3d-model-vertices model-part))))
    (let ((model (load-mesh mesh-filename
                            :scale scale
                            :size size
                            :compound compound
                            :flip-winding-order flip-winding-order)))
      ;; model has multiple components, such that it makes sense to make a compound shape
      (if (and compound (> (length model) 1))
          (let ((compound-shape (make-instance 'compound-shape))
                (id-pose (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector 0 0 0)
                          (cl-transforms:make-identity-rotation))))
            (mapcar (alexandria:compose
                     (alexandria:curry #'add-child-shape compound-shape id-pose)
                     #'make-ch-mesh-shape)
                    model)
            compound-shape)
          (make-ch-mesh-shape (car model))))))


;;;;;;;;;;;;;;;;;;;;;;;; OBJECT ATTACHMENTS ;;;;;;;;;;;;;;;;;;;;;;

(defstruct collision-information
  rigid-body-name flags)

(defun get-collision-information (object other-object)
  "Collision information about `object' from `other-object',
who is the parent of `object' in the attachment."
  (cdr (cdr (assoc (name object) (attached-objects other-object)))))

(defun create-static-collision-information (object)
  (if (not (object *current-bullet-world* (name object)))
      (error "Cannot find object named ~a" (name object))
      (loop for body in (rigid-bodies object)
            collecting (make-collision-information
                        :rigid-body-name (name body)
                        :flags (cond (;; If the object does have no
                                      ;; attached objects we take the
                                      ;; current state of it.
                                      (not (attached-objects object))
                                      (collision-flags body))
                                     ;; If the object is attached with
                                     ;; other objects we use the saved
                                     ;; collision information from
                                     ;; these objects. If the
                                     ;; collision information of all
                                     ;; these objects is static, this
                                     ;; returns static too.
                                     ((only-static-in-attachments-p object)
                                      '(:cf-static-object))
                                     (t
                                      NIL)))
            do (setf (collision-flags body) :cf-static-object))))

(defun only-static-in-attachments-p (object)
  "Returns T, if the `object's flags are `cf-static-object' in
  the attachments of `object's attached objects."
  (declare (type object object))
  (let* ((attached-object-names
           (mapcar #'car
                   (attached-objects
                    (object
                     *current-bullet-world*
                     (name object)))))
         (attachments-of-attached-objects
           (mapcar
            (alexandria:compose
             #'attached-objects
             (alexandria:curry #'object *current-bullet-world*))
           attached-object-names))
         (attachments-to-object
           (mapcar 
            #'car
            (loop for attachments in attachments-of-attached-objects
                  collecting
                  (remove-if-not (alexandria:curry 
                                  #'equalp
                                  (name object))
                                 attachments :key #'car))))
         (collision-information-list
           (mapcar
            #'caddr
            (remove-if-not #'identity
                           attachments-to-object))))

    (when collision-information-list
      (every (alexandria:curry #'equalp '(:cf-static-object))
             (mapcar #'collision-information-flags
                     collision-information-list)))))


(defun reset-collision-information (object collision-information)
  (loop for collision-data in collision-information
        for body = (rigid-body
                    object (collision-information-rigid-body-name
                            collision-data))
        do (setf (collision-flags body)
                 (if (attached-objects object)
                     '(:cf-static-object)
                     (collision-information-flags collision-data)))))


(defstruct attachment
  "Represents a link between an object and another object or its link.
`object' must be an instance of class OBJECT.
`link' must be a string, the name of the link.
If `loose' is non-NIL, it means that if the link moves, the pose
of the object should _not_ be updated. `grasp' is the type of grasp orientation.
`attachment' is the type of the attachment."
  (object nil :type (or symbol string))
  (link "" :type string)
  (loose nil :type (or nil t))
  (grasp nil :type (or null keyword))
  (attachment nil :type (or null keyword)))

(defgeneric attach-object (object-to-attach-to object &key &allow-other-keys)
  (:documentation "Adds `object' to the set of attached objects of `object-to-attach-to'."))

(defgeneric detach-object (object-to-detach-from object &key &allow-other-keys)
  (:documentation "Removes `object' from the attached objects of `object-to-detach-from'."))

(defgeneric detach-all-from-link (object link)
  (:documentation "Removes all attachments form the given `link' of `object'."))

(defgeneric detach-all-objects (object)
  (:documentation "Removes all attachments form the list of attached objects of `object'."))

(defmethod attach-object ((object-to-attach-to-name symbol) (object-name symbol)
                          &key attachment-type loose skip-removing-loose link grasp)
  "Attaches object named `object-name' to another object named `object-to-attach-to-name'."
  (multiple-value-bind (obj obj-found)
      (btr:object *current-bullet-world* object-name)
    (multiple-value-bind (other-obj other-obj-found)
        (btr:object *current-bullet-world* object-to-attach-to-name)
      (when (and obj-found other-obj-found)
        (attach-object other-obj obj
                       ;; merged keywords from items.lisp and robot-model.lisp
                       :attachment-type attachment-type :loose loose
                       :skip-removing-loose skip-removing-loose :link link
                       :grasp grasp)))))

(defmethod attach-object ((object-to-attach-to-names list) (object-name symbol)
                          &key attachment-type loose skip-removing-loose link grasp)
  "Attaches object named `object-name' to other objects,
the names of which are in `object-to-attach-names'."
  (multiple-value-bind (obj obj-found)
      (btr:object *current-bullet-world* object-name)
    (when obj-found
      (attach-object
       (remove-if-not #'identity
                      (mapcar (alexandria:curry #'object *current-bullet-world*)
                              object-to-attach-to-names))
       obj
       :attachment-type attachment-type :loose loose
       :skip-removing-loose skip-removing-loose :link link
       :grasp grasp))))

(defmethod detach-object ((object-to-detach-from-name symbol) (object-name symbol) &key)
  "Detaches object named `object-name' from another object named `object-to-detach-from-name'."
  (multiple-value-bind (obj obj-found)
      (btr:object *current-bullet-world* object-name)
    (multiple-value-bind (other-obj other-obj-found)
        (btr:object *current-bullet-world* object-to-detach-from-name)
      (when (and obj-found other-obj-found)
        (detach-object obj other-obj)))))

(defmethod detach-all-from-link ((object-to-detach-from-name symbol) link)
  "Detaches objects from object named `object-to-detach-from-name'."
  (multiple-value-bind (obj obj-found)
      (btr:object *current-bullet-world* object-to-detach-from-name)
    (when obj-found
      (detach-all-from-link obj link))))

(defmethod detach-all-objects ((object-to-detach-from-name symbol))
  "Detaches objects from object named `object-to-detach-from-name'."
  (multiple-value-bind (obj obj-found)
      (btr:object *current-bullet-world* object-to-detach-from-name)
    (when obj-found
      (detach-all-objects obj))))

(defun get-loose-attached-objects (object)
  "Returns all objects attached to `object',
where ATTACHMENTs have the keyword LOOSE as not NIL."
  (mapcar #'car
          (remove-if-not
           (alexandria:compose #'attachment-loose #'car #'car #'cdr)
           (attached-objects object))))

(let ((already-visited '()))
  (defun remove-loose-attachment-for (object)
    "Searches if the `object' was connected loosely to other
objects and removes ALL corresponding attachments if so.
To search through the attached objects of `object' the variable
ALREADY-VISITED will help to prevent endless loops, as this is a
recursive function."
    (let ((loose-attached-objects (get-loose-attached-objects object)))
      (when loose-attached-objects
        ;; Map the following: (detach-object object loosely-attached-object)
        (mapcar (alexandria:curry #'detach-object object)
                (mapcar (alexandria:curry #'object *current-bullet-world*)
                        loose-attached-objects))))
    ;; searching recursivly:
    ;; if `object' has attachments, `remove-loose-attachment-for'
    ;; gets called with these to remove every indirect loose
    ;; attachment: e. g. `object' is not loosely attached but one of
    ;; its attached objects is connected loosely to something
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
