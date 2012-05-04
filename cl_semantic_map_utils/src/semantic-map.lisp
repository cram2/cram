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

(in-package :sem-map-utils)

(defvar *cached-semantic-map* nil)
(defvar *cached-owl-types* (make-hash-table :test 'equal))

(defclass semantic-map ()
  ((parts :initform (make-hash-table :test 'equal) :initarg :parts)))

(defclass semantic-map-part ()
  ((type :initarg :type :reader obj-type)
   (name :initarg :name :reader name)
   (owl-name :initarg :owl-name :reader owl-name)
   (urdf-name :initarg :urdf-link-name :reader urdf-name)
   (sub-parts :reader sub-parts)
   (aliases :initarg :aliases :reader aliases)))

(defclass semantic-map-geom (semantic-map-part)
  ((pose :initarg :pose :reader pose)
   (dimensions :initarg :dimensions :reader dimensions)))

(defclass semantic-map-joint (semantic-map-part)
  ((minimal-value :initarg :minimal-value :reader joint-minimal-value)
   (maximal-value :initarg :maximal-value :reader joint-maximal-value)
   (connected-objects :initarg :connected-objects :reader joint-connected-objects)))

(defclass semantic-map-prismatic-joint (semantic-map-joint)
  ((direction :initarg :direction :reader joint-direction)))

(defgeneric copy-semantic-map-object (obj)
  (:method ((map semantic-map))
    (make-instance 'semantic-map
      :parts (alexandria:alist-hash-table
              (mapcar (lambda (elem)
                        (destructuring-bind (key . val) elem
                          (cons key (copy-semantic-map-object val))))
                      (alexandria:hash-table-alist (slot-value map 'parts))))))

  (:method ((part semantic-map-part))
    (let* ((slots '(type name owl-name urdf-name aliases))
           (initargs '(:type :name :owl-name :urdf-link-name :aliases))
           (copy (apply #'make-instance 'semantic-map-part
                        (mapcan (lambda (initarg slot)
                                  (when (slot-boundp part slot)
                                    (list initarg (slot-value part slot))))
                                initargs slots))))
      (setf (slot-value copy 'sub-parts)
            ;; This is pretty ugly. It basically means that we always
            ;; query the complete semantic map the first time we
            ;; create it because SUB-PARTS expands everything
            (mapcar #'copy-semantic-map-object (sub-parts part)))
      copy))
  
  (:method ((geom semantic-map-geom))
    (with-slots (pose dimensions) geom
      (let ((copy (call-next-method)))
        (change-class
         copy 'semantic-map-geom
         :pose (cl-transforms:copy-pose pose)
         :dimensions (cl-transforms:copy-3d-vector dimensions)))))

  (:method ((joint semantic-map-joint))
    (with-slots (minimal-value maximal-value connected-objects)
        joint
      (let ((copy (call-next-method)))
        (change-class
         copy 'semantic-map-joint
         :minimal-value minimal-value
         :maximal-value maximal-value
         :connected-objects connected-objects))))

  (:method ((joint semantic-map-prismatic-joint))
    (with-slots (direction) joint
      (let ((copy (call-next-method)))
        (change-class
         copy 'semantic-map-prismatic-joint
         :direction direction)))))

(defgeneric semantic-map-parts (map &key recursive)
  (:method ((map semantic-map) &key recursive)
    (let ((direct-children (loop for part being the hash-values of (slot-value map 'parts)
                                 collecting part)))
      (append direct-children
              (when recursive
                (mapcan (lambda (child)
                          (semantic-map-parts child :recursive recursive))
                        direct-children)))))

  (:method ((map semantic-map-part) &key recursive)
    (let ((direct-children (sub-parts map)))
      (append direct-children
              (when recursive
                (mapcan (lambda (child)
                          (semantic-map-parts child :recursive recursive))
                        direct-children))))))

(defgeneric semantic-map-part-names (map)
  (:method ((map semantic-map))
    (loop for name being the hash-keys of (slot-value map 'parts)
          collecting name))

  (:method ((map semantic-map-part))
    (mapcar #'name (sub-parts map))))

(defgeneric semantic-map-part (map name &key recursive)
  (:method ((map semantic-map) name &key recursive)
    (or (gethash name (slot-value map 'parts))
        (loop for p-name being the hash-keys in (slot-value map 'parts)
              using (hash-value part)
              when (equal (typecase name
                            (string name)
                            (symbol (cram-roslisp-common:rosify-lisp-name name)))
                          p-name)
                do (return part))
        (when recursive
          (some (lambda (part)
                  (semantic-map-part part name :recursive recursive))
                (semantic-map-parts map)))))

  (:method ((part semantic-map-part) name &key recursive)
    (let ((name (typecase name
                  (string name)
                  (symbol (cram-roslisp-common:rosify-lisp-name name)))))
      (or
       (find name (sub-parts part)
             :key #'name
             :test #'equal)
       (when recursive
         (some (lambda (part)
                 (semantic-map-part part name :recursive recursive))
               (semantic-map-parts part)))))))

(defgeneric make-semantic-map-part (type name owl-name)
  (:method ((type t) name owlname)
    ;; TODO(moesenle): The default handling feels pretty wrong
    ;; here. We need to find a better way to encode default handling
    ;; somehow inside an owl type initializer.
    (or (run-owl-type-initializer type name owlname)
        (with-vars-bound (?pose ?dim ?labels)
            (lazy-car
             (json-prolog:prolog
              `(and ("objectPose" ,owlname ?pose)
                    ("objectDimensions" ,owlname ?w ?d ?h)
                    ("findall" ?l ("objectLabel" ,owlname ?l) ?labels)
                    (= '(?d ?w ?h) ?dim))
              :package :sem-map-utils))
          (let ((aliases (unless (is-var ?labels)
                           ?labels)))
            (if (or (is-var ?pose) (is-var ?dim))
                (make-instance 'semantic-map-part
                  :type type :name name :owl-name owlname
                  :aliases (mapcar (lambda (label)
                                     (remove #\' (symbol-name label)))
                                   aliases))
                (make-instance 'semantic-map-geom
                  :type type
                  :name name
                  :owl-name owlname
                  :pose (cl-transforms:transform->pose
                         (cl-transforms:matrix->transform
                          (make-array
                           '(4 4) :displaced-to (make-array
                                                 16 :initial-contents ?pose))))
                  :dimensions (apply #'cl-transforms:make-3d-vector ?dim)
                  :aliases (mapcar (lambda (label)
                                     (remove #\' (symbol-name label)))
                                   aliases))))))))

(defgeneric update-pose (obj new-pose &key relative recursive)
  (:documentation "Updates the pose of `obj' using `new-pose'. When
  `relative' is T, `new-pose' is interpreted as relative offset to the
  curren pose of`obj', otherwise `new-pose' indicates the new
  pose. When `recursive' is T (default), the pose of all sub-objects
  of `obj' is updated, too.")
  (:method ((obj semantic-map-geom) new-pose &key (relative nil) (recursive t))
    (cond (relative
           (setf (pose obj) (cl-transforms:transform-pose
                             (cl-transforms:reference-transform new-pose)
                             (pose obj)))
           (when recursive
             (dolist (sub (sub-parts obj))
               (update-pose sub new-pose :recursive t :relative t))))
          (recursive
           (let ((relative-offset (cl-transforms:transform*
                                   (cl-transforms:reference-transform new-pose)
                                   (cl-transforms:transform-inv
                                    (cl-transforms:reference-transform (pose obj))))))
             (setf (pose obj) new-pose)
             (dolist (sub (sub-parts obj))
               (update-pose sub relative-offset :recursive t :relative t))))
          (t (setf (pose obj) new-pose))))
  (:method ((obj semantic-map-part) new-pose &key relative (recursive t))
    (when recursive
      (dolist (sub (sub-parts obj))
        (update-pose sub new-pose :relative relative :recursive t)))))

(defgeneric (setf pose) (new-value geom)
  (:method (new-value (geom semantic-map-geom))
    (with-slots (pose) geom
      ;; TODO: update joint-state in knowrob's knowledge base
      (setf pose new-value))))

(defmethod sub-parts :before ((part semantic-map-part))
  (unless (slot-boundp part 'sub-parts)
    (setf (slot-value part 'sub-parts)
          (force-ll
           (lazy-mapcan
            (lambda (bdgs)
              (with-vars-bound (?type ?name ?sub) bdgs
                (unless (or (is-var ?type) (is-var ?sub))
                  (list
                   (make-semantic-map-part
                    (remove #\' (symbol-name ?type))
                    (remove #\' (symbol-name ?name))
                    (remove #\' (symbol-name ?sub)))))))
            (json-prolog:prolog
             `(and
               ("rdf_has" ,(owl-name part) "http://ias.cs.tum.edu/kb/knowrob.owl#properPhysicalParts" ?sub)
               ("objectType" ?sub ?tp)
               ("rdf_atom_no_ns" ?tp ?type)
               ("rdf_atom_no_ns" ?sub ?name))
             :package :sem-map-utils))))))

(defmethod urdf-name :before ((part semantic-map-part))
  (unless (slot-boundp part 'urdf-name)
    (with-slots (owl-name urdf-name) part
      (let ((label (var-value
                    '?link
                    (lazy-car (json-prolog:prolog
                               `("rdf_has"
                                 ,owl-name "http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName"
                                 ("literal" ?link))
                               :package :sem-map-utils)))))
        (if (is-var label)
            (setf urdf-name nil)
            (setf urdf-name (remove #\' (symbol-name label))))))))

(defun urdf-name->obj-name (urdf-name)
  (with-vars-bound (?name)
      (lazy-car
       (json-prolog:prolog
        `(and
          ("rdf_has"
           ?owlname "http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName"
           ("literal" ,urdf-name))
          ("rdf_atom_no_ns" ?owlname ?name))
        :package :sem-map-utils))
    (unless (is-var ?name)
      (remove #\' (symbol-name ?name)))))

(defun clear-semantic-map-cache ()
  (setf *cached-semantic-map* nil))

(defmacro with-clear-semantic-map-cache (&body body)
  `(let ((*cached-semantic-map* nil))
     ,@body))

(defmacro with-semantic-map-cache (cache &body body)
  `(let ((*cached-semantic-map* ,cache))
     ,@body))

(defun init-semantic-map-cache ()
  (unless (or (not (json-prolog:check-connection)) *cached-semantic-map*)
    (setf *cached-semantic-map*
          (make-instance 'semantic-map
            :parts (alexandria:alist-hash-table
                    (mapcar (lambda (elem)
                              (cons (name elem) elem))
                            (force-ll
                             (lazy-mapcan
                              (lambda (bdgs)
                                (with-vars-bound (?type ?n ?o) bdgs
                                  (unless (or (is-var ?type) (is-var ?o))
                                    (list (make-semantic-map-part
                                           (remove #\' (symbol-name ?type))
                                           (remove #\' (symbol-name ?n))
                                           (remove #\' (symbol-name ?o)))))))
                              (json-prolog:prolog
                               '(and ("rootObjects" ?objs)
                                 ("member" ?o ?objs)
                                 ("objectType" ?o ?tp)
                                 ("rdf_atom_no_ns" ?tp ?type)
                                 ("rdf_atom_no_ns" ?o ?n))
                               :package :sem-map-utils))))
                    :test 'equal)))))

(defun get-semantic-map ()
  (init-semantic-map-cache)
  (when *cached-semantic-map*
    (copy-semantic-map-object *cached-semantic-map*)))

(defun owl-names-equal (lhs rhs)
  (let ((lhs (etypecase lhs
               (symbol (remove #\' (symbol-name lhs)))
               (string lhs)))
        (rhs (etypecase rhs
               (symbol (remove #\' (symbol-name rhs)))
               (string rhs))))
    (equal lhs rhs)))

(defun owl-type-p (type ref-type)
  "Checks if `type' is equal to `ref-type', i.e. if it's a sub-type of
  `ref-type'"
  (flet ((cached-sub-types (ref-type)
           (declare (type string ref-type))
           (or (gethash ref-type *cached-owl-types*)
               (let* ((type-namespace-str "http://ias.cs.tum.edu/kb/knowrob.owl#")
                      (ref-type-w/ns-str (concatenate
                                          'string
                                          type-namespace-str
                                          ref-type)))
                 (setf (gethash ref-type *cached-owl-types*)
                       (remove-duplicates
                        (force-ll
                         (lazy-mapcar (lambda (bdg)
                                        (with-vars-bound (?type) bdg
                                          (let ((type-str (remove #\' (symbol-name ?type))))
                                            (subseq type-str (1+ (position #\# type-str))))))
                                      (json-prolog:prolog
                                       `(or ("owl_subclass_of" ,ref-type-w/ns-str ?type)
                                            ("owl_subclass_of" ?type ,ref-type-w/ns-str))
                                       :package :sem-map-utils)))
                        :test #'equal))))))
    (let ((ref-type-str (etypecase ref-type
                          (symbol (remove #\' (symbol-name ref-type)))
                          (string ref-type)))
          (type-str (etypecase type
                      (symbol (remove #\' (symbol-name type)))
                      (string type))))
      (find type-str (cached-sub-types ref-type-str)
            :test #'equal))))

(defun sub-parts-with-type (map type &key (recursive t))
  "Returns a lazy list of all objects of type `type' that are children
of map. When `recursive' is T, recursively traverses all sub-parts, i.e. returns not only direct children."
  ;; Update the cache if not updated yet
  (let ((type (etypecase type
                (symbol (cram-roslisp-common:rosify-lisp-name type))
                (string type))))
    (lazy-mapcan (lambda (part)
                   (lazy-append
                    (when (owl-type-p (obj-type part) type)
                      (list part))
                    (when recursive
                      (sub-parts-with-type part type))))
                 (semantic-map-parts map))))

(defun sub-parts-with-name (map name &key (recursive t))
  "Returns a lazy list of all objects of type `type' that are children
of map. When `recursive' is T, recursively traverses all sub-parts, i.e. returns not only direct children."
  ;; Update the cache if not updated yet
  (let ((name (etypecase name
                (symbol (cram-roslisp-common:rosify-lisp-name name))
                (string name))))
    (lazy-mapcan (lambda (part)
                   (lazy-append
                    (when (or (owl-names-equal (owl-name part) name)
                              (owl-names-equal (name part) name)
                              (member name (aliases part) :test #'owl-names-equal))
                      (list part))
                    (when recursive
                      (sub-parts-with-name part name))))
                 (semantic-map-parts map))))

(def-owl-type-initializer ("PrismaticJoint" name owl-name)
  (with-vars-bound (?min ?max ?connected ?labels ?directionx ?directiony ?directionz)
      (car
       (json-prolog:prolog-1
        `(and 
          ("rdf_has" ,owl-name "http://ias.cs.tum.edu/kb/knowrob.owl#minJointValue"
                     ("literal" ("type" ?_ ?min_)))
          ("rdf_has" ,owl-name "http://ias.cs.tum.edu/kb/knowrob.owl#maxJointValue"
                     ("literal" ("type" ?_ ?max_)))
          ("findall" ?c (and
                         ("rdf_has"
                          ,owl-name
                          "http://ias.cs.tum.edu/kb/knowrob.owl#connectedTo-Rigidly"
                          ?c_)
                         ("rdf_atom_no_ns" ?c_ ?c))
                     ?connected)
          ("rdf_has" "http://ias.cs.tum.edu/kb/knowrob.owl#Slider46"
                     "http://ias.cs.tum.edu/kb/knowrob.owl#direction" ?direction)
          ("rdf_has" ?direction "http://ias.cs.tum.edu/kb/knowrob.owl#vectorX"
                     ("literal" ("type" ?_ ?direction_x_)))
          ("rdf_has" ?direction "http://ias.cs.tum.edu/kb/knowrob.owl#vectorY"
                     ("literal" ("type" ?_ ?direction_y_)))
          ("rdf_has" ?direction "http://ias.cs.tum.edu/kb/knowrob.owl#vectorZ"
                     ("literal" ("type" ?_ ?direction_z_)))
          ("atom_number" ?min_ ?min) ("atom_number" ?max_ ?max)
          ("atom_number" ?direction_x_ ?directionx)
          ("atom_number" ?direction_y_ ?directiony)
          ("atom_number" ?direction_z_ ?directionz)
          ("findall" ?l ("objectLabel" ,owl-name ?l) ?labels))
        :package :sem-map-utils))
    (make-instance 'semantic-map-prismatic-joint
      :type "PrismaticJoint"
      :name name
      :owl-name owl-name
      :direction (unless (or (is-var ?directionx) (is-var ?directiony)
                             (is-var ?directionz))
                   (cl-transforms:make-3d-vector
                    ?directionx ?directiony ?directionz))
      :minimal-value (unless (is-var ?min) ?min)
      :maximal-value (unless (is-var ?max) ?max)
      :connected-objects (mapcar (lambda (name-symbol)
                                   (remove #\' (symbol-name name-symbol)))
                                 (unless (is-var ?connected) ?connected))
      :aliases (mapcar (lambda (label)
                         (remove #\' (symbol-name label)))
                       (unless (is-var ?labels) ?labels)))))

(def-owl-type-initializer ("HingedJoint" name owl-name)
  (with-vars-bound (?min ?max ?connected ?labels)
      (car
       (json-prolog:prolog-1
        `(and 
          ("rdf_has" ,owl-name "http://ias.cs.tum.edu/kb/knowrob.owl#minJointValue"
                     ("literal" ("type" ?_ ?min_)))
          ("rdf_has" ,owl-name "http://ias.cs.tum.edu/kb/knowrob.owl#maxJointValue"
                     ("literal" ("type" ?_ ?max_)))
          ("findall" ?c (and
                         ("rdf_has"
                          ,owl-name
                          "http://ias.cs.tum.edu/kb/knowrob.owl#connectedTo-Rigidly"
                          ?c_)
                         ("rdf_atom_no_ns" ?c_ ?c))
                     ?connected)
          ("atom_number" ?min_ ?min) ("atom_number" ?max_ ?max)
          ("findall" ?l ("objectLabel" ,owl-name ?l) ?labels))
        :package :sem-map-utils))
    (make-instance 'semantic-map-joint
      :type "HingedJoint"
      :name name
      :owl-name owl-name
      :minimal-value (unless (is-var ?min) ?min)
      :maximal-value (unless (is-var ?max) ?max)
      :connected-objects (mapcar (lambda (name-symbol)
                                   (remove #\' (symbol-name name-symbol)))
                                 (unless (is-var ?connected) ?connected))
      :aliases (mapcar (lambda (label)
                         (remove #\' (symbol-name label)))
                       (unless (is-var ?labels) ?labels)))))
