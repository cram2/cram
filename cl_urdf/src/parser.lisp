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

(in-package :cl-urdf)

(define-condition urdf-type-not-supported (simple-warning) ())
(define-condition urdf-attribute-not-supported (simple-warning) ())
(define-condition urdf-malformed (simple-error) ())

(defgeneric parse-xml-node (name node &optional robot)
  (:method (name node &optional robot)
    (declare (ignore node robot))
    (warn 'urdf-type-not-supported
          :format-control "URDF element type `~a' not supported. Ignoring it."
          :format-arguments (list name))))

(defun read-fields (str)
  "Returns a list of space-separated fields in `str'. This function uses
  the Lisp reader for parsing."
  (let ((*read-eval* nil))
    (read-from-string (concatenate 'string "(" str ")"))))

(defun read-triple (str)
  (let ((fields (read-fields str)))
    (assert (eql (list-length fields) 3))
    fields))

(defun read-number (str)
  (let ((*read-eval* nil))
    (let ((value
           (read-from-string str)))
      (assert (typep value 'number))
      value)))

(defun xml-element-child (node name)
  (when (s-xml:xml-element-p (car (s-xml:xml-element-children node)))
    (find name (s-xml:xml-element-children node)
          :key #'s-xml:xml-element-name)))

(defun parse-child-node (node name &optional robot)
  (let ((child-node (xml-element-child node name)))
    (when child-node
      (parse-xml-node (s-xml:xml-element-name child-node)
                      child-node robot))))

(defmethod parse-xml-node ((name (eql :|robot|)) node &optional robot)
  (declare (ignore robot))
  (labels ((find-root-link (link)
             (if (from-joint link)
                 (find-root-link (parent (from-joint link)))
                 link)))
    (let ((robot (make-instance
                  'robot
                  :name (s-xml:xml-element-attribute node :|name|)))
          (material-descriptions nil)
          (link-descriptions nil)
          (joint-descriptions nil)
          (other nil))
      ;; First we need to load materials, then links and finally joints
      (dolist (child (s-xml:xml-element-children node))
        (case (s-xml:xml-element-name child)
          (:|material| (push child material-descriptions))
          (:|link| (push child link-descriptions))
          (:|joint| (push child joint-descriptions))
          (t (push child other))))
      (with-slots (materials links joints root-link) robot
        (dolist (child material-descriptions)
          (setf (gethash (s-xml:xml-element-attribute child :|name|) materials)
                (parse-xml-node :|material| child robot)))
        (dolist (child link-descriptions)
          (setf (gethash (s-xml:xml-element-attribute child :|name|) links)
                (parse-xml-node :|link| child robot)))
        (dolist (child joint-descriptions)
          (setf (gethash (s-xml:xml-element-attribute child :|name|) joints)
                (parse-xml-node :|joint| child robot)))
        (setf root-link (find-root-link (with-hash-table-iterator (it-fn links)
                                          (nth-value 2 (it-fn))))))
      (dolist (child other robot)
        (parse-xml-node (s-xml:xml-element-name child) child robot)))))

(defmethod parse-xml-node ((name (eql :|material|)) node &optional robot)
  (flet ((parse-color (color-node)
           (when color-node
             (read-fields (s-xml:xml-element-attribute color-node :|rgba|))))
         (parse-texture (texture-node)
           (when texture-node
             (s-xml:xml-element-attribute texture-node :|filename|))))
    (or (gethash (s-xml:xml-element-attribute node :|name|) (materials robot))
        (make-instance
         'material
         :name (s-xml:xml-element-attribute node :|name|)
         :color (or (parse-color (xml-element-child node :|color|))
                    '(0.8 0.8 0.8 1.0))
         :texture (parse-texture (xml-element-child node :|texture|))))))

(defmethod parse-xml-node ((name (eql :|link|)) node &optional robot)
  (let ((inertial (xml-element-child node :|inertial|))
        (visual (xml-element-child node :|visual|))
        (collision (xml-element-child node :|collision|)))
    (make-instance
     'link
     :name (s-xml:xml-element-attribute node :|name|)
     :inertial (when inertial
                 (parse-xml-node :|inertial| inertial robot))
     :visual (when visual
               (parse-xml-node :|visual| visual robot))
     :collision (when collision
                  (parse-xml-node :|collision| collision robot)))))

(defmethod parse-xml-node ((name (eql :|inertial|)) node &optional robot)
  (let ((mass-node (xml-element-child node :|mass|))
        (origin-node (xml-element-child node :|origin|))
        (inertial (make-instance 'inertial)))
    (with-slots (mass origin) inertial
      (when mass-node
        (setf mass (parse-xml-node :|mass| mass-node robot)))
      (when origin-node
        (setf origin (parse-xml-node :|origin| origin-node robot))))
    inertial))

(defmethod parse-xml-node ((name (eql :|visual|)) node &optional robot)
  (let ((origin-node (xml-element-child node :|origin|))
        (material-node (xml-element-child node :|material|))
        (visual (make-instance
                 'visual
                 :geometry (parse-xml-node
                            :|geometry|
                            (xml-element-child node :|geometry|)
                            robot))))
    (with-slots (origin material) visual
      (when origin-node
        (setf origin (parse-xml-node :|origin| origin-node robot)))
      (when material-node
        (setf material (parse-xml-node :|material| material-node robot))))
    visual))

(defmethod parse-xml-node ((name (eql :|collision|)) node &optional robot)
  (let ((origin-node (xml-element-child node :|origin|))
        (collision (make-instance
                    'collision
                    :geometry (parse-xml-node
                               :|geometry|
                               (xml-element-child node :|geometry|)
                               robot))))
    (with-slots (origin material) collision
      (when origin-node
        (setf origin (parse-xml-node :|origin| origin-node robot))))
    collision))

(defmethod parse-xml-node ((name (eql :|geometry|)) node &optional robot)
  ;; The geometry node is supposed to have only one child node
  (let ((geom-node (car (s-xml:xml-element-children node))))
    (parse-xml-node (s-xml:xml-element-name geom-node) geom-node robot)))

(defmethod parse-xml-node ((name (eql :|box|)) node &optional robot)
  (declare (ignore robot))
  (make-instance 'box :size (apply
                             #'cl-transforms:make-3d-vector
                             (read-triple
                              (s-xml:xml-element-attribute node :|size|)))))

(defmethod parse-xml-node ((name (eql :|cylinder|)) node &optional robot)
  (declare (ignore robot))
  (make-instance
   'cylinder
   :radius (read-number (s-xml:xml-element-attribute node :|radius|))
   :length (read-number (s-xml:xml-element-attribute node :|length|))))

(defmethod parse-xml-node ((name (eql :|sphere|)) node &optional robot)
  (declare (ignore robot))
  (make-instance
   'sphere
   :radius (read-number (s-xml:xml-element-attribute node :|radius|))))

(defmethod parse-xml-node ((name (eql :|mesh|)) node &optional robot)
  (declare (ignore robot))
  (let ((scale (read-fields (s-xml:xml-element-attribute node :|scale|))))
    (make-instance
        'mesh
      :filename (s-xml:xml-element-attribute node :|filename|)
      :scale (case (list-length scale)
               (1 (first scale))
               (3 (apply #'cl-transforms:make-3d-vector scale)))
      :size (s-xml:xml-element-attribute node :|size|))))

(defmethod parse-xml-node ((name (eql :|joint|)) node &optional robot)
  (let* ((axis-node (xml-element-child node :|axis|))
         (origin-node (xml-element-child node :|origin|))
         (limits-node (xml-element-child node :|limit|))
         (parent-name (parse-xml-node :|parent| (xml-element-child node :|parent|) robot))
         (child-name (parse-xml-node :|child| (xml-element-child node :|child|) robot))
         (joint
          (make-instance 'joint
                         :name (s-xml:xml-element-attribute node :|name|)
                         :type (intern (string-upcase
                                        (s-xml:xml-element-attribute node :|type|))
                                       (find-package :keyword))
                         :parent (gethash parent-name (links robot))
                         :child (gethash child-name (links robot)))))
    (with-slots (axis origin limits parent child) joint
      (when axis-node
        (setf axis (parse-xml-node :|axis| axis-node robot)))
      (when origin-node
        (setf origin (parse-xml-node :|origin| origin-node robot)))
      (when limits-node
        (setf limits (parse-xml-node :|limit| limits-node robot)))
      (push joint (slot-value parent 'to-joints))
      (if (from-joint child)
          (error 'urdf-malformed
                 :format-control "Link `~a' seems to have two parents"
                 :format-arguments (list (name child)))
          (setf (slot-value child 'from-joint) joint)))
    joint))

(defmethod parse-xml-node ((name (eql :|parent|)) node &optional robot)
  (declare (ignore robot))
  (s-xml:xml-element-attribute node :|link|))

(defmethod parse-xml-node ((name (eql :|child|)) node &optional robot)
  (declare (ignore robot))
  (s-xml:xml-element-attribute node :|link|))

(defmethod parse-xml-node ((name (eql :|origin|)) node &optional robot)
  (declare (ignore robot))
  (flet ((euler->quaternion (ax ay az)
           (cl-transforms:euler->quaternion :ax ax :ay ay :az az)))
    (cl-transforms:make-transform
     (apply #'cl-transforms:make-3d-vector
            (if (s-xml:xml-element-attribute node :|xyz|)
                (read-triple (s-xml:xml-element-attribute node :|xyz|))
                '(0 0 0)))
     (if (s-xml:xml-element-attribute node :|rpy|)
         (apply #'euler->quaternion (read-triple (s-xml:xml-element-attribute node :|rpy|)))
         (cl-transforms:make-quaternion 0 0 0 1)))))

(defmethod parse-xml-node ((name (eql :|mass|)) node &optional robot)
  (declare (ignore robot))
  (read-number (s-xml:xml-element-attribute node :|value|)))

(defmethod parse-xml-node ((name (eql :|axis|)) node &optional robot)
  (declare (ignore robot))
  (apply #'cl-transforms:make-3d-vector
         (read-triple (s-xml:xml-element-attribute node :|xyz|))))

(defmethod parse-xml-node ((name (eql :|limit|)) node &optional robot)
  (declare (ignore robot))
  (let ((lower-str (s-xml:xml-element-attribute node :|lower|))
        (upper-str (s-xml:xml-element-attribute node :|upper|))
        (limits (make-instance
                'limits
                :effort (read-number
                         (s-xml:xml-element-attribute node :|effort|))
                :velocity (read-number
                           (s-xml:xml-element-attribute node :|velocity|)))))
    (with-slots (lower upper) limits
      (when lower-str
        (setf lower (read-number lower-str)))
      (when upper-str
        (setf upper (read-number upper-str))))
    limits))

(defgeneric parse-urdf (identifier)
  (:documentation "Parses a URDF file and returns the corresponding
  Lisp representation. `identifier' can either be a pathname pointing
  to a URDF file, a stream or a string that contains the complete XML
  description.")
  (:method ((identifier stream))
    (let ((xml (s-xml:parse-xml identifier :output-type :xml-struct)))
      (parse-xml-node (s-xml:xml-element-name xml) xml)))
  (:method ((identifier string))
    (parse-urdf (make-string-input-stream identifier)))
  (:method ((identifier pathname))
    (with-open-file (stream identifier :direction :input)
      (parse-urdf stream))))
