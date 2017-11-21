;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :rs)

(defparameter *show-json-warnings* nil)

(define-condition json-key-not-supported (simple-warning) ())

(defgeneric parse-json-node (name node)
  (:method (name node)
    (when *show-json-warnings*
      (warn 'json-key-not-supported
            :format-control "JSON element type `~a' not supported. Ignoring"
            :format-arguments (list name)))
    (list name node)))

(defun parse-alist (alist)
  (remove-if #'null
             (mapcar (lambda (key-value-cons)
                       (let ((key-name (intern (string-upcase (car key-value-cons))
                                               :keyword)))
                         (parse-json-node key-name (cdr key-value-cons))))
                     alist)))

(defun parse-nested-node (name node)
  (mapcar (lambda (key-value-pair)
            (destructuring-bind (key value)
                key-value-pair
             (list (intern (concatenate 'string (string-upcase name) "-" (string-upcase key))
                           :keyword) value)))
          (parse-alist node)))

(defmacro getassoc (key alist)
  `(cdr (assoc ,key ,alist :test #'equal)))

(defmethod parse-json-node (name (node list))
  (let ((parsed-node (parse-alist node)))
    (if name
        (list name parsed-node)
        parsed-node)))

(defmethod parse-json-node ((name (eql :timestamp)) node) nil)
(defmethod parse-json-node ((name (eql :pipelineid)) node) nil)

(defmethod parse-json-node ((name (eql :id)) node)
  (list :name (roslisp-utilities:lispify-ros-name
               (let ((knowrob-string (string-trim "'" node)))
                 (let* ((position-of-# (position #\# knowrob-string :from-end t)))
                   (if position-of-#
                       (subseq knowrob-string (1+ position-of-#))
                       knowrob-string)))
               :keyword)))

(defmethod parse-json-node ((name (eql :type)) node)
  (list name (intern (string-upcase node) :keyword)))
(defmethod parse-json-node ((name (eql :shape)) node)
  (list name (intern (string-upcase node) :keyword)))
(defmethod parse-json-node ((name (eql :size)) node)
  (list name (intern (string-upcase node) :keyword)))
(defmethod parse-json-node ((name (eql :source)) node)
  (list name (intern (string-upcase node) :keyword)))

(defmethod parse-json-node ((name (eql :segment)) node)
  (parse-nested-node name node))
(defmethod parse-json-node ((name (eql :boundingbox)) node)
  (parse-nested-node :bb node))

(defmethod parse-json-node ((name (eql :pose)) node)
  (setf *rs-result-debug* node)
  (if (getassoc "pose" node)
      (list name (parse-alist node))
      (let* ((frame-id (getassoc "frame_id" node))
             (pos-x (getassoc "pos_x" node))
             (pos-y (getassoc "pos_y" node))
             (pos-z (getassoc "pos_z" node))
             (rot-x (getassoc "rot_x" node))
             (rot-y (getassoc "rot_y" node))
             (rot-z (getassoc "rot_z" node))
             (rot-w (getassoc "rot_w" node))
             (stamp (getassoc "stamp" node))
             (object-pose (cl-transforms-stamped:make-pose-stamped
                           frame-id
                           stamp
                           (cl-transforms:make-3d-vector pos-x pos-y pos-z)
                           (cl-transforms:make-quaternion rot-x rot-y rot-z rot-w))))
        (list name object-pose))))

(defmethod parse-json-node ((name (eql :transform)) node)
  (let* ((frame-id (getassoc "frame_id" node))
         (child-frame-id (getassoc "child_frame_id" node))
         (pos-x (getassoc "pos_x" node))
         (pos-y (getassoc "pos_y" node))
         (pos-z (getassoc "pos_z" node))
         (rot-x (getassoc "rot_x" node))
         (rot-y (getassoc "rot_y" node))
         (rot-z (getassoc "rot_z" node))
         (rot-w (getassoc "rot_w" node))
         (stamp (getassoc "stamp" node))
         (object-transform
           (cl-transforms:make-transform
            (cl-transforms:make-3d-vector pos-x pos-y pos-z)
            (cl-transforms:make-quaternion rot-x rot-y rot-z rot-w))))
    ;; string knowrob namespace from frame IDs
    (let ((child-frame-position-of-# (position #\# child-frame-id :from-end t)))
      (when child-frame-position-of-#
        (setf child-frame-id (subseq child-frame-id (1+ child-frame-position-of-#)))))
    ;; make sure transform is defined in robot-base-frame
    (let ((object-transform-stamped
            (if (equalp frame-id cram-tf:*robot-base-frame*)
                (cl-transforms-stamped:transform->transform-stamped
                 frame-id
                 child-frame-id
                 stamp
                 object-transform)
                (let ((transform-to-base-frame (cl-transforms-stamped:lookup-transform
                                                cram-tf:*transformer*
                                                cram-tf:*robot-base-frame*
                                                frame-id
                                                :timeout cram-tf:*tf-default-timeout*)))
                  (cl-transforms-stamped:transform->transform-stamped
                   cram-tf:*robot-base-frame*
                   child-frame-id
                   (cl-transforms-stamped:stamp transform-to-base-frame)
                   (cl-transforms:transform* transform-to-base-frame object-transform))))))
      (list name object-transform-stamped))))

(defmethod parse-json-node ((name (eql :color)) node)
  (cons name (mapcar #'first
                     (subseq (sort (parse-alist node)
                                   #'> :key #'second)
                             0 3))))

(defmethod parse-json-node ((name (eql :detection)) node)
  ;; (sort ll #'> :key (lambda (x) (second (find :confidence x :key #'car))))
  ;; (cons name (second (find :type (alexandria:extremum
  ;;                                 (second (call-next-method))
  ;;                                 #'>
  ;;                                 :key (lambda (x)
  ;;                                        (second (find :confidence x :key #'car))))
  ;;                          :key #'first)))
  (find :class (parse-alist node) :key #'car))

(defmethod parse-json-node ((name (eql :class)) node) ; ignore confidence entry for now
  (list :type (let ((parsed-nested-key-values (parse-alist node)))
                (roslisp-utilities:lispify-ros-name
                 (let ((knowrob-string (string-trim "'" (second (assoc :name parsed-nested-key-values)))))
                   (let* ((position-of-# (position #\# knowrob-string :from-end t)))
                     (if position-of-#
                         (subseq knowrob-string (1+ position-of-#))
                         knowrob-string)))
                 :keyword))))

(defmethod parse-json-node ((name (eql :dimensions-2d)) node)
  (list name (let ((parsed-dimensions (parse-alist node)))
               (cl-transforms:make-3d-vector (second (assoc :width parsed-dimensions))
                                             (second (assoc :height parsed-dimensions)) 0))))

(defmethod parse-json-node ((name (eql :dimensions-3d)) node)
  (list name (let ((parsed-dimensions (parse-alist node)))
               (cl-transforms:make-3d-vector (second (assoc :width parsed-dimensions))
                                             (second (assoc :depth parsed-dimensions))
                                             (second (assoc :height parsed-dimensions))))))


(defun flatten-one-level (tree)
  (let (flattened-tree)
    (mapc (lambda (key-value-pair)
            (let ((key (first key-value-pair))
                  (value (second key-value-pair)))
              (if (listp key)
                  (mapc (lambda (inner-key-value-pair)
                          (push inner-key-value-pair flattened-tree))
                        key-value-pair)
                  (let ((index-of-key-in-flattened
                          (position key flattened-tree :key #'car)))
                    (if index-of-key-in-flattened
                        (nconc (nth index-of-key-in-flattened flattened-tree) (list value)) 
                        (push key-value-pair flattened-tree))))))
          tree)
    flattened-tree))

(defun parse-json-result (json-string)
  (flatten-one-level (parse-json-node nil (yason:parse json-string :object-as :alist))))

 ;; (#<CRAM-DESIGNATORS:OBJECT-DESIGNATOR
 ;;   ((:TYPE :WHEEL)
 ;;    (:CLUSTER-ID "http://knowrob.org/kb/thorin_simulation.owl#Wheel1")
 ;;    (:POSE
 ;;     ((:SOURCE :SIMULATION)
 ;;      (:TRANSFORM
 ;;       #<CL-TRANSFORMS-STAMPED:TRANSFORM-STAMPED 
 ;;   FRAME-ID: "map", CHILD-FRAME-ID: "Wheel1", STAMP: 1499343257775080150
 ;;   #<3D-VECTOR (-0.9950000047683716d0 1.5770000219345093d0 0.8920000195503235d0)>
 ;;   #<QUATERNION (0.0d0 0.0d0 0.0d0 1.0d0)>>)))
 ;;    (:CLASS ((:CONFIDENCE 1.0) (:NAME "Wheel"))))
 ;;   {10085D63C3}>
 ;; #<CRAM-DESIGNATORS:OBJECT-DESIGNATOR
 ;;   ((:TYPE :WHEEL)
 ;;    (:CLUSTER-ID "http://knowrob.org/kb/thorin_simulation.owl#Wheel2")
 ;;    (:POSE
 ;;     ((:SOURCE :SIMULATION)
 ;;      (:TRANSFORM
 ;;       #<CL-TRANSFORMS-STAMPED:TRANSFORM-STAMPED 
 ;;   FRAME-ID: "map", CHILD-FRAME-ID: "Wheel2", STAMP: 1499343257782435062
 ;;   #<3D-VECTOR (-0.9950000047683716d0 1.5269999504089355d0 0.8920000195503235d0)>
 ;;   #<QUATERNION (0.0d0 0.0d0 0.0d0 1.0d0)>>)))
 ;;    (:CLASS ((:CONFIDENCE 1.0) (:NAME "Wheel"))))
 ;;   {10085D6783}>
 ;; #<CRAM-DESIGNATORS:OBJECT-DESIGNATOR
 ;;   ((:TYPE :WHEEL)
 ;;    (:CLUSTER-ID "http://knowrob.org/kb/thorin_simulation.owl#Wheel3")
 ;;    (:POSE
 ;;     ((:SOURCE :SIMULATION)
 ;;      (:TRANSFORM
 ;;       #<CL-TRANSFORMS-STAMPED:TRANSFORM-STAMPED 
 ;;   FRAME-ID: "map", CHILD-FRAME-ID: "Wheel3", STAMP: 1499343257788766092
 ;;   #<3D-VECTOR (-0.9950000047683716d0 1.4769999980926514d0 0.8920000195503235d0)>
 ;;   #<QUATERNION (0.0d0 0.0d0 0.0d0 1.0d0)>>)))
 ;;    (:CLASS ((:CONFIDENCE 1.0) (:NAME "Wheel"))))
 ;;   {10085D6B43}>
 ;; #<CRAM-DESIGNATORS:OBJECT-DESIGNATOR
 ;;   ((:TYPE :WHEEL)
 ;;    (:CLUSTER-ID "http://knowrob.org/kb/thorin_simulation.owl#Wheel4")
 ;;    (:POSE
 ;;     ((:SOURCE :SIMULATION)
 ;;      (:TRANSFORM
 ;;       #<CL-TRANSFORMS-STAMPED:TRANSFORM-STAMPED 
 ;;   FRAME-ID: "map", CHILD-FRAME-ID: "Wheel4", STAMP: 1499343257795483558
 ;;   #<3D-VECTOR (-0.9950000047683716d0 1.4270000457763672d0 0.8920000195503235d0)>
 ;;   #<QUATERNION (0.0d0 0.0d0 0.0d0 1.0d0)>>)))
 ;;    (:CLASS ((:CONFIDENCE 1.0) (:NAME "Wheel"))))
 ;;   {10085D6F03}>)
