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


(defun parse-hash-table (hash-table)
  (loop for key being the hash-keys in hash-table using (hash-value value)
        for key-name = (intern (string-upcase key) :keyword)
        for parsed-key-value-list = (parse-json-node key-name value)
        if parsed-key-value-list
          collecting parsed-key-value-list))

(defun parse-nested-node (name node)
  (mapcar (lambda (key-value-pair)
            (destructuring-bind (key value)
                key-value-pair
              (list (intern (concatenate 'string (string-upcase name) "-" (string-upcase key))
                            :keyword) value)))
          (parse-hash-table node)))

(defun parse-number (node)
  node)

(defun parse-ignored-node ()
  nil)

(defun parse-case-sensitive-string (node)
  (intern node :keyword))

(defun parse-string (node)
  (intern (string-upcase node) :keyword))

(defun parse-pose (node)
  (let* ((source (gethash "source" node))
         (pose-stamped (gethash "pose_stamped" node))
         (header (gethash "header" pose-stamped))
         (frame-id (gethash "frame_id" header))
         (stamp-hash-tbl (gethash "stamp" header))
         (stamp-sec (gethash "sec" stamp-hash-tbl))
         (stamp-nsec (gethash "nsec" stamp-hash-tbl))
         (stamp (+ stamp-sec (/ stamp-nsec 1.0d9)))
         (pose (gethash "pose" pose-stamped))
         (orientation (gethash "orientation" pose))
         (rot-x (gethash "x" orientation))
         (rot-y (gethash "y" orientation))
         (rot-z (gethash "z" orientation))
         (rot-w (gethash "w" orientation))
         (position (gethash "position" pose))
         (pos-x (gethash "x" position))
         (pos-y (gethash "y" position))
         (pos-z (gethash "z" position))
         (object-pose (cl-transforms-stamped:make-pose-stamped
                       frame-id
                       stamp
                       (cl-transforms:make-3d-vector pos-x pos-y pos-z)
                       (cl-transforms:make-quaternion rot-x rot-y rot-z rot-w))))
    (list (parse-string source) object-pose)))

(defun parse-poses (node)
  (mapcar #'parse-pose node))

;; (defun parse-transform (node)
;;   (return-from parse-transform (list node))
;;   (let* ((frame-id (gethash "frame_id" node))
;;          (child-frame-id (gethash "child_frame_id" node))
;;          (pos-x (gethash "pos_x" node))
;;          (pos-y (gethash "pos_y" node))
;;          (pos-z (gethash "pos_z" node))
;;          (rot-x (gethash "rot_x" node))
;;          (rot-y (gethash "rot_y" node))
;;          (rot-z (gethash "rot_z" node))
;;          (rot-w (gethash "rot_w" node))
;;          (stamp (gethash "stamp" node))
;;          (object-transform
;;            (cl-transforms:make-transform
;;             (cl-transforms:make-3d-vector pos-x pos-y pos-z)
;;             (cl-transforms:make-quaternion rot-x rot-y rot-z rot-w))))
;;     ;; string knowrob namespace from frame IDs
;;     (let ((child-frame-position-of-# (position #\# child-frame-id :from-end t)))
;;       (when child-frame-position-of-#
;;         (setf child-frame-id (subseq child-frame-id (1+ child-frame-position-of-#)))))
;;     ;; make sure transform is defined in robot-base-frame
;;     (let ((object-transform-stamped
;;             (if (equalp frame-id cram-tf:*robot-base-frame*)
;;                 (cl-transforms-stamped:transform->transform-stamped
;;                  frame-id
;;                  child-frame-id
;;                  stamp
;;                  object-transform)
;;                 (let ((transform-to-base-frame (cl-transforms-stamped:lookup-transform
;;                                                 cram-tf:*transformer*
;;                                                 cram-tf:*robot-base-frame*
;;                                                 frame-id
;;                                                 :timeout cram-tf:*tf-default-timeout*)))
;;                   (cl-transforms-stamped:transform->transform-stamped
;;                    cram-tf:*robot-base-frame*
;;                    child-frame-id
;;                    (cl-transforms-stamped:stamp transform-to-base-frame)
;;                    (cl-transforms:transform* transform-to-base-frame object-transform))))))
;;       object-transform-stamped)))

(defun parse-color (node)
  (let* ((colors-sorted
           (sort (parse-hash-table node)
                 #'> :key #'second))
         (num-colors
           (length colors-sorted))
         (top-3-colors
           (subseq colors-sorted 0 (min num-colors 3))))
    (mapcar #'first top-3-colors)))

(defun parse-class (node)
  (let* ((parsed-nested-key-values (parse-hash-table node))
         (class-name (second (assoc :class-name parsed-nested-key-values)))
         (knowrob-string (string-trim "'" class-name))
         (position-of-# (position #\# knowrob-string :from-end t))
         (trimmed-class-name (if position-of-#
                                 (subseq knowrob-string (1+ position-of-#))
                                 knowrob-string))
         (resulting-class (roslisp-utilities:lispify-ros-name
                           trimmed-class-name
                           :keyword)))
    resulting-class))


(defmethod parse-json-node (name (node hash-table))
  (let* ((parsed-node (parse-hash-table node)))
    (if name
        (list name parsed-node)
        parsed-node)))

(defmethod parse-json-node (name (node list))
  (let ((parsed-node (mapcar #'parse-hash-table node)))
    (if name
        (list name parsed-node)
        parsed-node)))

(defmethod parse-json-node ((name (eql :timestamp)) node)
  (parse-ignored-node))
;; (defmethod parse-json-node ((name (eql :uid)) node)
;;   (list :name (roslisp-utilities:lispify-ros-name
;;                (let ((knowrob-string (string-trim "'" node)))
;;                  (let* ((position-of-# (position #\# knowrob-string :from-end t)))
;;                    (if position-of-#
;;                        (subseq knowrob-string (1+ position-of-#))
;;                        knowrob-string)))
;;                :keyword)))
(defmethod parse-json-node ((name (eql :uid)) node)
  ;; (list name (parse-case-sensitive-string node))
  (parse-ignored-node))
(defmethod parse-json-node ((name (eql :id)) node)
  (list :name (parse-case-sensitive-string
               (let ((first-char (aref node 0)))
                (if (and (>= (char-code first-char) 48)
                         (<= (char-code first-char) 57))
                    (concatenate 'string "OBJ" node)
                    node)))))

(defmethod parse-json-node ((name (eql :type)) node)
  (list name (parse-string node)))
(defmethod parse-json-node ((name (eql :shape)) node)
  (list name (mapcar #'parse-string node)))
(defmethod parse-json-node ((name (eql :size)) node)
  (list name (parse-string node)))
(defmethod parse-json-node ((name (eql :source)) node)
  (list name (parse-string node)))

(defmethod parse-json-node ((name (eql :segment)) node)
  (parse-nested-node name node))
(defmethod parse-json-node ((name (eql :boundingbox)) node)
  (parse-nested-node :bb node))
;; (defmethod parse-json-node ((name (eql :pose)) node)
;;   (list name (parse-pose node)))
(defmethod parse-json-node ((name (eql :poses)) node)
  (list name (parse-poses node)))

;; (defmethod parse-json-node ((name (eql :transform)) node)
;;   (list name (parse-transform node)))

(defmethod parse-json-node ((name (eql :color)) node)
  (list name (parse-color node)))

;; (defmethod parse-json-node ((name (eql :detection)) node)
;;   ;; (sort ll #'> :key (lambda (x) (second (find :confidence x :key #'car))))
;;   ;; (cons name (second (find :type (alexandria:extremum
;;   ;;                                 (second (call-next-method))
;;   ;;                                 #'>
;;   ;;                                 :key (lambda (x)
;;   ;;                                        (second (find :confidence x :key #'car))))
;;   ;;                          :key #'first)))
;;   (find :class (parse-alist node) :key #'car))

(defmethod parse-json-node ((name (eql :class)) node)
  ;; ignore confidence entry for now
  (list :type (parse-class node)))

(defmethod parse-json-node ((name (eql :dimensions-2d)) node)
  (list name (let ((parsed-dimensions (parse-hash-table node)))
               (cl-transforms:make-3d-vector (second (assoc :width parsed-dimensions))
                                             (second (assoc :height parsed-dimensions)) 0))))

(defmethod parse-json-node ((name (eql :dimensions-3d)) node)
  (list name (let ((parsed-dimensions (parse-hash-table node)))
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
  (flatten-one-level (parse-json-node nil (yason:parse json-string :object-as :hash-table))))

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
