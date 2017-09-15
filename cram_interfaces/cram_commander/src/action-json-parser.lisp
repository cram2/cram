;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :commander)

(defparameter *show-json-warnings* t)

(define-condition json-key-not-supported (simple-warning) ())

(define-condition json-parser-failed (error)
  ((description :initarg :description :reader error-description))
  (:report (lambda (condition stream)
             (format stream (error-description condition)))))

(defgeneric parse-json-node (name node)
  (:documentation "Parses a key-value pair, where `name' is the key
and `node' - a list of values. The result is a list, e.g. (parsed-named parsed-value1).")
  (:method (name node)
    "Default implementation throws a warning when a node is not supported
and returns a list with (key value)"
    (when *show-json-warnings*
      (warn 'json-key-not-supported
            :format-control "JSON element type `~a' not supported. Ignoring"
            :format-arguments (list name)))
    (cons name node)))


(defun parse-property-list (the-list)
  "A general function that for each key-value list element converts the key into a keyword
and calls PARSE-JSON-NODE on the resulting key-value pairs.
This is where the result of YASON:PARSE lands."
  (remove-if #'null
             (mapcar (lambda (elem)
                       (if (listp (cdr elem))
                        (let ((key-name (intern (string-upcase (car elem))
                                                :keyword)))
                          (parse-json-node key-name (cdr elem)))))
                     the-list)))

(defun parse-designator-description (the-list)
  "A general function to parse things like (a location ((prop1 val1)))"
  (destructuring-bind (article designator-type designator-properties)
      the-list
    (declare (ignore article))
    (desig:make-designator
     (intern (string-upcase designator-type) :keyword)
     (parse-property-list designator-properties))))

(defun parse-action-json (json-string)
  (handler-case
      (parse-designator-description (yason:parse json-string :object-as :alist))
    (error (error-object)
      (error 'json-parser-failed
             :description (format nil
                                  "Could not parse json string ~a:~%~a"
                                  json-string
                                  error-object)))))


;;; Individual implementations of parsing different key-value pairs,
;;; where value is always a list of values.


(defun pose-from-list (the-list)
  (destructuring-bind ((x y z) (q1 q2 q3 w))
      the-list
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

;;; ACTION PROPERTIES

(defmethod parse-json-node ((name (eql :type)) node)
  (list name (intern (string-upcase (car node)) :keyword)))
(defmethod parse-json-node ((name (eql :device)) node)
  (list name (intern (string-upcase (car node)) :keyword)))
(defmethod parse-json-node ((name (eql :state)) node)
  (list name (intern (string-upcase (car node)) :keyword)))
(defmethod parse-json-node ((name (eql :object)) node)
  (list name (intern (string-upcase (car node)) :keyword)))

(defmethod parse-json-node ((name (eql :area)) node)
  (cons name node))
(defmethod parse-json-node ((name (eql :altitude)) node)
  (cons name node))
(defmethod parse-json-node ((name (eql :value)) node)
  (cons name node))
(defmethod parse-json-node ((name (eql :agent)) node)
  (cons name node))

(defmethod parse-json-node ((name (eql :goal)) node)
  (list name (parse-designator-description (car node))))

(defmethod parse-json-node ((name (eql :to)) node)
  (if (listp (car node))
      (if (= 3 (length (car node)))
          (list name (parse-designator-description (car node)))
          (if (= 2 (length (car node)))
              (list name (pose-from-list (car node)))
              (list name (car node))))
      (list name (intern (string-upcase (car node)) :keyword))))

(defmethod parse-json-node ((name (eql :destination)) node)
  (if (listp (car node))
      (if (= 3 (length (car node)))
          (list name (parse-designator-description (car node)))
          (if (= 2 (length (car node)))
              (list name (pose-from-list (car node)))
              (list name (car node))))
      (list name (intern (string-upcase (car node)) :keyword))))

(defmethod parse-json-node ((name (eql :at)) node)
  (if (listp (car node))
      (if (= 3 (length (car node)))
          (list name (parse-designator-description (car node)))
          (if (= 2 (length (car node)))
              (list name (pose-from-list (car node)))
              (list name (car node))))
      (list name (intern (string-upcase (car node)) :keyword))))

;;; LOCATION PROPERTIES

(defmethod parse-json-node ((name (eql :of)) node)
  (cons name node))
(defmethod parse-json-node ((name (eql :ontop)) node)
  (cons name node))
(defmethod parse-json-node ((name (eql :viewpoint)) node)
  (cons name node))
(defmethod parse-json-node ((name (eql :next-to)) node)
  (cons name node))
(defmethod parse-json-node ((name (eql :left)) node)
  (cons name node))
(defmethod parse-json-node ((name (eql :right)) node)
  (cons name node))
(defmethod parse-json-node ((name (eql :in-front-of)) node)
  (cons name node))
(defmethod parse-json-node ((name (eql :behind)) node)
  (cons name node))

(defmethod parse-json-node ((name (eql :pose)) node)
  (if (listp (car node))
      (if (= 2 (length (car node)))
          (list name (pose-from-list (car node)))
          (list name (car node)))
      (list name (intern (string-upcase (car node)) :keyword))))


;; (defmacro getassoc (key alist)
;;   `(cdr (assoc ,key ,alist :test #'equal)))

;; (defmethod parse-json-node ((name (eql :pose)) node)
;;   (if (getassoc "POSE" node)
;;       (cons name (parse-alist node))
;;       (let* ((frame-id (getassoc "frame_id" node))
;;          (pos-x (getassoc "pos_x" node))
;;          (pos-y (getassoc "pos_y" node))
;;          (pos-z (getassoc "pos_z" node))
;;          (rot-x (getassoc "rot_x" node))
;;          (rot-y (getassoc "rot_y" node))
;;          (rot-z (getassoc "rot_z" node))
;;          (rot-w (getassoc "rot_w" node))
;;          (stamp (getassoc "stamp" node))
;;          (object-pose (cl-transforms-stamped:make-pose-stamped
;;                        frame-id
;;                        stamp
;;                        (cl-transforms:make-3d-vector pos-x pos-y pos-z)
;;                        (cl-transforms:make-quaternion rot-x rot-y rot-z rot-w)))
;;          (object-pose-in-map (cl-transforms-stamped:transform-pose-stamped
;;                                cram-tf:*transformer*
;;                                :use-current-ros-time t
;;                                :timeout cram-tf:*tf-default-timeout*
;;                                :pose object-pose
;;                                :target-frame cram-tf:*fixed-frame*)))
;;     (cons name object-pose-in-map))))

;; (defmethod parse-json-node ((name (eql :detection)) node)
;;   ;; (sort ll #'> :key (lambda (x) (second (find :confidence x :key #'car))))
;;   ;; (cons name (second (find :type (alexandria:extremum
;;   ;;                                 (second (call-next-method))
;;   ;;                                 #'>
;;   ;;                                 :key (lambda (x)
;;   ;;                                        (second (find :confidence x :key #'car))))
;;   ;;                          :key #'first)))
;;   (find :class (parse-alist node) :key #'car))

;; (defmethod parse-json-node ((name (eql :dimensions-2d)) node)
;;   (cons name (let ((parsed-dimensions (parse-alist node)))
;;                (cl-transforms:make-3d-vector (second (assoc :width parsed-dimensions))
;;                                              (second (assoc :height parsed-dimensions)) 0))))

;; (defmethod parse-json-node ((name (eql :dimensions-3d)) node)
;;   (cons name (let ((parsed-dimensions (parse-alist node)))
;;                (cl-transforms:make-3d-vector (second (assoc :width parsed-dimensions))
;;                                              (second (assoc :depth parsed-dimensions))
;;                                              (second (assoc :height parsed-dimensions))))))


;; (defun parse-nested-node (name node)
;;   (mapcar (lambda (key-value-pair)
;;             (destructuring-bind (key value)
;;                 key-value-pair
;;              (cons (intern (concatenate 'string (string-upcase name) "-" (string-upcase key))
;;                            :keyword) value)))
;;           (parse-alist node)))

;; (defmethod parse-json-node ((name (eql :segment)) node)
;;   (parse-nested-node name node))
;; (defmethod parse-json-node ((name (eql :boundingbox)) node)
;;   (parse-nested-node :bb node))


;; (defun parse-list (the-list)
;;   "Parses a list: it can be a designator property list or a list of property values"
;;   (remove-if #'null
;;              (mapcar (lambda (elem)
;;                        (if (listp elem)
;;                            (if (listp (cdr elem)) ; it's an a list or a proper list
;;                                (parse-list elem)  ; it's a proper list
;;                                (let ((key-name (intern (string-upcase (car elem))
;;                                                        :keyword)))
;;                                  (parse-json-node key-name (cdr elem)))) ; it's an alist
;;                             ; it's not a list
;;                            ))
;;                      the-list)))


;; (defmethod parse-json-node (name (node list))
;;   "For when value is a list, specifically, only alists are supported.
;; This is where the result of YASON:PARSE lands."
;;   (let ((parsed-node (parse-list node)))
;;     (if name
;;         (cons name parsed-node)
;;         parsed-node)))


;; (defun flatten-one-level (tree)
;;   (let (flattened-tree)
;;     (mapc (lambda (key-value-pair)
;;             (let ((key (first key-value-pair))
;;                   (value (second key-value-pair)))
;;               (if (listp key)
;;                   (mapc (lambda (inner-key-value-pair)
;;                           (push inner-key-value-pair flattened-tree))
;;                         key-value-pair)
;;                   (let ((index-of-key-in-flattened
;;                           (position key flattened-tree :key #'car)))
;;                     (if index-of-key-in-flattened
;;                         (nconc (nth index-of-key-in-flattened flattened-tree) (list value)) 
;;                         (push key-value-pair flattened-tree))))))
;;           tree)
;;     flattened-tree))




;; "{\"TYPE\":[\"SCANNING\"],\"AREA\":[\"BRIDGE-1\"]}"
