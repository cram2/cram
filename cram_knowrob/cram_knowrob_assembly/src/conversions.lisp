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

(in-package :kr-assembly)

(defparameter *knowrob-namespaces*
  '((:rdf "http://www.w3.org/1999/02/22-rdf-syntax-ns#")
    (:rdfs "http://www.w3.org/2000/01/rdf-schema#")
    (:owl "http://www.w3.org/2002/07/owl#")
    (:xsd "http://www.w3.org/2001/XMLSchema#")
    (:knowrob "http://knowrob.org/kb/knowrob.owl#")
    (:log "http://knowrob.org/kb/unreal_log.owl#")
    (:u-map "http://knowrob.org/kb/u_map.owl#")
    (:swrl "http://www.w3.org/2003/11/swrl#")
    (:computable "http://knowrob.org/kb/computable.owl#")
    (:srdl2-action "http://knowrob.org/kb/srdl2-action.owl#")
    (:knowrob_assembly "http://knowrob.org/kb/knowrob_assembly.owl#")
    (:knowrob_beliefstate "http://www.knowrob.org/kb/knowrob_beliefstate#")
    (:knowrob_paramserver "http://knowrob.org/kb/knowrob_paramserver.owl#")
    (:thorin_parts "http://knowrob.org/kb/thorin_parts.owl#")
    (:thorin_simulation "http://knowrob.org/kb/thorin_simulation.owl#")))

(defparameter *default-namespace* :knowrob)
(defparameter *individuals-namespace* :thorin_simulation)
(defparameter *classes-namespace* :thorin_parts)

(defparameter *cram-knowrob-names*
  '(;; agents
    ;; (:red-wasp "SherpaWaspRed" :knowrob :log)
    ;; agent parts
    (:camaro-body "CamaroBody")
    ;; sensors
    ;; (:engine "Engine" :knowrob :log)
    ;; objects
    ;; (:victim "SherpaVictim" :knowrob :log)
    ;; actions
    ;; (:go "Movement-TranslationEvent" :knowrob :log)
    ;; entities
    ;; (:camera-image "CameraImage" :knowrob :log)
    ;; (:location "Location" :knowrob :log)
    ;; (:pose "Pose" :knowrob :u-map)
    )
  "An association list of CRAM name to KnowRob name mappings")

(defparameter *unique-id-length* 8
  "How many characters to append to create unique IDs for OWL individuals")

(defgeneric cram->knowrob (object &key &allow-other-keys)
  (:documentation "Convert CRAM `object' into it's KnowRob representation (mostly a string)")
  (:method ((object null) &key &allow-other-keys)
    (roslisp:ros-info (kr-assembly conversions) "Got a NIL object to convert. Ignoring.")))

(defun append-namespace (namespace string)
  (concatenate 'string (cadr (assoc namespace *knowrob-namespaces*)) string))

(defun generate-unique-id (string)
  (flet ((random-string (&optional (length *unique-id-length*))
           (let ((chars "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"))
             (coerce (loop repeat length collect (aref chars (random (length chars))))
                     'string))))
    (concatenate 'string string "_" (random-string))))

(defun generate-timepoint (&optional time)
  (declare (type (or null double-float) time))
  (append-namespace *default-namespace*
                    (format nil "timepoint_~,3f" (or time (roslisp:ros-time)))))

(defmethod cram->knowrob ((cram-name string) &key namespace-id)
  (if namespace-id
      (if (position #\# cram-name :from-end t)
          ;; if it already has a namespace, don't mess with it
          cram-name
          (append-namespace namespace-id cram-name))
      cram-name))

(defmethod cram->knowrob ((cram-name symbol) &key namespace-id)
  (declare (type (or null keyword) namespace-id))
  (let ((kr-name (or (second (assoc cram-name *cram-knowrob-names*))
                     (roslisp-utilities:rosify-lisp-name cram-name))))
    (if namespace-id
        (append-namespace namespace-id kr-name)
        kr-name)))

(defmethod cram->knowrob ((transform-st cl-transforms-stamped:transform-stamped) &key)
  (let* ((parent-frame (cl-transforms-stamped:frame-id transform-st))
         (child-frame (cl-transforms-stamped:child-frame-id transform-st))
         (xyz (cl-transforms:translation transform-st))
         (qqqw (cl-transforms:orientation transform-st))
         (x (cl-transforms:x xyz))
         (y (cl-transforms:y xyz))
         (z (cl-transforms:z xyz))
         (q1 (cl-transforms:x qqqw))
         (q2 (cl-transforms:y qqqw))
         (q3 (cl-transforms:z qqqw))
         (w (cl-transforms:w qqqw)))
    `(,parent-frame ,child-frame (,x ,y ,z) (,q1 ,q2 ,q3 ,w))))

;; (defmethod cram->knowrob ((designator location-designator) &key name-with-id)
;;   (let ((pose-name (loggable-pose-name)))
;;     (call-logging-action
;;      (make-logging-goal
;;       name-with-id
;;       (loggable-location-type)
;;       (vector (loggable-property-with-resource :|pose| pose-name))))
;;     (log-owl (desig:desig-prop-value designator :pose)
;;              :name-with-id pose-name)))

;; (defmethod log-owl-action ((type (eql :go)) designator &key start-time agent)
;;   (let ((goal-location-name (loggable-location-name)))
;;     (call-logging-action
;;      (loggable-action
;;       type start-time NIL T agent
;;       (loggable-property-with-resource :|goalLocation| goal-location-name)))
;;     (log-owl (or (car (remove :go (desig:desig-prop-values designator :to)))
;;                  (desig:desig-prop-value designator :destination))
;;              :name-with-id goal-location-name)))

;; (defmethod log-owl-action ((type (eql :going)) designator &key start-time agent)
;;   (log-owl-action :go designator :start-time start-time :agent agent))


(defgeneric knowrob->cram (object-type object &key &allow-other-keys))

(defmethod knowrob->cram ((object-type (eql :string)) knowrob-symbol &key (strip-namespace t))
  (let ((knowrob-string (string-trim "'" (symbol-name knowrob-symbol))))
    (if strip-namespace
      (let* ((position-of-# (position #\# knowrob-string :from-end t)))
        (if position-of-#
            (subseq knowrob-string (1+ position-of-#))
            knowrob-string))
      knowrob-string)))

(defmethod knowrob->cram ((object-type (eql :symbol)) knowrob-symbol &key (strip-namespace t)
                                                                       (package *package*))
  (roslisp-utilities:lispify-ros-name
   (knowrob->cram :string knowrob-symbol :strip-namespace strip-namespace)
   package))

(defmethod knowrob->cram ((object-type (eql :transform)) transform-list &key)
  "`transform-list' looks like this:
 [string reference_frame, string target_frame, [float x, y, z], [float x, y, z, w]]"
  (cl-transforms-stamped:make-transform-stamped
     (knowrob->cram :string (first transform-list))
     (knowrob->cram :string (second transform-list))
     0.0
     (apply #'cl-transforms:make-3d-vector (third transform-list))
     (apply #'cl-transforms:make-quaternion (fourth transform-list))))

(defmethod knowrob->cram ((object-type (eql :grasp-spec)) grasp-spec-list &key)
  "`grasp-spec-list' looks like this:
 [string Gripper, string Object, string Robot,
  [string reference_frame, string target_frame, [float x, y, z], [float x, y, z, w]]]"
  (list
   (knowrob->cram :symbol (first grasp-spec-list))
   (knowrob->cram :symbol (second grasp-spec-list))
   (knowrob->cram :symbol (third grasp-spec-list))
   (knowrob->cram :transform (fourth grasp-spec-list))))
