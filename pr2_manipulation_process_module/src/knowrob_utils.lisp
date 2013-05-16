;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :pr2-manipulation-process-module)

(defun knowrob-symbol->string (knowrob-symbol &optional (remove-quotes t))
  "Takes a 'knowrob-symbol' as typically returned when asking knowrob through json-prolog-client and returns the equivalent string. If remove-quotes is not NIL, the first and last character of the name of the symbol will be removed."
  (declare (type symbol knowrob-symbol))
  (let ((long-symbol-name (symbol-name knowrob-symbol)))
    (unless (> (length long-symbol-name) 1)
      (error
       'simple-error
       :format-control "Asked to remove quote symbols from a string with less than 2 symbols. String: ~a"
       :format-arguments '(long-symbol-name)))
    (if remove-quotes
        (subseq long-symbol-name 1 (- (length long-symbol-name) 1))
        long-symbol-name)))

(defun knowrob-symbol->number (knowrob-symbol)
  (declare (type symbol knowrob-symbol))
  (read-from-string (knowrob-symbol->string knowrob-symbol)))

(defun knowrob-symbol-list->vector-3d (knowrob-symbol-list)
  (declare (type list knowrob-symbol-list))
  (unless (= (length knowrob-symbol-list) 3)
    (error
       'simple-error
       :format-control "Given list of knowrob symbol did not have length 3: ~a~%"
       :format-arguments (list knowrob-symbol-list)))
  (let ((numbers (mapcar #'knowrob-symbol->number knowrob-symbol-list)))
    (cl-transforms:make-3d-vector (first numbers) (second numbers) (third numbers))))

(defun query-knowrob-phase-constraints (phase tool)
  (let ((phase-name (etypecase phase
                      (string phase)
                      (symbol (knowrob-symbol->string phase))))
        (tool-name (etypecase tool
                      (string tool)
                      (symbol (knowrob-symbol->string tool)))))
    (var-value '?cs (lazy-car 
                     (json-prolog:prolog 
                      `("findall" ?c 
                                  ("motion_constraint" ,phase-name ,tool-name ?c)
                                  ?cs)
                               :package 'pr2-manip-pm)))))

(defun query-knowrob-motion-constraint (constraint tool-name object-name)
  (let* ((constraint-name (etypecase constraint
                            (string constraint)
                            (symbol (knowrob-symbol->string constraint))))
         (bindings (json-prolog:prolog
                    `("constraint_properties" ,constraint-name ?type ?tool_feature
                                              ?world_feature ?weight ?lower ?upper
                                              ?min_vel ?max_vel)
                    :package 'pr2-manip-pm)))
    (let ((type-symbol (var-value '?type (lazy-car bindings)))
          (tool-feature-symbol (var-value '?tool_feature (lazy-car bindings)))
          (world-feature-symbol (var-value '?world_feature (lazy-car bindings)))
          (weight-symbol (var-value '?weight (lazy-car bindings)))
          (lower-symbol (var-value '?lower (lazy-car bindings)))
          (upper-symbol (var-value '?upper (lazy-car bindings)))
          (min-vel-symbol (var-value '?min_vel (lazy-car bindings)))
          (max-vel-symbol (var-value '?max_vel (lazy-car bindings))))
      (ecase (knowrob-constraint-type->cram-constraint-type type-symbol)
        (:distance-constraint
         (cram-feature-constraints:make-distance-constraint 
          constraint-name
          (query-knowrob-constraint-feature tool-feature-symbol tool-name)
          (query-knowrob-constraint-feature world-feature-symbol object-name)
          (knowrob-symbol->number lower-symbol)
          (knowrob-symbol->number upper-symbol)
          :weight (knowrob-symbol->number weight-symbol)
          :max-vel (knowrob-symbol->number max-vel-symbol)
          :min-vel (knowrob-symbol->number min-vel-symbol)))
        (:height-constraint
         (cram-feature-constraints:make-height-constraint
          constraint-name
          (query-knowrob-constraint-feature tool-feature-symbol tool-name)
          (query-knowrob-constraint-feature world-feature-symbol object-name)
          (knowrob-symbol->number lower-symbol)
          (knowrob-symbol->number upper-symbol)
          :weight (knowrob-symbol->number weight-symbol)
          :max-vel (knowrob-symbol->number max-vel-symbol)
          :min-vel (knowrob-symbol->number min-vel-symbol)))
        (:perpendicular-constraint
         (cram-feature-constraints:make-perpendicular-constraint
          constraint-name
          (query-knowrob-constraint-feature tool-feature-symbol tool-name)
          (query-knowrob-constraint-feature world-feature-symbol object-name)
          (knowrob-symbol->number lower-symbol)
          (knowrob-symbol->number upper-symbol)
          :weight (knowrob-symbol->number weight-symbol)
          :max-vel (knowrob-symbol->number max-vel-symbol)
          :min-vel (knowrob-symbol->number min-vel-symbol)))
        (:pointing-at-constraint
         (cram-feature-constraints:make-pointing-at-constraint
          constraint-name
          (query-knowrob-constraint-feature tool-feature-symbol tool-name)
          (query-knowrob-constraint-feature world-feature-symbol object-name)
          (knowrob-symbol->number lower-symbol)
          (knowrob-symbol->number upper-symbol)
          :weight (knowrob-symbol->number weight-symbol)
          :max-vel (knowrob-symbol->number max-vel-symbol)
          :min-vel (knowrob-symbol->number min-vel-symbol)))))))
                                              
(defun query-knowrob-constraint-feature (feature frame-id)
  ;; query knowrob for information about the feature
  (let* ((feature-name (etypecase feature
                         (string feature)
                         (symbol (knowrob-symbol->string feature))))
         (bindings (json-prolog:prolog 
                    `("feature_properties" ,feature-name ?type ?label ?_
                                           ?position ?direction ?_)
                    :package 'pr2-manip-pm)))
    ;; extract information from the bindings
    (let ((type-symbol (var-value '?type (lazy-car bindings)))
          (name-symbol (var-value '?label (lazy-car bindings)))
          (position-symbol (var-value '?position (lazy-car bindings)))
          (direction-symbol (var-value '?direction (lazy-car bindings))))
      ;; make cram-internal represenation of feature and return it
      (ecase (knowrob-feature-type->cram-feature-type type-symbol)
        (:point-feature (cram-feature-constraints:make-point-feature
                         (knowrob-symbol->string name-symbol)
                         frame-id
                         :position (knowrob-symbol-list->vector-3d position-symbol)))
        (:line-feature (cram-feature-constraints:make-line-feature
                        (knowrob-symbol->string name-symbol)
                        frame-id
                        :position (knowrob-symbol-list->vector-3d position-symbol)
                        :direction (knowrob-symbol-list->vector-3d direction-symbol)))
        (:plane-feature (cram-feature-constraints:make-plane-feature
                         (knowrob-symbol->string name-symbol)
                         frame-id
                         :position (knowrob-symbol-list->vector-3d position-symbol)
                         :normal (knowrob-symbol-list->vector-3d direction-symbol)))))))

(defun knowrob-feature-type->cram-feature-type (type)
  (let ((type-name (etypecase type
                     (string type)
                     (symbol (knowrob-symbol->string type)))))
    (cond ((string= type-name "http://ias.cs.tum.edu/kb/knowrob.owl#PlaneFeature")
           :plane-feature)
          ((string= type-name "http://ias.cs.tum.edu/kb/knowrob.owl#LineFeature")
           :line-feature)
          ((string= type-name "http://ias.cs.tum.edu/kb/knowrob.owl#PointFeature")
           :point-feature)
          (t (error
              'simple-error
              :format-control "Provided knowrob feature does not match known features: ~a~%"
              :format-arguments (list type))))))

(defun knowrob-constraint-type->cram-constraint-type (type)
  (let ((type-name (etypecase type
                           (string type)
                           (symbol (knowrob-symbol->string type)))))
    (cond ((string= type-name "http://ias.cs.tum.edu/kb/motion-constraints.owl#DistanceConstraint") :distance-constraint)
          ((string= type-name "http://ias.cs.tum.edu/kb/motion-constraints.owl#HeightConstraint") :height-constraint)
          ((string= type-name "http://ias.cs.tum.edu/kb/motion-constraints.owl#PerpendicularityConstraint") :perpendicular-constraint)
          ((string= type-name "http://ias.cs.tum.edu/kb/motion-constraints.owl#PointingAtConstraint") :pointing-at-constraint)
           (t (error
               'simple-error
               :format-control "Provided knowrob constraint type does not match known constraint types: ~a~%"
               :format-arguments (list type))))))