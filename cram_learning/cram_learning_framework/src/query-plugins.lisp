;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :learning)

(defmethod man-int:get-object-type-grasps :around (object-type arm)
  (let ((all-possible-grasps-unsorted
          (call-next-method))
        (learned-grasps
          (list
           (cut:var-value
            '?Grasp
            (car (json-prolog:prolog-simple
                  (format nil
                          "object_type_grasps(~a, ~a, ?grasp)."
                          object-type arm)))))))
    (append learned-grasps
            (reduce (lambda (list-to-remove-from learned-grasp)
                      (remove learned-grasp list-to-remove-from))
                    (append (list all-possible-grasps-unsorted) learned-grasps)))))

(defun make-learned-costmap-generator (location-type object-type object-name
                                       object-pose-in-base object-pose-in-map)
  (cut:with-vars-strictly-bound (?matrix ?top-left-corner-x ?top-left-corner-y ?resolution)
      (car (json-prolog:prolog-simple
            (format nil
                    "designator_costmap(~a, ~a, ~a, ~a, ~a, ~
                                        ?matrix, ~
                                        ?top-left-corner-x, ?top-left-corner-y, ?resolution)."
                    location-type object-type object-name object-pose-in-base object-pose-in-map)))
    (costmap:make-matrix-cost-function ?top-left-corner-x ?top-left-corner-y ?resolution ?matrix)))

(defmethod costmap:costmap-generator-name->score ((name (eql 'learned-generator))) 9)

(def-fact-group location-costmap-integration (costmap:desig-costmap)
  (<- (costmap:desig-costmap ?designator ?costmap)
    (rob-int:reachability-designator ?designator)
    (desig:desig-prop ?designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:type ?object-type))
    (spec:property ?current-object-designator (:name ?object-name))
    (lisp-fun man-int:get-object-pose ?current-object-designator ?object-pose-in-base)
    (lisp-fun man-int:get-object-pose-in-map ?current-object-designator ?object-pose-in-map)
    (costmap:costmap ?costmap)
    (costmap:costmap-add-function
     learned-generator
     (make-learned-costmap-generator
      :reaching ?object-type ?object-name ?object-pose-in-base ?object-pose-in-map)
     ?costmap)))


(defmethod generic-perform :around (designator)
  (when (and (typep designator 'desig:action-designator)
             (eql (desig:desig-prop-value designator :type) :fetching))
    (let* ((some-object (desig:desig-prop-value designator :object))
           (object (desig:current-desig some-object))
           (object-type (desig:desig-prop-value object :type))
           (object-name (desig:desig-prop-value object :name))
           (object-pose-in-base (man-int:get-object-pose object))
           (object-pose-in-map (man-int:get-object-pose-in-map object))
           (arm (desig:desig-prop-value designator :arm)))
      (json-prolog:prolog-simple
       (format nil "performing_action(~a, ~a, ~a, ~a, ~a, ~a)."
               :fetching object-type object-name object-pose-in-base object-pose-in-map arm))
      (call-next-method)
      (json-prolog:prolog-simple
       (format nil "finished_action(~a)."
               :fetching)))))





