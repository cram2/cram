;;; Copyright (c) 2012, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :spatial-relations-demo)

;; TODO: comments
;; TODO: declares everywhere!

(defparameter *num-of-sets-on-table* 2) ;; max is 4

(defmethod parameterize-demo ((demo-name (eql :table-setting)))
  (setf *demo-object-types*
        '((:main . (:plate :fork :knife :mug))
          (:clutter . (:pot :bowl :mondamin))))
  (setf *demo-objects-initial-poses*
        `((:plate ,@(mapcar (lambda (x) `((1.45 0.8 ,x) (0 0 0 1)))
                           (alexandria:iota *num-of-sets-on-table*
                                            :start 0.86 :step 0.0244)))
          (:fork ,@(mapcar (lambda (x) `((,x 0.5 0.865) (0 0 1 1)))
                          (alexandria:iota *num-of-sets-on-table*
                                           :start 1.35 :step 0.05)))
          (:knife ,@(mapcar (lambda (x) `((,x 0.5 0.857) (0 0 1 1)))
                           (alexandria:iota
                            *num-of-sets-on-table*
                            :start (+ 1.35 (* (+ *num-of-sets-on-table* 1) 0.04))
                            :step 0.05)))
          (:mug ,@(subseq '(((1.5 1.17 0.9119799601336841d0) (0 0 0 1))
                            ((1.46 1.04 0.9119799601336841d0) (0 0 0 1))
                            ((1.35 1.11 0.9119799601336841d0) (0 0 0 1))
                            ((1.65 1.02 0.9119799601336841d0) (0 0 0 1))
                            ((1.35 1.23 0.9119799601336841d0) (0 0 0 1)))
                          0 *num-of-sets-on-table*))
          (:pot ((-1.0 1.65 0.9413339835685429d0) (0 0 0 1)))
          (:bowl ((-0.9 1.95 0.8911207699875103d0) (0 0 0 1))
                ((-0.95 1.16 0.8911207699875103d0) (0 0 0 1))
                ((-0.9 1.3 0.8911207699875103d0) (0 0 0 1))
                ((-1.0 2.36 0.8911207699875103d0) (0 0 0 1)))
          (:mondamin ((-1.0 1.06 0.9573383588498887d0) (0 0 0 1))
                    ((-1.0 2.1 0.9573383588498887d0) (0 0 0 1))))))


;;;;;;;;;;;;;;;;;;;;;;;;; ONLY DESIGS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun put-plates-on-table-with-far (&optional (number-of-plates *num-of-sets-on-table*))
  (spawn-demo :table-setting :set :main)
  (cpl-impl:top-level
    (cram-language-designator-support:with-designators
        ((des-for-plate-2 :location
                          `((:right-of plate-1) (:far-from plate-1) (:for plate-2)))
         (des-for-plate-4 :location
                          `((:left-of plate-3) (:far-from plate-3) (:for plate-4))))
      (when (> number-of-plates 0)
        (prolog `(assert (object-pose ?_ plate-1 ((-1.5 1.84 0.85747d0) (0 0 0 1)))))
        (when (> number-of-plates 1)
          (prolog `(assert (object-pose ?_ plate-3 ((-1.0 1.84 0.85747d0) (0 0 0 1)))))
          (when (> number-of-plates 2)
            (prolog `(assert-object-pose-on plate-2 ,des-for-plate-2))
            (when (> number-of-plates 3)
              (prolog `(assert-object-pose-on plate-4 ,des-for-plate-4)))))))))

(defun make-plate-desig (plate-id &optional (counter-name "kitchen_island_counter_top")
                                    (plate-num *num-of-sets-on-table*))
  (let ((plate-name (new-symbol-with-id "PLATE" plate-id)))
    (make-designator :location `((:on "CounterTop") (:name ,counter-name)
                                 (:for ,plate-name) (:context :table-setting)
                                 (:object-count ,plate-num)))))

(defun make-object-near-plate-desig (object-type object-id &optional (plate-id object-id))
  (let ((plate-name (new-symbol-with-id "PLATE" plate-id))
        (object-name (new-symbol-with-id object-type object-id)))
    (make-designator :location (append (string-case object-type
                                         ("FORK" `((:left-of ,plate-name)))
                                         ("KNIFE" `((:right-of ,plate-name)))
                                         ("MUG" `((:right-of ,plate-name)
                                                  (:behind ,plate-name))))
                                       `((:near ,plate-name)
                                         (:for ,object-name))))))

(defun assign-multiple-obj-pos (object-type &optional (object-number *num-of-sets-on-table*))
  (dotimes (i object-number)
    (move-object (new-symbol-with-id object-type i)
                 (string-case object-type
                   ("PLATE" (reference (make-plate-desig i)))
                   (t (reference (make-object-near-plate-desig object-type i)))))))

(defun set-table-without-robot ()
  (spawn-demo :table-setting :set :main)
  (move-demo-objects-away)
  (time
   (dolist (object-type (cdr (assoc :main *demo-object-types*)))
     (assign-multiple-obj-pos object-type))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; PROJECTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpl-impl:def-cram-function put-plate-on-table (plate-obj-desig)
  (cram-language-designator-support:with-designators
      ((on-kitchen-island :location `((:on "Cupboard")
                                      (:name "kitchen_island")
                                      (:for ,plate-obj-desig) (:context :table-setting) 
                                      (:object-count ,*num-of-sets-on-table*))))
    (format t "now trying to achieve the location of plate on kitchen-island~%")
    (plan-lib:achieve `(plan-lib:loc ,plate-obj-desig ,on-kitchen-island))))

(cpl-impl:def-cram-function put-plate-from-counter-on-table ()
  (sb-ext:gc :full t)
  (format t "Put a PLATE from counter on table~%")
  (let ((plate (find-object-on-counter :plate "kitchen_sink_block")))
    (sb-ext:gc :full t)
    (put-plate-on-table plate)
    plate))

(cpl-impl:def-cram-function put-object-from-counter-on-table (obj-type)
  (sb-ext:gc :full t)
  (format t "Put a PLATE from counter on table~%")
  (let ((object (find-object-on-counter obj-type "kitchen_sink_block")))
    (sb-ext:gc :full t)
    (put-plate-on-table object)
    object))

(cpl-impl:def-cram-function put-object-near-plate (object-to-put plate-obj
                                                   spatial-relations)
  (cram-language-designator-support:with-designators
      ((put-down-location :location `(,@(loop for property in spatial-relations
                                              collecting `(,property ,plate-obj))
                                      (:near ,plate-obj) (:for ,object-to-put)
                                      (:on "Cupboard"))))
    (plan-lib:achieve `(plan-lib:loc ,object-to-put ,put-down-location))))

(cpl-impl:def-cram-function put-object-from-counter-near-plate (object-type plate-obj)
  (format t "Put ~a from counter on table near ~a~%"
          object-type (desig-prop-value plate-obj :name))
  (sb-ext:gc :full t)
  (let ((obj (find-object-on-counter object-type "kitchen_sink_block")))
    (sb-ext:gc :full t)
    (put-object-near-plate obj plate-obj
                           (ecase object-type
                             (:fork '(:left-of))
                             (:knife '(:right-of))
                             (:mug '(:right-of :behind))))
    (sb-ext:gc :full t)))

(cpl-impl:def-top-level-cram-function put-stuff-on-table ()
  (cram-projection:with-projection-environment
      projection-process-modules::pr2-bullet-projection-environment
  (loop for i from 1 to *num-of-sets-on-table* do
    (let ((plate (put-plate-from-counter-on-table)))
      (mapcar (alexandria:rcurry #'put-object-from-counter-near-plate plate)
              '(:fork :knife :mug))))))


(defun set-table-in-projection ()
  (spawn-demo :table-setting :set :main)
  (put-stuff-on-table))

(defun teleport-a-plate ()
  (prolog `(and (assert (object-pose ?_ plate-1 ((-1.2 1.14 0.85747016972d0) (0 0 0 1)))))))

(defun bring-robot-to-table ()
  (move-demo-objects-away)
  (prolog `(and
            (assert (object-pose ?_ plate-1 ((-2.2 2.14 0.85747016972d0) (0 0 0 1))))
            (assert (object-pose ?_ plate-2 ((-1.75 2.14 0.85747016972d0) (0 0 0 1))))
            (assert (object-pose ?_ plate-3 ((-2.2 1.34 0.85747016972d0) (0 0 0 1))))
            (assert (object-pose ?_ plate-4 ((-1.75 1.34 2.85747016972d0) (0 0 0 1))))))
  (put-stuff-on-table))

(defun pick-and-place (type)
  (let ((obj-id (gensym)))
    (format t "id: ~a~%" obj-id)
    (prolog `(and (bullet-world ?w)
                  (cram-robot-interfaces:robot ?robot)
                  (assert (object ?w :mesh ,obj-id
                                  ((2 0 0) (0 0 0 1))
                                  :mesh ,type :mass 0.2 :color (0.8 0.3 0)
                                  :disable-collisions-with (?robot)
                                  ))))
    (move-object obj-id '((1.3 0.8 1.0) (0 0 0 1)))

    (cpl-impl:top-level
      (cram-projection:with-projection-environment
          projection-process-modules::pr2-bullet-projection-environment
        (let ((obj (put-object-from-counter-on-table type)))
          obj)))))




;; ;; desig for a glass near a plate
;; (setf des (desig:make-designator 'desig-props:location '((desig-props:right-of plate-1) (desig-props:behind plate-1) (desig-props:near plate-1) (desig-props:for mug-1) (desig-props:on counter-top) (desig-props:name kitchen-island))))

;; desig for a plate on a table
;; (setf des (desig:make-designator 'desig-props:location '((desig-props:on counter-top) (desig-props:name kitchen-island) (desig-props:context :table-setting) (desig-props:for plate-1) (desig-props:object-count 4))))

;; ;; desig for a fork to the left of plate
;; (setf des (desig:make-designator 'desig-props:location '((desig-props:left-of plate-1)  (desig-props:near plate-1) (desig-props:for fork-1))))
;; (force-ll (prolog `(and (symbol-value des ?des) (assign-object-pos fork-1 ?des))))
