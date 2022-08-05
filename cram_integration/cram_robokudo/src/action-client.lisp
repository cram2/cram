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

(in-package :rk)

(defparameter *ros-action* "robokudo/query")

(defun make-robokudo-action-client ()
  (actionlib-client:make-simple-action-client
   'robokudo-action
   *ros-action* "robokudo_msgs/QueryAction"
   120))

(roslisp-utilities:register-ros-init-function make-robokudo-action-client)

;;;;;;;;;;;;;;;;;;; INPUT ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-robokudo-query (key-value-pairs-list)
  (flet ((convert-entry (key &key (atom-or-list :atom))
           (let ((key-val-pair (find key key-value-pairs-list :key #'car)))
             (if key-val-pair
                 (let ((value (second key-val-pair)))
                   (if (eq atom-or-list :atom)
                       (if (listp value)
                           (car value)
                           value)
                       (if (listp value)
                           (map 'vector #'identity value)
                           (vector value))))
                 (if (eq atom-or-list :atom)
                     ""
                     #(""))))))
    (roslisp:make-message
     'robokudo_msgs-msg:querygoal
     :obj (roslisp:make-message
           'robokudo_msgs-msg:objectdesignator
           :uid (convert-entry :uid)
           :type (convert-entry :type)
           :shape (convert-entry :shape :atom-or-list :list)
           :color (convert-entry :color :atom-or-list :list)
           :location (convert-entry :location)
           :size (convert-entry :size)
           ;; :pose (convert-entry :pose :atom-otherwise-list nil)
           ;; :posesource (convert-entry :posesource :atom-otherwise-list nil)
           ))))

(defun ensure-robokudo-input-parameters (key-value-pairs-list quantifier)
  (let ((key-value-pairs-list
          (remove-if
           #'null
           (mapcar (lambda (key-value-pair)
                     (destructuring-bind (key &rest values)
                         key-value-pair
                       (let ((value (car values)))
                         ;; below are the only keys supported by RS atm
                         (if (or (eql key :type)
                                 (eql key :shape)
                                 (eql key :color)
                                 (eql key :location)
                                 (eql key :size)
                                 (eql key :material))
                             (list key
                                   (etypecase value
                                     ;; RS is only case-sensitive on "TYPE"s
                                     (keyword
                                      (remove #\-
                                              (if (eql key :type)
                                                  (string-capitalize
                                                   (symbol-name
                                                    (case value
                                                      ;; (:bowl :ikea-red-bowl)
                                                      ;; (:cup :ikea-red-cup)
                                                      ;; (:spoon :soup-spoon)
                                                      (t value))))
                                                  (string-downcase
                                                   (symbol-name value)))))
                                     (string
                                      value)
                                     (list
                                      (mapcar (lambda (item)
                                                (etypecase item
                                                  (keyword
                                                   (string-downcase
                                                    (symbol-name item)))
                                                  (string
                                                   item)))
                                              value))
                                     (desig:location-designator
                                      (desig:desig-prop-value
                                       (or (desig:desig-prop-value value :on)
                                           (desig:desig-prop-value value :in))
                                       :owl-name))))))))
                   key-value-pairs-list)))
        (quantifier quantifier
                    ;; (etypecase quantifier
                    ;;   (keyword (ecase quantifier
                    ;;              ((:a :an) :a)
                    ;;              (:the :the)
                    ;;              (:all :all)))
                    ;;   (number quantifier))
                    ))
    (values key-value-pairs-list quantifier)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;; OUTPUT ;;;;;;;;;;;;;;;;;;;;;;;;

(defun parse-result (message)
  (declare (type robokudo_msgs-msg:objectdesignator message))
  "Returns a keyword key-value pairs list"
  (flet ((to-keyword (string)
           (if (string-equal string "")
               nil
               (roslisp-utilities:lispify-ros-underscore-name string :keyword))))
    `((:name ,(to-keyword
               ;; (roslisp:msg-slot-value message :uid)
               (format nil "~a-1" (roslisp:msg-slot-value message :type))))
      (:type ,(to-keyword (roslisp:msg-slot-value message :type)))
      (:shape ,(map 'list #'to-keyword (roslisp:msg-slot-value message :shape)))
      (:color ,(map 'list #'to-keyword (roslisp:msg-slot-value message :color)))
      (:size ,(to-keyword (roslisp:msg-slot-value message :size)))
      (:location ,(to-keyword (roslisp:msg-slot-value message :location)))
      (:pose ,(map 'list #'cl-transforms-stamped:from-msg (roslisp:msg-slot-value message :pose)))
      (:posesource ,(map 'list #'to-keyword (roslisp:msg-slot-value message :posesource))))))

(defun ensure-robokudo-result (result quantifier status)
  (when (or (eql status :preempted) (eql status :aborted) (not result))
    (cpl:fail 'common-fail:perception-low-level-failure
              :description "robokudo didn't answer"))
  (let ((result (roslisp:msg-slot-value result :res)))
    (let ((number-of-objects (length result)))
      (when (< number-of-objects 1)
        (cpl:fail 'common-fail:perception-object-not-found
                  :description "couldn't find the object"))
      (etypecase quantifier
        (keyword (ecase quantifier
                   ;; this case should return a lazy list
                   ;; but I don't like them so...
                   ((:a :an) (parse-result (aref result 0)))
                   (:the (if (= number-of-objects 1)
                             (parse-result (aref result 0))
                             (cpl:fail 'common-fail:perception-low-level-failure
                                       :description (format nil
                                                            "There was more ~
                                                             than one of THE ~
                                                             object"))))
                   (:all (map 'list #'parse-result result))))
        (number (if (= number-of-objects quantifier)
                    (map 'list #'parse-result result)
                    (cpl:fail 'common-fail:perception-low-level-failure
                              :description (format nil "perception returned ~a ~
                                                        objects although there ~
                                                        should've been ~a"
                                                   number-of-objects
                                                   quantifier))))))))

(defun map-rs-color-to-rgb-list (rs-color)
  (when (stringp rs-color)
    (setf rs-color (intern (string-upcase rs-color) :keyword)))
  (case rs-color
    (:white '(1.0 1.0 1.0))
    (:red '(1.0 0.0 0.0))
    (:orange '(1.0 0.5 0.0))
    (:yellow '(1.0 1.0 0.0))
    (:green '(0.0 1.0 0.0))
    (:blue '(0.0 0.0 1.0))
    (:cyan '(0.0 1.0 1.0))
    (:purple '(1.0 0.0 1.0))
    (:black '(0.0 0.0 0.0))
    (:grey '(0.5 0.5 0.5))
    (t '(0.5 0.5 0.5))))

(defun which-estimator-for-object (object-description)
  :ClusterPosePCAAnnotator
  ;; (let ((type (second (find :type object-description :key #'car)))
  ;;       (cad-model (find :cad-model object-description :key #'car))
  ;;       (obj-part (find :obj-part object-description :key #'car)))
  ;;   (if cad-model
  ;;       :templatealignment
  ;;       (if (eq type :spoon)
  ;;           :3destimate ;:2destimate
  ;;           (if obj-part
  ;;               :handleannotator
  ;;               :3destimate))))
  )

(defun find-pose-in-object-designator (object-description)
  (let* ((estimator
           (which-estimator-for-object object-description))
         (all-poses
           (second (find :pose object-description :key #'car)))
         (all-posesources
           (second (find :posesource object-description :key #'car)))
         (pose-description-we-want
           (find-if (lambda (source-pose-pair)
                      (eq (car source-pose-pair) estimator))
                    (mapcar #'list all-posesources all-poses))))
    (unless pose-description-we-want
      (cpl:fail 'common-fail:perception-low-level-failure
                :description (format nil
                                     "Robokudo object didn't have a POSE from estimator ~a."
                                     estimator)))
    (second pose-description-we-want)))

(defun make-robokudo-designator (rs-answer keyword-key-value-pairs-list)
  (when (and (find :type rs-answer :key #'car)
             (find :type keyword-key-value-pairs-list :key #'car))
    ;; TYPE comes from original query
    ;; (setf rs-answer (remove :type rs-answer :key #'car))
    ;;
    ;; Actually not, because if we want to perceive "type KitchenObject",
    ;; we get KitchenObject back and that's not nice.
    ;; so for now we'll have a mapping...
    (let ((cram-type
            (ecase (second (find :type rs-answer :key #'car))
              (:KoellnMuesliKnusperHonigNuss
               :breakfast-cereal)
              (:muesli
               :breakfast-cereal)
              (:milk
               :milk)
              (:CupEcoOrange
               :cup)
              (:EdekaRedBowl
               :bowl)
              (:IkeaRedBowl
               :bowl)
              (:SoupSpoon
               :spoon)
              (:spoon
               :spoon)
              (:IkeaRedCup
               :cup)
              (:bowl
               :bowl)
              (:cup
               :cup)
              (:mug
               :cup)
              (:WeideMilchSmall
               :milk)
              (:BLUEPLASTICSPOON
               :spoon)
              (:BALEAREINIGUNGSMILCHVITAL
               :balea-bottle)
              (:DENKMITGESCHIRRREINIGERNATURE
               :dish-washer-tabs)
              (:GarnierMineralUltraDry
               :deodorant)
              (:DMRoteBeteSaftBio
               :juice-box)
              (:JeroenCup
               :jeroen-cup))))
      (setf rs-answer
            (subst-if `(:type ,cram-type)
                      (lambda (x)
                        (and (listp x) (eq (car x) :type)))
                      rs-answer))
      ;; REPLACE NAME WITH TYPE FOR HPN
      ;; (setf rs-answer
      ;;       (subst-if `(:name ,cram-type)
      ;;                 (lambda (x)
      ;;                   (and (listp x) (eq (car x) :name)))
      ;;                 rs-answer))
      ))
  ;; (when (and (find :location rs-answer :key #'car) ; <- LOCATION comes from original query
  ;;            (find :location keyword-key-value-pairs-list :key #'car))
  ;;   (setf rs-answer (remove :location rs-answer :key #'car)))
  (setf rs-answer (remove :location rs-answer :key #'car))
  (when (and (find :color rs-answer :key #'car) ; <- COLOR comes from original query
             (find :color keyword-key-value-pairs-list :key #'car))
    (setf rs-answer (remove :color rs-answer :key #'car)))
  ;; (setf rs-answer (remove :color rs-answer :key #'car))
                                        ; <- if we don't do this
                                        ; might end up asking about mutliple colors
  (setf rs-answer (remove :material rs-answer :key #'car)) ; <- if we don't do this
                                        ; might end up asking about different materials
  (setf rs-answer (remove :shape rs-answer :key #'car)); <- SHAPE comes from original query
  (setf rs-answer (remove :size rs-answer :key #'car)) ; <- don't care about size
  ;; (when (and (find :pose rs-answer :key #'car)
  ;;            (find :pose keyword-key-value-pairs-list :key #'car))
  ;;   (remove :pose keyword-key-value-pairs-list :key #'car))
  (setf keyword-key-value-pairs-list (remove :pose keyword-key-value-pairs-list :key #'car))
  (let ((combined-properties
          (append rs-answer ; <- overwrite old stuff with new stuff
                  (set-difference keyword-key-value-pairs-list rs-answer :key #'car))))
    (let* ((name
             (or (second (find :name combined-properties :key #'car))
                 (cpl:fail 'common-fail:perception-low-level-failure
                           :description "Robokudo object didn't have a NAME")))
           (color
             (let ((rs-colors (assoc :color combined-properties
                                     :test #'equal)))
               (if rs-colors
                   (map-rs-color-to-rgb-list
                    (if (listp (cadr rs-colors))
                        (caadr rs-colors)
                        (cadr rs-colors)))
                   '(0.5 0.5 0.5)))))

      (let* ((pose-stamped-in-whatever
               (find-pose-in-object-designator combined-properties))
             (pose-stamped-in-base-frame
               (if cram-tf:*robot-base-frame*
                   (cram-tf:ensure-pose-in-frame
                    pose-stamped-in-whatever
                    cram-tf:*robot-base-frame*
                    :use-zero-time t)
                   pose-stamped-in-whatever))
             (transform-stamped-in-base-frame
               (cram-tf:pose-stamped->transform-stamped
                pose-stamped-in-base-frame
                (roslisp-utilities:rosify-underscores-lisp-name name)))
             (pose-stamped-in-map-frame
               (if cram-tf:*fixed-frame*
                   (cram-tf:ensure-pose-in-frame
                    pose-stamped-in-whatever
                    cram-tf:*fixed-frame*
                    :use-zero-time t)
                   pose-stamped-in-whatever))
             (transform-stamped-in-map-frame
               (cram-tf:pose-stamped->transform-stamped
                pose-stamped-in-map-frame
                (roslisp-utilities:rosify-underscores-lisp-name name))))

        (cram-tf:visualize-marker pose-stamped-in-whatever :r-g-b-list '(0 0 1) :id 1234)

        (let* ((properties-without-pose
                 (remove :pose combined-properties :key #'car))
               (output-properties
                 (append properties-without-pose
                         `((:pose ((:pose ,pose-stamped-in-base-frame)
                                   (:transform ,transform-stamped-in-base-frame)
                                   (:pose-in-map ,pose-stamped-in-map-frame)
                                   (:transform-in-map ,transform-stamped-in-map-frame)
                                   (:pose-in-camera ,pose-stamped-in-whatever)))))))

          (let ((output-designator
                  (desig:make-designator :object output-properties)))

            (setf (slot-value output-designator 'desig:data)
                  (make-instance 'desig:object-designator-data
                    :object-identifier name
                    :pose pose-stamped-in-map-frame
                    :color color))

            output-designator))))))

;;;;;;;;;;;;;;;;; ACTION ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun call-robokudo-action (keyword-key-value-pairs-list &key (quantifier :all))
  (declare (type (or keyword number) quantifier))
  (multiple-value-bind (key-value-pairs-list quantifier)
      (ensure-robokudo-input-parameters keyword-key-value-pairs-list quantifier)

    (multiple-value-bind (result status)
        (actionlib-client:call-simple-action-client
         'robokudo-action
         :action-goal (make-robokudo-query key-value-pairs-list))
      (let* ((rs-parsed-result (ensure-robokudo-result result quantifier status))
             (rs-result (ecase quantifier
                          ((:a :an :the) (make-robokudo-designator
                                          rs-parsed-result
                                          keyword-key-value-pairs-list))
                          (:all (map 'list (alexandria:rcurry #'make-robokudo-designator
                                                              keyword-key-value-pairs-list)
                                     rs-parsed-result)))))
        rs-result))))
