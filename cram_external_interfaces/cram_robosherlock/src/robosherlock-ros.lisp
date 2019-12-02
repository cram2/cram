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

(defun make-robosherlock-action-client ()
  (actionlib-client:make-simple-action-client
   'robosherlock-action
   "RoboSherlock/query_action" "robosherlock_msgs/RSQueryAction"
   5))

(roslisp-utilities:register-ros-init-function make-robosherlock-action-client)

(defun make-robosherlock-query (key-value-pairs-list)
  (flet ((convert-entry (key &key (atom-otherwise-list t))
           (let ((key-val-pair (find key key-value-pairs-list :key #'car)))
             (if key-val-pair
                 (let ((value (second key-val-pair)))
                   (if atom-otherwise-list
                       (if (listp value)
                           (car value)
                           value)
                       (if (listp value)
                           (map 'vector #'identity value)
                           (vector value))))
                 (if atom-otherwise-list
                     ""
                     #(""))))))
    (roslisp:make-message
     'robosherlock_msgs-msg:rsquerygoal
     :obj (roslisp:make-message
           'robosherlock_msgs-msg:objectdesignator
           :uid (convert-entry :uid)
           :type (convert-entry :type)
           :shape (convert-entry :shape :atom-otherwise-list nil)
           :color (convert-entry :color :atom-otherwise-list nil)
           :location (convert-entry :location)
           :size (convert-entry :size)
           ;; :pose (convert-entry :pose :atom-otherwise-list nil)
           ;; :posesource (convert-entry :posesource :atom-otherwise-list nil)
           ))))

(defun ensure-robosherlock-input-parameters (key-value-pairs-list quantifier)
  (let ((key-value-pairs-list
          (remove-if
           #'null
           (mapcar (lambda (key-value-pair)
                     (destructuring-bind (key &rest values)
                         key-value-pair
                       (let ((value (car values)))
                         ;; below are the only keys supported by RS at the moment
                         (if (or (eql key :type)
                                 (eql key :shape)
                                 (eql key :color)
                                 (eql key :location)
                                 (eql key :size))
                             (list key
                                   (etypecase value ; RS is only case-sensitive on "TYPE"s
                                     (keyword (remove #\-
                                                      (if (eql key :type)
                                                          (string-capitalize (symbol-name value))
                                                          (string-downcase (symbol-name value)))))
                                     (string value)
                                     (list (mapcar (lambda (item)
                                                     (etypecase item
                                                       (keyword
                                                        (string-downcase (symbol-name item)))
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

(defun parse-result (message)
  (declare (type robosherlock_msgs-msg:objectdesignator message))
  "Returns a keyword key-value pairs list"
  (flet ((to-keyword (string)
           (if (string-equal string "")
               nil
               (roslisp-utilities:lispify-ros-underscore-name string :keyword))))
    `((:name ,(to-keyword ;; (roslisp:msg-slot-value message :uid)
                          (format nil "~a-1" (roslisp:msg-slot-value message :type))))
      (:type ,(to-keyword (roslisp:msg-slot-value message :type)))
      (:shape ,(map 'list #'to-keyword (roslisp:msg-slot-value message :shape)))
      (:color ,(map 'list #'to-keyword (roslisp:msg-slot-value message :color)))
      (:size ,(to-keyword (roslisp:msg-slot-value message :size)))
      (:location ,(to-keyword (roslisp:msg-slot-value message :location)))
      (:pose ,(map 'list #'cl-transforms-stamped:from-msg (roslisp:msg-slot-value message :pose)))
      (:posesource ,(map 'list #'to-keyword (roslisp:msg-slot-value message :posesource))))))

(defun ensure-robosherlock-result (result quantifier status)
  (when (or (eql status :preempted) (eql status :aborted) (not result))
    (cpl:fail 'common-fail:perception-low-level-failure
              :description "robosherlock didn't answer"))
  (let ((result (roslisp:msg-slot-value result :res)))
    (let ((number-of-objects (length result)))
      (when (< number-of-objects 1)
        (cpl:fail 'common-fail:perception-object-not-found
                  :description "couldn't find the object"))
      (format t "~%~%~%Robosherlock said: ~A~%~%~%" result)
      (etypecase quantifier
        (keyword (ecase quantifier
                   ((:a :an) (parse-result (aref result 0))
                    ;; this case should return a lazy list but I don't like them so...
                    )
                   (:the (if (= number-of-objects 1)
                             (parse-result (aref result 0))
                             (cpl:fail 'common-fail:perception-low-level-failure
                                       :description "There was more than one of THE object")))
                   (:all (map 'list #'parse-result result))))
        (number (if (= number-of-objects quantifier)
                    (map 'list #'parse-result result)
                    (cpl:fail 'common-fail:perception-low-level-failure
                              :description (format nil "perception returned ~a objects ~
                                                      although there should've been ~a"
                                                   number-of-objects quantifier))))))))

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
    (grey '(0.5 0.5 0.5))
    (t '(0.5 0.5 0.5))))

(defun which-estimator-for-object (object-description)
  (let ((type (second (find :type object-description :key #'car)))
        (cad-model (find :cad-model object-description :key #'car))
        (obj-part (find :obj-part object-description :key #'car)))
    (if cad-model
        :templatealignment
        (if (eq type :spoon)
            :2destimate
            (if obj-part
                :handleannotator
                :3destimate)))))

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
                                     "Robosherlock object didn't have a POSE from estimator ~a."
                                     estimator)))
    (second pose-description-we-want)))

(defun make-robosherlock-designator (rs-answer keyword-key-value-pairs-list)
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
              (:CupEcoOrange
               :cup)
              (:EdekaRedBowl
               :bowl)
              (:WeideMilchSmall
               :milk))))
      (setf rs-answer
            (subst-if `(:type ,cram-type)
                      (lambda (x)
                        (and (listp x) (eq (car x) :type)))
                      rs-answer))
      ;; REPLACE NAME WITH TYPE FOR HPN
      (setf rs-answer
            (subst-if `(:name ,cram-type)
                      (lambda (x)
                        (and (listp x) (eq (car x) :name)))
                      rs-answer))))
  ;; (when (and (find :location rs-answer :key #'car) ; <- LOCATION comes from original query
  ;;            (find :location keyword-key-value-pairs-list :key #'car))
  ;;   (setf rs-answer (remove :location rs-answer :key #'car)))
  (setf rs-answer (remove :location rs-answer :key #'car))
  ;; (when (and (find :color rs-answer :key #'car) ; <- COLOR comes from original query
  ;;            (find :color keyword-key-value-pairs-list :key #'car))
  ;;   (setf rs-answer (remove :color rs-answer :key #'car)))
  (setf rs-answer (remove :color rs-answer :key #'car)) ; <- if we don't do this
                                                        ; might end up asking about mutliple colors
  (setf rs-answer (remove :shape rs-answer :key #'car)); <- SHAPE comes from original query
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
                           :description "Robosherlock object didn't have a NAME")))
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
                                   (:transform-in-map ,transform-stamped-in-map-frame)))))))

          (let ((output-designator
                  (desig:make-designator :object output-properties)))

            (setf (slot-value output-designator 'desig:data)
                  (make-instance 'desig:object-designator-data
                    :object-identifier name
                    :pose pose-stamped-in-map-frame
                    :color color))

            output-designator))))))

(defun call-robosherlock-service (keyword-key-value-pairs-list &key (quantifier :all))
  (declare (type (or keyword number) quantifier))
  (multiple-value-bind (key-value-pairs-list quantifier)
      (ensure-robosherlock-input-parameters keyword-key-value-pairs-list quantifier)

    (multiple-value-bind (result status)
        (actionlib-client:call-simple-action-client
         'robosherlock-action
         :action-goal (make-robosherlock-query key-value-pairs-list))
      (let* ((rs-parsed-result (ensure-robosherlock-result result quantifier status))
             (rs-result (ecase quantifier
                          ((:a :an :the) (make-robosherlock-designator
                                          rs-parsed-result
                                          keyword-key-value-pairs-list))
                          (:all (map 'list (alexandria:rcurry #'make-robosherlock-designator
                                                              keyword-key-value-pairs-list)
                                     rs-parsed-result)))))
        rs-result))))



;; rosservice call /RoboSherlock/query "query: '{\"detect\": {\"type\": \"Wheel\"}}'"



;; (defvar *robosherlock-service* nil
;;   "Persistent service client for querying RoboSherlock JSON interface.")

;; (defparameter *robosherlock-service-name* "/RoboSherlock/query")

;; (defun init-robosherlock-service ()
;;   "Initializes *robosherlock-service* ROS publisher"
;;   (loop until (roslisp:wait-for-service *robosherlock-service-name* 5)
;;         do (roslisp:ros-info (robosherlock-service) "Waiting for robosherlock service."))
;;   (prog1
;;       (setf *robosherlock-service*
;;             (make-instance 'roslisp:persistent-service
;;               :service-name *robosherlock-service-name*
;;               :service-type 'robosherlock_msgs-srv:rsqueryservice))
;;     (roslisp:ros-info (robosherlock-service) "Robosherlock service client created.")))

;; (defun get-robosherlock-service ()
;;   (if (and *robosherlock-service*
;;            (roslisp:persistent-service-ok *robosherlock-service*))
;;       *robosherlock-service*
;;       (init-robosherlock-service)))

;; (defun destroy-robosherlock-service ()
;;   (when *robosherlock-service*
;;     (roslisp:close-persistent-service *robosherlock-service*))
;;   (setf *robosherlock-service* nil))

;; (roslisp-utilities:register-ros-cleanup-function destroy-robosherlock-service)

;; (defun make-robosherlock-query (detect-or-inspect &optional key-value-pairs-list)
;;   (let* ((query (reduce (lambda (query-so-far key-value-pair)
;;                           (concatenate 'string query-so-far
;;                                        (if (listp (second key-value-pair))
;;                                            (format nil "\"~a\":[\"~a\"], "
;;                                                    (first key-value-pair)
;;                                                    (first (second key-value-pair)))
;;                                            (format nil "\"~a\":\"~a\", "
;;                                                    (first key-value-pair)
;;                                                    (second key-value-pair)))))
;;                         key-value-pairs-list
;;                         :initial-value (format nil "{\"~a\":{"
;;                                                (string-downcase (symbol-name detect-or-inspect)))))
;;          (query-without-last-comma (string-right-trim '(#\, #\ ) query))
;;          (query-with-closing-bracket (concatenate 'string query-without-last-comma "}}")))
;;     (roslisp:make-request
;;      robosherlock_msgs-srv:rsqueryservice
;;      :query query-with-closing-bracket)))

;; (defun ensure-robosherlock-input-parameters (key-value-pairs-list quantifier)
;;   (let ((key-value-pairs-list
;;           (remove-if
;;            #'null
;;            (mapcar (lambda (key-value-pair)
;;                      (destructuring-bind (key &rest values)
;;                          key-value-pair
;;                        (let ((value (car values))
;;                              (key-string
;;                                (etypecase key
;;                                  (keyword (string-downcase (symbol-name key)))
;;                                  (string (string-downcase key)))))
;;                          ;; below are the only keys supported by RS at the moment
;;                          (if (or (string-equal key-string "type")
;;                                  (string-equal key-string "shape")
;;                                  (string-equal key-string "color")
;;                                  (string-equal key-string "cad-model")
;;                                  (string-equal key-string "location")
;;                                  (string-equal key-string "obj-part"))
;;                              (list key-string
;;                                    (etypecase value ; RS is only case-sensitive on "TYPE"s
;;                                      (keyword (remove #\- (string-capitalize (symbol-name value))))
;;                                      (string value)
;;                                      (list (mapcar (lambda (item)
;;                                                      (etypecase item
;;                                                        (keyword
;;                                                         (string-downcase (symbol-name item)))
;;                                                        (string
;;                                                         item)))
;;                                                    value))
;;                                      (desig:location-designator
;;                                       (desig:desig-prop-value
;;                                        (or (desig:desig-prop-value value :on)
;;                                            (desig:desig-prop-value value :in))
;;                                        :owl-name))))))))
;;                    key-value-pairs-list)))
;;         (quantifier quantifier
;;                     ;; (etypecase quantifier
;;                     ;;   (keyword (ecase quantifier
;;                     ;;              ((:a :an) :a)
;;                     ;;              (:the :the)
;;                     ;;              (:all :all)))
;;                     ;;   (number quantifier))
;;                     ))
;;     (values key-value-pairs-list quantifier)))

;; (defparameter *rs-result-debug* nil)
;; (defparameter *rs-result-designator* nil)
;; (defparameter *rs-query-debug* nil)
;; (defun call-robosherlock-service (detect-or-inspect keyword-key-value-pairs-list
;;                                   &key (quantifier :all))
;;   (declare (type (or keyword number) quantifier))
;;   (multiple-value-bind (key-value-pairs-list quantifier)
;;       (ensure-robosherlock-input-parameters keyword-key-value-pairs-list quantifier)

;;     (setf *rs-query-debug* key-value-pairs-list)
;;     (roslisp:with-fields (answer)
;;         (cpl:with-failure-handling
;;             (((or simple-error roslisp:service-call-error) (e)
;;                (format t "Service call error occured!~%~a~%Reinitializing...~%~%" e)
;;                (destroy-robosherlock-service)
;;                (init-robosherlock-service)
;;                (let ((restart (find-restart 'roslisp:reconnect)))
;;                  (if restart
;;                      (progn (roslisp:wait-duration 5.0)
;;                             (invoke-restart 'roslisp:reconnect))
;;                      (progn (cpl:retry))))))
;;           (roslisp:call-persistent-service
;;            (get-robosherlock-service)
;;            (make-robosherlock-query detect-or-inspect key-value-pairs-list)))
;;       (setf *rs-result-debug* answer)
;;       (let* ((rs-parsed-result (ensure-robosherlock-result answer quantifier))
;;              (rs-result (ecase quantifier
;;                           ((:a :an :the) (make-robosherlock-designator
;;                                           rs-parsed-result
;;                                           keyword-key-value-pairs-list))
;;                           (:all (map 'list (alexandria:rcurry #'make-robosherlock-designator
;;                                                               keyword-key-value-pairs-list)
;;                                      rs-parsed-result)))))
;;         (setf *rs-result-designator* rs-result)
;;         rs-result))))
