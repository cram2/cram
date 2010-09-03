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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :kipla-reasoning)

;;; Location designators are resolved a little bit differently than
;;; object designators. To resolve, the cram/reasoning prolog
;;; predicate desig-loc is used. All solutions are provided an can be
;;; accessed with next-solution. A mechanism is provided to
;;; post-process the solutions from reasoning, e.g. to sort according
;;; to eucledian distance.

(defconstant +costmap-n-samples+ 5)

(defgeneric make-location-proxy (type value)
  (:documentation "Creates a location proxy of `type' and initializes
  it with `value'."))

(defgeneric location-proxy-current-solution (proxy)
  (:documentation "Returns the current solution of the proxy"))

(defgeneric location-proxy-next-solution (proxy)
  (:documentation "Returns the next solution of the proxy object or
  NIL if no more solutions exist."))

(defgeneric location-proxy-precedence-value (proxy)
  (:documentation "Returns a number that indicates the proxie's
  precedence. Lower numbers correspond to lower precedence."))

(defgeneric location-proxy-solution->pose (desig solution)
  (:method (desig (solution cl-transforms:pose))
    (cl-tf:make-pose-stamped "/map" (roslisp:ros-time)
                             (cl-transforms:origin solution)
                             (cl-transforms:orientation solution)))
  (:method (desig (solution cl-transforms:transform))
    (cl-tf:make-pose-stamped "/map" (roslisp:ros-time)
                             (cl-transforms:translation solution)
                             (cl-transforms:rotation solution)))
  (:method (desig (solution cl-tf:pose-stamped))
    solution))

(defclass pose-location-proxy ()
  ((pose :accessor location-proxy-current-solution :initarg :pose))
  (:documentation "Proxy class for designators that contain poses."))

(defclass point-location-proxy ()
  ((point :accessor location-proxy-current-solution :initarg :pose))
  (:documentation "Proxy class for designators that contain poses."))

(defclass costmap-location-proxy (point-location-proxy)
  ((next-solutions :accessor :next-solutions :initform nil
                   :documentation "List of the next solution. We want
                   to minimize driving distances, so we always
                   generate a bunch of solutions, order them by
                   distance to the robot and always chose the closest
                   one when generating a new solution.")
   (costmap :initarg :costmap :reader costmap))
  (:documentation "Proxy class to generate designator solutions from a
  costmap."))

(defclass location-designator (designator designator-id-mixin)
  ((current-solution :reader current-solution :initform nil)))

(register-designator-type location location-designator)

(defmethod reference ((desig location-designator))
  (with-slots (data current-solution) desig
    (unless current-solution
      (setf data (mapcar (curry #'apply #'make-location-proxy)
                         (sort (mapcar (curry #'var-value '?value)
                                       (force-ll (prolog `(desig-loc ,desig ?value))))
                               #'> :key (compose #'location-proxy-precedence-value #'car))))
      (assert data () (format nil "Unable to resolve designator `~a'" desig))
      (setf current-solution (location-proxy-solution->pose
                              desig
                              (location-proxy-current-solution (car data))))
      (assert current-solution () (format nil "Unable to resolve designator `~a'" desig))
      (assert-desig-binding desig current-solution))
    current-solution))

(defmethod next-solution ((desig location-designator))
  ;; Make sure that we initialized the designator properly
  (unless (slot-value desig 'current-solution)
    (reference desig))
  (with-slots (data) desig
    (or (successor desig)
        (let ((new-desig (make-designator 'location (description desig))))
          (or
           (let ((next (location-proxy-next-solution (car data))))
             (when next
               (setf (slot-value new-desig 'data) data)
               (setf (slot-value new-desig 'current-solution)
                     (location-proxy-solution->pose new-desig next))
               (prog1 (equate desig new-desig)
                 (assert-desig-binding new-desig (slot-value new-desig 'current-solution)))))
           (when (cdr data)
             (let ((next (location-proxy-current-solution (cadr data))))
               (when next
                 (setf (slot-value new-desig 'data) (cdr data))
                 (setf (slot-value new-desig 'current-solution)
                       (location-proxy-solution->pose new-desig next))
                 (prog1
                     (equate desig new-desig)
                   (assert-desig-binding new-desig (slot-value new-desig 'current-solution)))))))))))

;; Todo: make the poses stamped
(defmethod make-location-proxy ((type (eql 'point)) (value cl-transforms:3d-vector))
  (make-instance 'pose-location-proxy
                 :pose (cl-transforms:make-pose
                        value (cl-transforms:make-quaternion 0 0 0 1))))

(defmethod make-location-proxy ((type (eql 'pose)) (value cl-transforms:transform))
  (make-instance 'pose-location-proxy
                 :pose (cl-transforms:make-pose (cl-transforms:translation value)
                                                (cl-transforms:rotation value))))

(defmethod make-location-proxy ((type (eql 'pose)) (value cl-transforms:pose))
  (make-instance 'pose-location-proxy
                 :pose value))

(defmethod make-location-proxy ((type (eql 'pose)) (value cl-tf:pose-stamped))
  (make-instance 'pose-location-proxy :pose value))

(defmethod make-location-proxy ((type (eql 'pose)) (value cl-tf:stamped-transform))
  (make-instance 'pose-location-proxy
                 :pose (cl-tf:make-pose-stamped
                        (cl-tf:frame-id value) (cl-tf:stamp value)
                        (cl-transforms:translation value)
                        (cl-transforms:rotation value))))

(defmethod location-proxy-precedence-value ((type (eql 'pose)))
  1)

(defmethod location-proxy-next-solution ((proxy pose-location-proxy))
  nil)

(defmethod make-location-proxy ((type (eql 'costmap)) (val location-costmap))
  (make-instance 'costmap-location-proxy :costmap val))

(defmethod location-proxy-precedence-value ((type (eql 'costmap)))
  0)

(defmethod initialize-instance :after ((proxy costmap-location-proxy) &key &allow-other-keys)
  (location-proxy-next-solution proxy))

(defmethod location-proxy-next-solution ((proxy costmap-location-proxy))
  (flet ((take-closest-point (points)
           (let ((closest (car points))
                 (dist (cl-transforms:v-dist (cl-transforms:translation
                                              (cl-tf:lookup-transform
                                               *tf*
                                               :target-frame "/map"
                                               :source-frame "/base_link"))
                                             (car points))))
             (dolist (p (cdr points) closest)
               (let ((new-dist (cl-transforms:v-dist (cl-transforms:translation
                                                      (cl-tf:lookup-transform
                                                       *tf*
                                                       :target-frame "/map"
                                                       :source-frame "/base_link"))
                                                     p)))
                 (when (< new-dist dist)
                   (setf dist new-dist)
                   (setf closest p)))))))
    (with-slots (point next-solutions costmap)
        proxy
      (let ((solutions (or next-solutions
                           (loop repeat +costmap-n-samples+
                                 collecting (generate-point costmap)))))
        (prog1 (setf point (take-closest-point solutions))
          (setf next-solutions (delete point solutions)))))))

(defmethod location-proxy-solution->pose (desig (solution cl-transforms:3d-vector))
  (with-vars-bound (?o)
      (lazy-car (prolog `(desig-orientation ,desig ,solution ?o)))
    (with-vars-bound (?z)
        (lazy-car (prolog `(desig-z-value ,desig ,solution ?z)))
      (cl-tf:make-pose-stamped
       "/map" (roslisp:ros-time)
       (cl-transforms:make-3d-vector
        (cl-transforms:x solution)
        (cl-transforms:y solution)
        (if (is-var ?z) 0.0d0 ?z))
       (if (is-var ?o)
           (cl-transforms:make-quaternion 0 0 0 1)
           ?o)))))
