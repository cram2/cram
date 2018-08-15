;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-bullet-reasoning-belief-state)

(defmethod cram-occasions-events:on-event btr-attach-object ((event cpoe:object-attached))
  (let* ((robot-object (btr:get-robot-object))
         (environment-object (btr:get-environment-object))
         (btr-object-name (cpoe:event-object-name event))
         (btr-object-name-string (symbol-name btr-object-name))
         (btr-object (btr:object btr:*current-bullet-world* btr-object-name))
         (link (cut:var-value
                '?ee-link
                (car (prolog:prolog
                      `(and (cram-robot-interfaces:robot ?robot)
                            (cram-robot-interfaces:end-effector-link ?robot ,(cpoe:event-arm event)
                                                                     ?ee-link)))))))
    (when (cut:is-var link) (error "[BTR-BELIEF OBJECT-ATTACHED] Couldn't find robot's EE link."))
    ;; first detach from environment in case it is attached
    (when (btr:object-attached environment-object btr-object)
      (btr:detach-object environment-object btr-object))
    ;; now attach to the robot-object
    (when btr-object
      (if (btr:object-attached robot-object btr-object)
          (btr:attach-object robot-object btr-object link :loose t)
          (btr:attach-object robot-object btr-object link :loose nil))

      ;;; giskard and knowrob event
      (unless cram-projection:*projection-environment*
        (let* ((map-to-ee-transform (cl-transforms-stamped:lookup-transform
                                     cram-tf:*transformer*
                                     cram-tf:*fixed-frame*
                                     link
                                     :timeout 2
                                     :time 0))
               (ee-to-map-transform (cram-tf:transform-stamped-inv map-to-ee-transform))
               (map-to-obj-transform (cram-tf:pose->transform-stamped
                                      cram-tf:*fixed-frame*
                                      btr-object-name-string
                                      0.0
                                      (btr:pose btr-object)))
               (ee-to-object-transform (cram-tf:multiply-transform-stampeds
                                        link btr-object-name-string
                                        ee-to-map-transform map-to-obj-transform))
               (ee-to-object-pose (cram-tf:strip-transform-stamped ee-to-object-transform)))

        ;;; knowrob event
          ;; (let ((origin (cl-transforms:origin ee-to-object-pose))
          ;;       (orientation (cl-transforms:orientation ee-to-object-pose)))
          ;;   (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z))
          ;;       origin
          ;;     (with-slots ((q1 cl-transforms:x) (q2 cl-transforms:y) (q3 cl-transforms:z)
          ;;                  (w cl-transforms:w))
          ;;         orientation
          ;;       (json-prolog:prolog-simple
          ;;        (let ((query
          ;;                (format nil "belief_at_update('~a', ([~f,~f,~f],[~f,~f,~f,~f]), ~
          ;;                           'http://knowrob.org/kb/PR2.owl#pr2_~a')."
          ;;                        btr-object-name
          ;;                        x y z q1 q2 q3 w
          ;;                        link)))
          ;;          query)))))

        ;;; giskard event
          ;; (call-giskard-environment-service
          ;;  :kill
          ;;  btr-object-name-string)
          ;; (call-giskard-environment-service
          ;;  :attached
          ;;  ;; btr-object-name-string
          ;;  "attached"
          ;;  ee-to-object-pose
          ;;  (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
          ;;      (btr:calculate-bb-dims btr-object)
          ;;    (list cl-transforms:x cl-transforms:y cl-transforms:z)))
          )))))

(defmethod cram-occasions-events:on-event btr-detach-object ((event cpoe:object-detached))
  (let* ((robot-object (btr:get-robot-object))
         (btr-object-name (cpoe:event-object-name event))
         (btr-object-name-string (symbol-name btr-object-name))
         (btr-object (btr:object btr:*current-bullet-world* btr-object-name))
         (link (cut:var-value
                '?ee-link
                (car (prolog:prolog
                      `(and (cram-robot-interfaces:robot ?robot)
                            (cram-robot-interfaces:end-effector-link ?robot ,(cpoe:event-arm event)
                                                                     ?ee-link)))))))

    (when (cut:is-var link) (error "[BTR-BELIEF OBJECT-DETACHED] Couldn't find robot's EE link."))
    (when btr-object
      (btr:detach-object robot-object btr-object link)
      (btr:simulate btr:*current-bullet-world* 10)

      (unless cram-projection:*projection-environment*
        ;; knowrob event
        ;; (let ((origin (cl-transforms:origin (btr:pose btr-object)))
        ;;       (orientation (cl-transforms:orientation (btr:pose btr-object))))
        ;;   (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z))
        ;;       origin
        ;;     (with-slots ((q1 cl-transforms:x) (q2 cl-transforms:y) (q3 cl-transforms:z)
        ;;                  (w cl-transforms:w))
        ;;         orientation
        ;;       (json-prolog:prolog-simple
        ;;        (let ((query
        ;;                (format nil "belief_at_update('~a', ([~f,~f,~f],[~f,~f,~f,~f]))."
        ;;                        btr-object-name x y z q1 q2 q3 w)))
        ;;          query)))))

        ;; giskard event
        ;; (call-giskard-environment-service
        ;;  :kill
        ;;  "attached")
        ;; (call-giskard-environment-service
        ;;  :add
        ;;  btr-object-name-string
        ;;  (cl-transforms-stamped:pose->pose-stamped
        ;;   cram-tf:*fixed-frame* 0.0 (btr:pose btr-object))
        ;;  (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
        ;;      (btr:calculate-bb-dims btr-object)
        ;;    (list cl-transforms:x cl-transforms:y cl-transforms:z)))
        ))))



;; (defmethod cram-occasions-events:on-event attach-objects ((event cpoe:object-attached))
;;   (let* ((robot (btr:get-robot-object))
;;          (current-event-object (desig:current-desig (cpoe:event-object event)))
;;          (object (get-designator-object current-event-object)))
;;     (when object
;;       (cond ((btr:object-attached robot object)
;;              ;; If the object is already attached, it is already in
;;              ;; the gripper. In that case, we update the designator
;;              ;; location designator by extending the current location
;;              ;; by a second pose in the gripper.
;;              (btr:attach-object robot object (cpoe:event-link event) :loose t)
;;              (desig:with-desig-props (at) current-event-object
;;                (assert (eql (desig:desig-prop-value at :in) :gripper))
;;                (update-object-designator-location
;;                 current-event-object
;;                 (desig:extend-designator-properties
;;                  at `((:pose ,(object-pose-in-frame object (cpoe:event-link event))))))))
;;             (t
;;              (btr:attach-object robot object (cpoe:event-link event) :loose nil)
;;              (update-object-designator-location
;;               current-event-object
;;               (desig:extend-designator-properties
;;                (make-object-location-in-gripper object (cpoe:event-link event))
;;                `((:pose ,(object-pose-in-frame object cram-tf:*robot-base-frame*))))))))
;;     (btr:timeline-advance
;;      btr:*current-timeline*
;;      (btr:make-event
;;       btr:*current-bullet-world*
;;       `(pick-up ,(cpoe:event-object event) ,(cpoe:event-side event))))))

;; (defmethod cram-occasions-events:on-event detach-objects ((event cpoe:object-detached))
;;   (let ((robot (btr:get-robot-object))
;;         (object (get-designator-object (cpoe:event-object event))))
;;     (when object
;;       (btr:detach-object robot object (cpoe:event-link event))
;;       (btr:simulate btr:*current-bullet-world* 10)
;;       (update-object-designator-location
;;        (desig:current-desig (cpoe:event-object event))
;;        (make-object-location (get-designator-object-name (cpoe:event-object event)))))
;;     (btr:timeline-advance
;;      btr:*current-timeline*
;;      (btr:make-event
;;       btr:*current-bullet-world*
;;       `(put-down ,(cpoe:event-object event) ,(cpoe:event-side event))))
;;     (btr:timeline-advance
;;      btr:*current-timeline*
;;      (btr:make-event
;;       btr:*current-bullet-world*
;;       `(location-change ,(cpoe:event-object event))))))

(defmethod cram-occasions-events:on-event robot-moved ((event cpoe:robot-state-changed))
  (unless cram-projection:*projection-environment*
    (let ((robot (btr:get-robot-object)))
      (when robot
        (btr:set-robot-state-from-tf
         cram-tf:*transformer*
         robot
         :timestamp (cram-occasions-events:event-timestamp event)))))
  (btr:timeline-advance
   btr:*current-timeline*
   (btr:make-event
    btr:*current-bullet-world*
    `(location-change robot))))

(defmethod cram-occasions-events:on-event open-or-close-object
    ((event cpoe:object-articulation-event))
  (with-slots (cpoe:object-designator cpoe:opening-distance) event
    (let ((perceived-object (desig:reference
                             (desig:newest-effective-designator cpoe:object-designator)))
          (semantic-map-object
            (cut:with-vars-strictly-bound (?semantic-map)
                (cut:lazy-car
                 (prolog `(and
                           (btr:bullet-world ?world)
                           (btr:semantic-map ?world ?semantic-map-name)
                           (btr:%object ?world ?semantic-map-name ?semantic-map))))
              ?semantic-map)))
      (btr:set-articulated-object-joint-position
       semantic-map-object
       (desig:object-identifier perceived-object)
       cpoe:opening-distance))))



;; TODO(cpo): Think about adding timeline advancement.
;; TODO(cpo): Limit manipulation-distance to one axis.
#+comment-out
(let ((previous-tcp-pose))
  (labels ((get-current-tcp (event)
             (let ((arm (cpoe:environment-event-arm event)))
               (cl-tf:lookup-transform
                cram-tf:*transformer*
                (cut:var-value
                 '?frame
                 (car (prolog:prolog
                       `(and (cram-robot-interfaces:robot ?robot)
                             (cram-robot-interfaces:robot-tool-frame ?robot ,arm ?frame)))))
                cram-tf:*fixed-frame*)))
           (move-joint-by-event (event open-or-close)
             (let ((joint-name (cpoe:environment-event-joint-name event))
                   (object (cpoe:environment-event-object event))
                   (current-tcp-pose (get-current-tcp event)))
               (progn
                 (let* ((current-opening (gethash joint-name
                                                  (btr:joint-states object)))
                        (manipulation-distance
                          (cl-tf:v-dist (cl-tf:translation previous-tcp-pose)
                                        (cl-tf:translation current-tcp-pose)))
                        (new-joint-angle
                          (apply
                           (case open-or-close
                             (:open #'+)
                             (:close #'-))
                           (list
                            current-opening
                            manipulation-distance))))
                   (btr:set-robot-state-from-joints
                    `((,joint-name
                       ,new-joint-angle))
                    object))
                 (setf previous-tcp-pose NIL)))))

    (defmethod cram-occasions-events:on-event grasp-container-handle
        ((event cpoe:container-handle-grasping-event))
      (setf previous-tcp-pose (get-current-tcp event)))

    (defmethod cram-occasions-events:on-event open-container
        ((event cpoe:container-opening-event))
      (move-joint-by-event event :open))

    (defmethod cram-occasions-events:on-event close-container
        ((event cpoe:container-closing-event))
      (move-joint-by-event event :close))))

(defun move-joint-by-event (event open-or-close)
  (let* ((joint-name (cpoe:environment-event-joint-name event))
         (object (cpoe:environment-event-object event))
         (distance (cpoe:environment-event-distance event))
         (current-opening (gethash joint-name (btr:joint-states object)))
         (new-joint-angle
           (funcall
            (case open-or-close
              (:open #'+)
              (:close #'-))
            current-opening
            distance)))
    (btr:set-robot-state-from-joints
     `((,joint-name
        ,new-joint-angle))
     object)))

(defmethod cram-occasions-events:on-event open-container ((event cpoe:container-opening-event))
  (move-joint-by-event event :open)
  (unless cram-projection:*projection-environment*
    (publish-environment-joint-state (btr:joint-states (cpoe:environment-event-object event)))))

(defmethod cram-occasions-events:on-event close-container ((event cpoe:container-closing-event))
  (move-joint-by-event event :close)
  (unless cram-projection:*projection-environment*
    (publish-environment-joint-state (btr:joint-states (cpoe:environment-event-object event)))))



(defmethod cram-occasions-events:on-event object-perceived ((event cpoe:object-perceived-event))
  (if cram-projection:*projection-environment*
      ;; if in projection, only add the object name to perceived designators list
      (let ((object-data (desig:reference (cpoe:event-object-designator event))))
        (or
         (gethash (desig:object-identifier object-data)
                  *object-identifier-to-instance-mappings*)
         (setf (gethash (desig:object-identifier object-data)
                        *object-identifier-to-instance-mappings*)
               (desig:object-identifier object-data))))
      ;; otherwise, spawn a new object in the bullet world
      (progn
        (register-object-designator-data
         (desig:reference (cpoe:event-object-designator event))
         :type (desig:desig-prop-value (cpoe:event-object-designator event) :type))

        (let* ((object-name (desig:desig-prop-value (cpoe:event-object-designator event) :name))
               (object-name-string (symbol-name object-name))
               (btr-object (btr:object btr:*current-bullet-world* object-name)))
          ;; (call-giskard-environment-service
          ;;  :kill
          ;;  object-name-string)
          ;; (call-giskard-environment-service
          ;;  :add
          ;;  object-name-string
          ;;  (cl-transforms-stamped:pose->pose-stamped
          ;;   cram-tf:*fixed-frame* 0.0 (btr:pose btr-object))
          ;;  (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
          ;;      (btr:calculate-bb-dims btr-object)
          ;;    (list cl-transforms:x cl-transforms:y cl-transforms:z)))
          ))))

(defun update-object-designator-location (object-designator location-designator)
  (desig:make-designator
   :object
   `((:at ,location-designator)
     ,@(remove :at (desig:properties object-designator) :key #'car))
   object-designator))

(defun get-supporting-object-bounding-box (object-name)
  (cut:with-vars-bound (?supporting-name ?supporting-link)
      (cut:lazy-car (prolog
                     `(or (btr:supported-by ?_ ,object-name ?supporting-name ?supporting-link)
                          (btr:supported-by ?_ ,object-name ?supporting-name))))
    (unless (cut:is-var ?supporting-name)
      (if (cut:is-var ?supporting-link)
          (btr:aabb (btr:object btr:*current-bullet-world* ?supporting-name))
          (btr:aabb (gethash
                     ?supporting-link
                     (btr:links (btr:object btr:*current-bullet-world* ?supporting-name))))))))

(defun make-object-location (object-name)
  (let ((object (btr:object btr:*current-bullet-world* object-name)))
    (assert object)
    (desig:make-designator
     :location
     `((:pose ,(cl-transforms-stamped:pose->pose-stamped
                cram-tf:*fixed-frame*
                (cut:current-timestamp)
                (cl-bullet:pose object)))))))

(defun make-object-location-in-gripper (object gripper-link)
  "Returns a new location designator that indicates a location in the
  robot's gripper."
  (declare (type btr:object object))
  (let* ((object-pose (cl-transforms-stamped:pose->pose-stamped
                       cram-tf:*fixed-frame*
                       0.0
                       (btr:pose object)))
         (robot (btr:get-robot-object)))
    (assert (member gripper-link (btr:object-attached robot object) :test #'equal))
    (let ((supporting-bounding-box (get-supporting-object-bounding-box (btr:name object))))
      (desig:make-designator
       :location
       `((:in :gripper) (:pose ,(object-pose-in-frame object gripper-link))
         (:z-offset ,(cond (supporting-bounding-box
                            (- (cl-transforms:z (cl-transforms:origin object-pose))
                               (+ (cl-transforms:z
                                   (cl-bullet:bounding-box-center supporting-bounding-box))
                                  (/ (cl-transforms:z
                                      (cl-bullet:bounding-box-dimensions
                                       supporting-bounding-box))
                                     2))))
                           (t 0.0))))))))

(defun object-pose-in-frame (object frame)
  (declare (type btr:object object)
           (type string frame))
  (cl-transforms-stamped:copy-pose-stamped
   (cl-transforms-stamped:transform-pose-stamped
    cram-tf:*transformer*
    :pose (cl-transforms-stamped:pose->pose-stamped
           cram-tf:*fixed-frame*
           0.0
           (btr:pose object))
    :target-frame frame
    :timeout cram-tf:*tf-default-timeout*)
   :stamp 0.0))
