(in-package :su-real)

;;;;;;;;;;;;;;;;;;;;;;;; KNOWROB EVENT HANDLERS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod coe:on-event giskard-attach-object ((event cpoe:object-attached-robot-knowrob))
  (unless cram-projection:*projection-environment*
    (if (eq (cpoe:event-other-object-name event) (rob-int:get-environment-name))
        (roslisp:ros-warn (giskard coll-scene)
                          "Attaching objects to environment is not supported yet.")
        (attach-object-to-arm-in-collision-scene-knowrob
         (cpoe:event-object-name event)
         (cpoe:event-arm event)
         (cpoe:event-link event)))))

(defmethod coe:on-event giskard-detach-object 1 ((event cpoe:object-detached-robot-knowrob))
  (unless cram-projection:*projection-environment*
    (let ((object-name (cpoe:event-object-name event)))
      (if object-name
          ;; if object-name is given, detach given object
          ;; (detach-object-in-collision-scene-knowrob object-name)
          (print "x")
          ;; otherwise detach all objects from the given arm
          (let* ((arm
                   (cpoe:event-arm event))
                 (link
                   (if arm
                       (cut:var-value
                        '?link
                        (car
                         (prolog:prolog
                          `(and (rob-int:robot ?rob)
                                (rob-int:end-effector-link ?rob ,arm ?link)))))
                       (cpoe:event-link event))))
            (unless (cut:is-var link)
              (mapcar #'detach-object-in-collision-scene-knowrob
                      (btr:link-attached-object-names
                       (btr:get-robot-object)
                       link))))))))

(defmethod coe:on-event giskard-detach-object-after 3 ((event cpoe:object-detached-robot-knowrob))
  ;; TODO: How do we get the updated pose? manipulation sends it back maybe? or when manipulation
  ;; sends back :SUCCEEDED after placing, we update the pose to the targetpose?
  ;; when we are not under timepressure when executing the plan, it might be good to perceive
  ;; the object again to confirm, which would give us the exact pose again.
  (unless cram-projection:*projection-environment*
    (roslisp:with-fields ((frame (cl-transforms-stamped:frame-id))
                          (w0 (w cl-transforms:orientation))
                          (w1 (x cl-transforms:orientation))
                          (w2 (y cl-transforms:orientation))
                          (w3 (z cl-transforms:orientation))
                          (x (x cl-transforms:origin))
                          (y (y cl-transforms:origin))
                          (z (z cl-transforms:origin)))
        (cpoe::event-pose event)
      (su-demos::with-knowledge-result ()
          `("object_pose" ,(cpoe::event-object-name event) (list ,frame
                                  (list ,(su-demos::round-for-knowrob x)
                                        ,(su-demos::round-for-knowrob y)
                                        ,(su-demos::round-for-knowrob (+ z (/ (cpoe::event-height event) 2))))
                                  (list ,(su-demos::round-for-knowrob w1)
                                        ,(su-demos::round-for-knowrob w2)
                                        ,(su-demos::round-for-knowrob w3)
                                        ,(su-demos::round-for-knowrob w0))))
        (print "updated")
        (break)
    ;; (update-object-pose-in-collision-scene-knowrob (cpoe:event-object-name event))
        ))))

(defmethod coe:on-event giskard-perceived ((event cpoe:object-perceived-event-knowrob))
  (unless cram-projection:*projection-environment*
    (roslisp:with-fields ((frame (cl-transforms-stamped:frame-id cram-designators::pose cram-designators:data))
                          (w0 (w cl-transforms:orientation cram-designators::pose cram-designators:data))
                          (w1 (x cl-transforms:orientation cram-designators::pose cram-designators:data))
                          (w2 (y cl-transforms:orientation cram-designators::pose cram-designators:data))
                          (w3 (z cl-transforms:orientation cram-designators::pose cram-designators:data))
                          (x (x cl-transforms:origin cram-designators::pose cram-designators:data))
                          (y (y cl-transforms:origin cram-designators::pose cram-designators:data))
                          (z (z cl-transforms:origin cram-designators::pose cram-designators:data))
                          (description (cram-designators:description))
                          (radius (robokudo_msgs-msg::radius cram-designators::objectsize cram-designators:data))
                          (x-size (x_size robokudo_msgs-msg::dimensions cram-designators::objectsize cram-designators:data))
                          (y-size (y_size robokudo_msgs-msg::dimensions cram-designators::objectsize cram-designators:data))
                          (z-size (z_size robokudo_msgs-msg::dimensions cram-designators::objectsize cram-designators:data)))
        (cpoe:event-object-designator event)
      (su-demos::with-knowledge-result (name)
          `("create_object" name ,(su-demos::transform-key-to-string (second (second description)))  ;;TODO Extract keyword
                            (list ,frame
                                  (list ,(su-demos::round-for-knowrob x)
                                        ,(su-demos::round-for-knowrob y)
                                        ,(su-demos::round-for-knowrob z))
                                  (list ,(su-demos::round-for-knowrob w1)
                                        ,(su-demos::round-for-knowrob w2)
                                        ,(su-demos::round-for-knowrob w3)
                                        ,(su-demos::round-for-knowrob w0)))
                            (list ("shape" ("box" 0.145 0.06 0.22))))
        (print name)
        ;; (add-object-to-collision-scene-knowrob
        ;;  name)
        ))))


;;;;;;;;;;;;;;;;;;;;;;;; KNOWROB UTILS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun reset-collision-scene-knowrob ()
  (giskard::call-giskard-environment-service
   :remove-all)
  (when (btr:get-environment-object)
    (giskard::call-giskard-environment-service
     :add-environment
     :name (roslisp-utilities:rosify-underscores-lisp-name
            (rob-int:get-environment-name))
     :pose (cl-transforms-stamped:pose->pose-stamped
            cram-tf:*fixed-frame* 0.0 (btr:pose (btr:get-environment-object)))
     :joint-state-topic "iai_kitchen/joint_states")))

(defun update-object-pose-in-collision-scene-knowrob (object-name)
  (su-demos::with-knowledge-result (frame shape pose)
      `(and ("object_shape_workaround" ,object-name frame shape _ _)
            ("object_pose" ,object-name pose))
    (when object-name
      (let* ((object-name-string
               frame)
             (pose-stamped (su-demos::make-pose-stamped-from-knowledge-result pose)))
        (giskard::call-giskard-environment-service
         :alter
         :name object-name-string
         :pose pose-stamped
         ;; :mesh-path (when btr-object
         ;;              (second (assoc (car (btr:item-types btr-object))
         ;;                             btr::*mesh-files*)))
         :dimensions (cdr shape))))))

(defun add-object-to-collision-scene-knowrob (object-name)
  (su-demos::with-knowledge-result (frame shape pose)
      `(and ("object_shape_workaround" ,object-name frame shape _ _)
            ("object_pose" ,object-name pose))
    (let* ((object-name-string
             frame)
           (robot-links-object-is-attached-to
             (list "map"))
           (pose-stamped (su-demos::make-pose-stamped-from-knowledge-result pose)))
    ;; to update an object pose, first remove the old object together with the pose
    (when (member object-name-string (giskard::get-groups) :test #'string=)
      (giskard::call-giskard-environment-service
       :remove
       :name object-name-string))
    ;; add it at the new perceived pose
    (giskard::call-giskard-environment-service
     :add
     :name object-name-string
     :pose pose-stamped
     ;; :mesh-path (when btr-object
     ;;              (second (assoc (car (btr:item-types btr-object))
     ;;                             btr::*mesh-files*)))
     :dimensions (cdr shape))
    ;; reattach the object if it was attached somewhere
    (when robot-links-object-is-attached-to
      (let ((link (car robot-links-object-is-attached-to)))
        (giskard::call-giskard-environment-service
         :attach
         :name object-name-string
         :pose pose-stamped
         ;; :mesh-path (when btr-object
         ;;              (second (assoc (car (btr:item-types btr-object))
         ;;                             btr::*mesh-files*)))
         :dimensions (cdr shape)
         :parent-link link))))))

(defun detach-object-in-collision-scene-knowrob (object-name)
  (su-demos::with-knowledge-result (frame)
      `("object_shape_workaround" ,object-name frame _ _ _)
    (let* ((object-name-string
             frame))
      ;; TODO: this might need to be replaced properly, talk to knowledge and
      ;; manipulation if this is needed
      ;; (when btr-object
      ;;   (let ((attached-to-another-link-as-well?
      ;;           (> (length
      ;;               (btr:object-name-attached-links
      ;;                (btr:get-robot-object)
      ;;                object-name))
      ;;              1)))
      ;;     (unless attached-to-another-link-as-well?
      (giskard::call-giskard-environment-service
       :detach
       :name object-name-string))))

(defun attach-object-to-arm-in-collision-scene-knowrob (object-name arm link)
  (su-demos::with-knowledge-result (frame shape pose)
      `(and ("object_shape_workaround" ,object-name frame shape _ _)
            ("object_pose" ,object-name pose))
    (let* ((object-name-string
             frame)
           (pose-stamped (su-demos::make-pose-stamped-from-knowledge-result pose))
           (link (if arm
                     (cut:var-value
                      '?ee-link
                      (car (prolog:prolog
                            `(and (rob-int:robot ?robot)
                                  (rob-int:end-effector-link ?robot ,arm ?ee-link)))))
                     link))
           )
      (when (cut:is-var link)
        (error "[GISKARD OBJECT-ATTACHED] Couldn't find robot's EE link."))
      (let* ((map-to-ee-transform
               (cl-transforms-stamped:lookup-transform
                cram-tf:*transformer*
                cram-tf:*fixed-frame*
                link
                :timeout cram-tf:*tf-default-timeout*
                :time 0))
             (ee-to-map-transform
               (cram-tf:transform-stamped-inv map-to-ee-transform))
             (map-to-obj-transform
               (cram-tf:pose->transform-stamped
                cram-tf:*fixed-frame*
                object-name-string
                0.0
                pose-stamped))
             (ee-to-object-transform
               (cram-tf:multiply-transform-stampeds
                link object-name-string
                ee-to-map-transform map-to-obj-transform))
             (ee-to-object-pose
               (cram-tf:strip-transform-stamped ee-to-object-transform)))
        ;; remove the object first, maybe it was already attached to something
        (giskard::call-giskard-environment-service
         :detach
         :name object-name-string)
        (giskard::call-giskard-environment-service
         :attach
         :name object-name-string
         :pose ee-to-object-pose
         ;; :mesh-path (when btr-object
         ;;              (second (assoc (car (btr:item-types btr-object))
         ;;                             btr::*mesh-files*)))
         :dimensions (cdr shape)
         :parent-link link
         :parent-link-group (roslisp-utilities:rosify-underscores-lisp-name
                             (rob-int:get-robot-name)))))))

(defun full-update-collision-scene-knowrob ()
  ;; TODO is the object attached to anything? how do we represent that in knowledge?
  (mapcar (lambda (object)
            (when (typep object 'btr:item)
              (let ((object-name (btr:name object)))
                (giskard::add-object-to-collision-scene-knowrob object-name)
                (when (btr:object-attached
                       (btr:get-robot-object)
                       (btr:object btr:*current-bullet-world* object-name))
                  (giskard::attach-object-to-arm-in-collision-scene-knowrob
                   object-name
                   nil
                   (car (btr:object-name-attached-links
                         (btr:get-robot-object)
                         object-name)))))))
          (btr:objects btr:*current-bullet-world*)))


(defun giskard-testing ()
     (let* ((?source-object-desig
                     (desig:an object
                               (type :crackerbox)))
            (?object-desig
              (exe:perform (desig:an action
                                     (type detecting)
                                     (object ?source-object-desig))))
            (?knowledge-name 
              (roslisp:with-fields ((frame (cl-transforms-stamped:frame-id cram-designators::pose cram-designators:data))
                                    (w0 (w cl-transforms:orientation cram-designators::pose cram-designators:data))
                                    (w1 (x cl-transforms:orientation cram-designators::pose cram-designators:data))
                                    (w2 (y cl-transforms:orientation cram-designators::pose cram-designators:data))
                                    (w3 (z cl-transforms:orientation cram-designators::pose cram-designators:data))
                                    (x (x cl-transforms:origin cram-designators::pose cram-designators:data))
                                    (y (y cl-transforms:origin cram-designators::pose cram-designators:data))
                                    (z (z cl-transforms:origin cram-designators::pose cram-designators:data))
                                    (radius (robokudo_msgs-msg::radius cram-designators::objectsize cram-designators:data))
                                    (x-size (x_size robokudo_msgs-msg::dimensions cram-designators::objectsize cram-designators:data))
                                    (y-size (y_size robokudo_msgs-msg::dimensions cram-designators::objectsize cram-designators:data))
                                    (z-size (z_size robokudo_msgs-msg::dimensions cram-designators::objectsize cram-designators:data)))
                  ?object-desig
                (su-demos::with-knowledge-result (name)
                    `("create_object" name (|:| "soma" "CrackerBox")
                            (list ,frame
                                  (list ,(su-demos::round-for-knowrob x)
                                        ,(su-demos::round-for-knowrob y)
                                        ,(su-demos::round-for-knowrob z))
                                  (list ,(su-demos::round-for-knowrob w1)
                                        ,(su-demos::round-for-knowrob w2)
                                        ,(su-demos::round-for-knowrob w3)
                                        ,(su-demos::round-for-knowrob w0)))
                            (list ("shape" ("box" ,(coerce y-size 'single-float)
                                                  ,(coerce x-size 'single-float)
                                                  ,(coerce z-size 'single-float)))))
                  name)
                (print frame))))
       (break)
       (add-object-to-collision-scene-knowrob ?knowledge-name)
       (print ?knowledge-name)
       (break)
       (su-demos::with-knowledge-result (frame)
           `("object_shape_workaround" ,?knowledge-name frame _ _ _)
         (let ((?object-name frame))
           (exe:perform (desig:an action
                                  (type picking-up)
                                  (object-name ?object-name)
                                  (collision-mode :allow-all)))))))
