
(in-package :kipla)

(defun refine-type (prev-type)
  (let* ((new-type (web_hri-srv:answer-val
                     (call-service "/hri/ask_human" 'web_hri-srv:askhuman
                                   :question (format nil "What is a ~a?" prev-type))))
         (type (car (oro-call "lookup" new-type "class"))))
    (oro-call "add" `((,prev-type "rdfs:subClassOf" ,(or type new-type))))
    (unless type
      (refine-type new-type))))

(defun ask-human-for-obj (obj)
  (let* ((obj-name-str
          (web_hri-srv:answer-val
           (call-service "/hri/ask_human" 'web_hri-srv:askhuman
                         :question (format nil "What is the name of the ~a ~a in my hand?"
                                           (or (desig-prop-value obj 'color) "unknown")
                                           (or (desig-prop-value obj 'type) "unknown")))))
         (obj-type-str
          (web_hri-srv:answer-val
           (call-service "/hri/ask_human" 'web_hri-srv:askhuman
                         :question (format nil "What kind of object is ~a?"
                                           obj-name-str)))))
    (oro-call "add" `((,(object-id obj) "rdfs:label"
                        ,(format nil "\\\"~a\\\"" obj-name-str))))
    (let ((real-type (car (oro-call "lookup" obj-type-str "class"))))
      (oro-call "add" `((,(object-id obj) "rdf:type"
                          ,(format nil  "~a" (or real-type obj-type-str)))))
      (unless real-type
        (refine-type obj-type-str)))))

(defun ask-human-for-more-objects ()
  (string-equal (web_hri-srv:answer-val
                 (call-service "/hri/ask_human" 'web_hri-srv:askhuman
                               :question "Do you want me to learn more objects?"))
                "yes"))

(defun show-object (obj)
  (with-designators ((show-action (action '((type trajectory) (to show) (side :right))))
                     (loc (location `((to reach) (obj ,obj)))))
    (look-at (jlo:make-jlo :name "/look_forward"))
    (at-location (loc)
      (achieve `(arms-at ,show-action)))))

(def-plan odd-objects-out ()
  (let ((tableware-objects (oro-call "find" '?o '((?o "rdf:type" "Tableware"))))
        (food-objects (oro-call "find" '?o '((?o "rdf:type" "Food"))))
        (objects (append (oro-call "find" '?o '((?o "rdf:type" "Artifact")))
                         (oro-call "find" '?o '((?o "rdf:type" "Food"))))))
    (call-service "/hri/tell_human" 'web_hri-srv:askhuman
                  :question (format nil "These objects are odd: ~{~a~^, ~}~%"
                                    (mapcar (lambda (x)
                                              (or (car (oro-call "find" '?l `((,x "rdfs:label" ?l))))
                                                  x))
                                            (remove-if (lambda (x)
                                                         (or
                                                          (member x tableware-objects :test #'string-equal)
                                                          (member x food-objects :test #'string-equal)))
                                                       objects))))))


(def-plan learn-objects (terminated)
  (format t "learn objects~%")
  (with-designators
      ((table (location '((on table))))
       (obj (object `((type object) (at ,table)))))
    (with-failure-handling
        ((object-not-found (f)
           (declare (ignore f))
           (setf (value terminated) t)))
      (format t "searching for objects~%")
      (let ((visible-objects (perceive-all obj))
            (previously-seen-objects (mapcar (alexandria:curry #'var-value '?d)
                                             (holds `(desig-refined ?d)))))
        (format t "found objects. asking human.~%")
        (loop for obj in (reverse visible-objects)
              do (setf previously-seen-objects
                       (delete obj previously-seen-objects :test #'desig-equal))
                 (format t "prev-seen-objs ~a ~a~%" obj previously-seen-objects)
              unless (holds `(desig-refined ,obj))
                do (let ((object-loc (object-pose (reference obj))))
                     (format t "object at ~a~%" object-loc)
                     (with-designators ((put-down-loc (location `((jlo ,object-loc)))))
                       (achieve `(object-in-hand ,obj :right))
                       (show-object obj)
                       (ask-human-for-obj obj)
                       (assert-occasion `(desig-refined ,obj))
                       (achieve `(object-placed-at ,obj ,put-down-loc))
                       (achieve `(arm-parked :right)))))
        ;; Retract all objects we cannot see anymore
        (loop for obj in previously-seen-objects
              do (progn
                   (retract-desig-binding obj (reference obj))))))))

(def-top-level-plan hri-control ()
  (let ((terminated (make-fluent :name 'terminated-fluent :value nil)))
    (pursue
      (run-process-modules)
      (wait-for terminated)
      (loop
        do (sleep 0.1)
           (format t "learning objects~%")
           (learn-objects terminated)
        while (ask-human-for-more-objects)))
    (odd-objects-out)))

