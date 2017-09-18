(in-package :tutorial-process-module)

(defmacro def-action-handler (name args &body body)
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))

(def-action-handler tutorial-action (message)
  (roslisp:ros-info (tutorial process-module) "Test action handler called.~%")
  (format t "Message: ~a~%" message)
  (format t "Here is also the place to add code for calling ROS services, doing file I/O, changing settings, and doing the overall low-level functionality."))

(def-action-handler perceive (obj-desig)
  "Returns a list of known virtual objects that match the description
given by `obj-desig'."
  (roslisp:ros-info (tutorial-pm perceive)
                    "Perceiving object with characteristics: ~a" obj-desig)
  (let* ((desc-search (description obj-desig))
         (objects-found
           (loop for known-object in *known-objects*
                 for desc-known = (description known-object)
                 when (description-matches desc-search desc-known)
                   collect (equate
                            obj-desig
                            (make-designator :object desc-known)))))
    objects-found))

(def-action-handler ground (obj-desig)
  "Looks up matching objects from the knowledge base that fit the
  description of `obj-desig' (without taking the `at' property into
  account)."
  (roslisp:ros-info (tutorial-pm perceive)
                    "Grounding object with characteristics: ~a" obj-desig)
  (let* ((desc-search (description obj-desig))
         (objects-found
           (loop for knowledge-object in *knowledge-base*
                 for desc-knowledge = (description knowledge-object)
                 when (description-matches desc-search desc-knowledge
                                           :ignore-properties
                                           `(desig-props:at))
                   collect (equate
                            obj-desig
                            (make-designator
                             'cram-designators:object
                             (join-descriptions desc-search desc-knowledge))))))
    objects-found))

(defun join-descriptions (desc-pri desc-sec)
  "Joins two description lists. Items in `desc-pri' take presedence
  over items in `desc-two'. Items not present in `desc-pri', but in
  `desc-sec' are added as well."
  (loop for item in desc-sec
        when (eql nil (position item desc-pri
                                :test (lambda (x y)
                                        (eql (car x) (car y)))))
          do (push item desc-pri))
  desc-pri)

(defun description-matches (desc-prototype desc-in-question
                            &key ignore-properties)
  "Rule to match: all properties from `desc-in-question' must match
the ones in `desc-prototype', but `desc-in-question' may include
additional properties."
  (or (eql desc-prototype nil)
      (reduce
       (lambda (x y) (and x y))
       (loop for prop-prototype in desc-prototype
             when (eql nil (position (car prop-prototype) ignore-properties
                                     :test (lambda (x y)
                                             (eql x y))))
               collect (not
                        (eql
                         nil
                         (position prop-prototype desc-in-question
                                   :test (lambda (p1 p2)
                                           (and (equal (car p1)
                                                       (car p2))
                                                (equal (cdr p1)
                                                       (cdr p2)))))))))))

(def-action-handler move (obj-desig pose)
  "Sets the current `at'-property (the virtual location) of object
`obj-desig' to `pose'."
  (roslisp:ros-info (tutorial-pm perceive)
                    "Moving object to pose: ~a" pose)
  (let ((obj-desc (append
                   (remove-if
                    (lambda (x)
                      (eql (car x) 'desig-props:at))
                    (description obj-desig))
                   `((desig-props:at ,(make-designator
                                       'cram-designators:location
                                       `((desig-props:pose ,pose))))))))
    (equate obj-desig
            (make-designator
             'cram-designators:object
             obj-desc))))
