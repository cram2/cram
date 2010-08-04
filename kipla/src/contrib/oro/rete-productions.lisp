
(in-package :kipla-reasoning)

(defgeneric desig-prop->oro-fact (desig prop-type prop-value)
  (:documentation "`prop-type' is the desig property symbol, `
                   prop-value' the corresponding value. Returns an
                   assertion for oro."))

(defmethod desig-prop->oro-fact ((desig object-designator) (prop-type (eql 'type)) val)
  (let ((obj-type (case val
                    (mug "Cup")
                    (t "Object"))))
    (when obj-type
      `(,(object-id desig) "rdf:type" ,obj-type))))

(defmethod desig-prop->oro-fact ((desig object-designator) (prop-type (eql 'color)) val)
  `(,(object-id desig) "hasColor" ,(string-downcase (string val))))

(defmethod desig-prop->oro-fact ((desig object-designator) (prop-type (eql 'at)) val)
  (let ((supporting-obj (desig-prop-value val 'on)))
    (when supporting-obj
      `(,(object-id desig) "isOn" ,supporting-obj))))

(def-production oro-desig-bound
  (desig-bound ?desig ?_))

(defun do-oro-desig-assertion (op &key ?desig &allow-other-keys)
  (case op
    (:assert (apply #'oro-assert (reduce (lambda (facts curr)
                                           (let ((oro-fact (apply #'desig-prop->oro-fact ?desig curr)))
                                             (if oro-fact
                                                 (cons oro-fact facts)
                                                 facts)))
                                         (description ?desig)
                                         :initial-value `(("myself" "sees" ,(object-id ?desig))
                                                          ("myself" "knowsAbout" ,(object-id ?desig))))))
    (:retract (apply #'oro-retract `(("myself" "sees" ,(object-id ?desig)))))))

(register-production-handler 'oro-desig-bound #'do-oro-desig-assertion)
