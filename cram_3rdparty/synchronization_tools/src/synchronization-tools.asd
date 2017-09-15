
(defclass asdf::synchronized-tools-source-file (asdf:cl-source-file) ())

(defmethod asdf:perform :around ((o asdf:compile-op)
                                 (c asdf::synchronized-tools-source-file))
  (let ((sb-ext:*derive-function-types* t))
    (call-next-method)))

(asdf:defsystem :synchronization-tools
  :author "Tobias C. Rittweiler <trittweiler@common-lisp.net>"
  :licence "BSD"
  :depends-on (:sb-rt :sb-queue)
  :default-component-class asdf::synchronized-tools-source-file
  :components
  ((:file "synchronization-tools")
   (:file "tests" :depends-on ("synchronization-tools"))))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system :synchronization-tools))))
  (let ((*package* (find-package :sb-rt)))
    (funcall (intern (string '#:do-tests) *package*))))
