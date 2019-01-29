(defsystem cram-sim-log-generator
  :depends-on (cram-language
               :cram-designators
               :cl-transforms
               :cl-transforms-stamped
               :cram-json-prolog
               :roslisp
               :cram-pr2-pick-place-demo
               :cram-cloud-logger
               cram-utilities
                cram-learning-framework
               )
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "main" :depends-on ("package"))
     (:file "neem-generator" :depends-on ("package"))))))
