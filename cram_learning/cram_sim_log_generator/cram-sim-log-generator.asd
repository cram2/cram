(defsystem cram-sim-log-generator
  :depends-on (:cram-language
               :cram-designators
               :cl-transforms
               :cl-transforms-stamped
               :cram-json-prolog
               :roslisp
               :cram-pr2-pick-place-demo
               :cram-pr2-process-modules
               :cram-pr2-description
               :cram-urdf-projection
               :cram-cloud-logger
               :cram-utilities)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "main" :depends-on ("package"))
     (:file "neem-generator" :depends-on ("package"))))))
