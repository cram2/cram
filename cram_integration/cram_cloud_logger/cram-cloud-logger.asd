(defsystem cram-cloud-logger
  :depends-on (cram-language
               :cram-designators
               :cl-transforms
               :cl-transforms-stamped
               :cram-json-prolog
               :roslisp)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "cloud-logger-client")))))
