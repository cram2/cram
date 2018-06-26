(defsystem cram-cloud-logger
  :depends-on (cram-language
               :cram-designators
               :cl-transforms
               :cl-transforms-stamped
               :cram-json-prolog
               :roslisp
               cram-utilities)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "cloud-logger-client" :depends-on ("package"))
     (:file "utils-for-perform" :depends-on ("package" "cloud-logger-client"))))))
