(defsystem cram-cloud-logger
  :depends-on (cram-language
               :cram-json-prolog
               :roslisp)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "cloud-logger-client")))))
