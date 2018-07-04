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
     (:file "utils" :depends-on ("package"))
     (:file "object-interface" :depends-on ("package" "utils"))
     (:file "cloud-logger-client" :depends-on ("package" "utils"))
     (:file "cloud-logger-query-handler" :depends-on ("package" "cloud-logger-client" "utils"))
     (:file "prolog-query-handler" :depends-on ("package" "utils" "cloud-logger-client"))
     (:file "knowrob-action-name-handler" :depends-on ("package" "utils"))
     (:file "logging-functions" :depends-on ("package" "cloud-logger-query-handler"))
     (:file "action-parameter-handler" :depends-on ("package" "logging-functions"))
     (:file "utils-for-perform" :depends-on ("package" "cloud-logger-query-handler" "prolog-query-handler" "object-interface" "knowrob-action-name-handler" "action-parameter-handler" "utils"))))))
