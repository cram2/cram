(defsystem cram-cloud-logger
  :depends-on (:cram-language
               :cram-designators
               :cl-transforms
               :cl-transforms-stamped
               :cram-json-prolog
               :roslisp
               :cram-utilities
               :cram-manipulation-interfaces
               :cram-executive
               :cram-projection)
  :components
  ((:module "src" 
    :components
    ((:file "package")
     (:module "mapper" 
      :components 
      ((:file "cram-2-knowrob-mapper")))
     (:file "utils" :depends-on ("package"))
     (:file "failure-handler" :depends-on ("mapper"))
     (:file "cloud-logger-client" :depends-on ("package" "utils"))
     (:file "cloud-logger-query-handler" :depends-on ("package" "cloud-logger-client" "utils"))
     (:file "prolog-query-handler" :depends-on ("package" "utils" "cloud-logger-client"))
     (:file "knowrob-action-name-handler" :depends-on ("package" "utils"))
     (:file "logging-functions" :depends-on ("package" "cloud-logger-query-handler"))
     (:file "action-parameter-handler" :depends-on ("package" "logging-functions"))
     (:file "utils-for-perform" :depends-on ("package" "cloud-logger-query-handler" "prolog-query-handler" "knowrob-action-name-handler" "action-parameter-handler" "utils" "failure-handler"))
     (:file "object-interface" :depends-on ("package" "utils" "cloud-logger-client" "utils-for-perform"))))))
