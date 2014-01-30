;; TODO: fix the bsd macro snippet thing

(defsystem cram-plan-transformations
  :author "gaya"
  :license "BSD"
  :description "ToDo"

  :depends-on (cram-reasoning
               cram-language
               designators
               cram-plan-library
               cram-plan-knowledge
               cram-utilities
               alexandria)
  :components
  ((:module "src"
           :components
           ((:file "package")
            (:file "parse-nl" :depends-on ("package"))))))

  