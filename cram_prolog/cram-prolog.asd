; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-prolog
  :name "cram-prolog"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Prolog-like language interpreter and reasoning framework."
  :long-description "Prolog-like language interpreter for keeping
                     dynamic knowledge bases and integrating other
                     sources of knowledge such as PROLOG through a
                     foreign language interface. It also provides an
                     implementation of the RETE algorithm."
  :depends-on (alexandria
               cram-utilities)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "prolog-handlers" :depends-on ("package"))
             (:file "fact-groups" :depends-on ("package" "prolog-handlers"))
             (:file "prolog-facts" :depends-on ("package" "fact-groups"))
             (:file "prolog" :depends-on ("package" "fact-groups" "prolog-handlers"))
             (:file "built-in-predicates" :depends-on ("package" "prolog-handlers" "prolog"))
             (:file "rete-prolog-handlers" :depends-on ("package" "prolog-handlers" "rete"))
             (:file "utils" :depends-on ("package"))
             (:module "rete"
                      :depends-on ("package")
                      :components
                      ((:file "rete")
                       (:file "alpha-node")
                       (:file "alpha-memory-node")
                       (:file "alpha-network")
                       (:file "token")
                       (:file "beta-join-node")
                       (:file "prolog-node")
                       (:file "productions")
                       (:file "prove" :depends-on ("alpha-network")))
                      :serial t)
             (:file "swank-indentation" :depends-on ("package"))))))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-prolog))))
  (asdf:operate 'asdf:load-op 'cram-prolog-tests)
  (asdf:operate 'asdf:test-op 'cram-prolog-tests))
