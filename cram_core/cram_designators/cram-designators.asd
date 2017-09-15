; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-designators
  :name "cram-designators"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Implementation of designators."
  :long-description "Designators are the interface of cram do describe
                     all possible parameters that need reasoning. In particular this
                     includes object descriptions, locations and trajectories."
  :depends-on (:alexandria
               :cram-utilities
               :cram-prolog
               :trivial-garbage)
  :components
  ((:module "src"
    :components
    ((:module "cram-designators"
      :components
              ((:file "properties")
               (:file "package" :depends-on ("properties"))
               (:file "designator-protocol" :depends-on ("package"))
               (:file "designator-id-mixin" :depends-on ("package" "designator-protocol"))
               (:file "equate-notification-mixin" :depends-on ("package" "designator-protocol"))
               (:file "designator-pose" :depends-on ("package" "designator-protocol"))
               (:file "object-designator" :depends-on ("package"
                                                       "designator-id-mixin"
                                                       "equate-notification-mixin"
                                                       "designator-protocol"))
               (:file "action-designator" :depends-on ("package"
                                                       "designator-id-mixin"
                                                       "equate-notification-mixin"
                                                       "designator-protocol"))
               (:file "motion-designator" :depends-on ("package"
                                                       "designator-id-mixin"
                                                       "equate-notification-mixin"
                                                       "designator-protocol"))
               (:file "location-designator" :depends-on ("package"
                                                         "designator-id-mixin"
                                                         "equate-notification-mixin"
                                                         "designator-protocol"))
               (:file "human-designator" :depends-on ("package"
                                                      "designator-id-mixin"
                                                      "equate-notification-mixin"
                                                      "designator-protocol"))
               (:file "location-designator-filters"
                :depends-on ("package" "location-designator"))
               (:file "facts" :depends-on ("package"
                                           "designator-protocol"
                                           "object-designator"
                                           "location-designator"
                                           "action-designator"
                                           "motion-designator"))
               (:file "utils" :depends-on ("package"))
               (:file "initialization-macros" :depends-on ("package"))))))))
