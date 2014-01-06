(in-package :cl-user)

(desig-props:def-desig-package cram-tutorial-executive
  (:use #:roslisp #:cram-utilities #:designators-ros
        #:cram-roslisp-common #:cram-designators
        #:cram-plan-knowledge #:cram-plan-library #:cpl)
  (:import-from #:cram-reasoning #:<- #:def-fact-group)
  (:desig-properties #:to #:perceive #:obj #:ground
                     #:type #:box #:bowl #:cutlery
                     #:color #:red #:white #:blue #:green
                     #:move #:at #:loc
                     #:name #:container #:category #:owner #:cornflakes
                     #:pose))
