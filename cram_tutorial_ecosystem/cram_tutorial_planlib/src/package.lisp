(in-package :cl-user)

(desig-props:def-desig-package cram-tutorial-planlib
  (:use #:roslisp #:cram-utilities #:designators-ros
        #:cram-roslisp-common #:cram-designators
        #:cram-plan-knowledge #:cram-plan-library #:cpl
	#:cram-plan-failures)
  (:import-from #:cram-reasoning #:<- #:def-fact-group)
  (:export ambiguous-perception perceive-object a the all)
  (:desig-properties #:to #:perceive #:obj #:ground
                     #:type #:box #:bowl #:cutlery
                     #:color #:red #:white #:blue #:green
                     #:move #:at #:loc
                     #:name #:container #:category #:owner #:cornflakes
                     #:pose #:all #:a #:the))
