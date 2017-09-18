(in-package :cl-user)

(desig-props:def-desig-package cram-tutorial-planlib
  (:use #:roslisp #:cram-utilities #:cram-tf
        #:cram-roslisp-common #:cram-designators
        #:cram-plan-occasions-events #:cram-plan-library #:cpl
	#:cram-plan-failures)
  (:import-from #:cram-reasoning #:<- #:def-fact-group)
  (:export ambiguous-perception perceive-object)
  (:desig-properties #:to #:perceive #:obj #:ground
                     #:type #:box #:bowl #:cutlery
                     #:color #:red #:white #:blue #:green
                     #:move #:at #:loc
                     #:name #:container #:category #:owner #:cornflakes
                     #:pose #:all #:a #:the))
