(in-package :cl-user)

(desig-props:def-desig-package tutorial-process-module
  (:nicknames :tut-pm)
  (:use #:common-lisp
        #:crs
        #:cut
        #:desig
        #:designators-ros
        #:cram-roslisp-common
        #:cram-process-modules
        #:cram-plan-failures
        #:cram-plan-knowledge)
  (:import-from alexandria ignore-some-conditions)
  (:import-from #:cram-plan-knowledge
                matching-process-module available-process-module)
  (:export tutorial-process-module generate-random-location)
  (:desig-properties #:to #:perceive #:obj #:ground
                     #:type #:box #:bowl #:cutlery
                     #:color #:red #:white #:blue #:green
                     #:move #:at #:loc
                     #:name #:container #:category #:owner #:cornflakes
                     #:pose))
