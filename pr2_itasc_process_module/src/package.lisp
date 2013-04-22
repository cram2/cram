(in-package :cl-user)

(desig-props:def-desig-package pr2-itasc-process-module
  (:nicknames :pr2-itasc-pm)
  (:use #:common-lisp
        #:cut
        #:cram-roslisp-common
        #:cram-process-modules)
  (:export pr2-itasc-process-module))
