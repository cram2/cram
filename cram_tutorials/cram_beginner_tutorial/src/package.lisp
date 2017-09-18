(defpackage :cram-beginner-tutorial
  (:nicknames :tut)
  (:use :cpl :roslisp :cl-transforms :cram-designators :cram-process-modules
        :cram-language-designator-support)
  (:import-from :cram-prolog :def-fact-group :<- :lisp-fun))
