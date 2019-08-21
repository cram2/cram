#|
  This file is a part of quadtree project.
  Copyright (c) 2015 Masayuki Takagi (kamonama@gmail.com)
|#

(in-package :cl-user)
(defpackage quadtree-test-asd
  (:use :cl :asdf))
(in-package :quadtree-test-asd)

(defsystem quadtree-test
  :author "Masayuki Takagi"
  :license "MIT"
  :depends-on (:quadtree
               :prove)
  :components ((:module "t"
                :components
                ((:test-file "quadtree"))))
  :description "Test for quadtree."
  :defsystem-depends-on (:prove-asdf)
  :perform (test-op :after (op c)
                    (funcall (intern #.(string :run-test-system) :prove.asdf) c)
                    (asdf:clear-system c)))
