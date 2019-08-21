#|
  This file is a part of quadtree project.
  Copyright (c) 2015 Masayuki Takagi (kamonama@gmail.com)
|#

(in-package :cl-user)
(defpackage quadtree-asd
  (:use :cl :asdf))
(in-package :quadtree-asd)

(defsystem quadtree
  :version "0.1"
  :author "Masayuki Takagi"
  :license "MIT"
  :depends-on (alexandria)
  :components ((:module "src"
                :components
                ((:file "quadtree"))))
  :description "Quadtree data structure in Common Lisp"
  :long-description
  #.(with-open-file (stream (merge-pathnames
                             #p"README.markdown"
                             (or *load-pathname* *compile-file-pathname*))
                            :if-does-not-exist nil
                            :direction :input)
      (when stream
        (let ((seq (make-array (file-length stream)
                               :element-type 'character
                               :fill-pointer t)))
          (setf (fill-pointer seq) (read-sequence seq stream))
          seq)))
  :in-order-to ((test-op (load-op quadtree-test))))
