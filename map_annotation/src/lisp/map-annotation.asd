
(asdf:defsystem map-annotation
  :depends-on ("cram-roslisp-common"
               "cl-transforms"
               "map_annotation-srv")
  :components
  ((:file "package")
   (:file "annotations" :depends-on ("package"))))
