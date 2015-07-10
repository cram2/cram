
(asdf:defsystem map-annotation
  :depends-on ("cram-roslisp-common"
               "cl-transforms"
               "cram-prolog"
               "map_annotation-srv")
  :components
  ((:file "package")
   (:file "annotations" :depends-on ("package"))
   (:file "facts" :depends-on ("package"))))
