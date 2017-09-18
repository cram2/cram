(defsystem cram-intermediate-tutorial
  :depends-on (geometry_msgs-msg shape_msgs-msg
               roslisp roslisp-utilities cl-transforms cl-tf2
               cram-moveit actionlib)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "cram-moveit-tutorial" :depends-on  ("package"))))))


