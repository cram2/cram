(defsystem cram-intermediate-tutorial
  :depends-on (geometry_msgs-msg shape_msgs-msg
               roslisp cl-transforms cl-tf2
               cram-moveit)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "cram-moveit-tutorial" :depends-on  ("package"))))))


