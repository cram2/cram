(defsystem cram-intermediate-tutorial
  :depends-on (roslisp cram-language cl-transforms geometry_msgs-msg designators cram-reasoning 
                 cram-language-designator-support actionlib actionlib_tutorials-msg process-modules cram-moveit cl-tf2)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "cram-moveit-tutorial" :depends-on  ("package"))))))


