(in-package :demos)
"A wrapper which allows to generate a NEEM of a demo"

;;; if simulated demo in bullet world:
(defun setup-bullet-logging ()
  ;;pipe tf from bullet to actual tf topic
  (setf cram-tf:*tf-broadcasting-topic* "tf")
  (setf cram-tf:*tf-broadcasting-enabled* T)
  
  ;;overwrite the default links to the owls and other dependencies
  (setf ccl::*environment-owl* "'package://iai_apartment/owl/iai-apartment.owl'")
  (setf ccl::*environment-owl-individual-name* "'http://knowrob.org/kb/iai-apartment.owl#apartment_root'")
  (setf ccl::*environment-urdf* "'package://iai_apartment/urdf/apartment.urdf'")
  (setf ccl::*environment-urdf-prefix* "'iai_apartment/'")
  (setf ccl::*agent-owl* "'package://knowrob/owl/robots/PR2.owl'")
  (setf ccl::*agent-owl-individual-name* "'http://knowrob.org/kb/PR2.owl#PR2_0'")
  (setf ccl::*agent-urdf* "'package://knowrob/urdf/pr2.urdf'")

  )

(defun pouring-neem ()
  (urdf-proj:with-simulated-robot
    (ccl::start-episode)
    (apartment-demo-merged :step 0)
    (ccl::stop-episode)))
  
