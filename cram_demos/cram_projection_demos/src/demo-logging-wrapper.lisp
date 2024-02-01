(in-package :demos)
"A wrapper which allows to generate a NEEM of a demo"

;;; if simulated demo in bullet world:
(defun setup-bullet-logging ()
  ;;pipe tf from bullet to actual tf topic
  (setf cram-tf:*tf-broadcasting-topic* "tf")
  (setf cram-tf:*tf-broadcasting-enabled* T))

(defun pouring-neem ()
  (urdf-proj:with-simulated-robot
    (ccl::start-episode)
    (apartment-demo-merged :step 0)
    (ccl::stop-episode)))
  
