(in-package :ccl)

(defclass cram-2-knowrob-mapper ()
  ((map :initform (make-hash-table :test 'equal))))

(defmethod add-definition-to-mapper (definition (mapper cram-2-knowrob-mapper))
  (loop for x in definition
        do (let ((cram-key (car x))
                 (knowrob-value (cadr x))
                 (map (slot-value mapper 'map)))
             (setf (gethash cram-key map) knowrob-value))))

(defmethod get-definition-from-mapper (definition (mapper cram-2-knowrob-mapper))
  (gethash definition (slot-value mapper 'map)))
