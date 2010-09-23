
(in-package :jlo)

(defvar *requested-jlo-objects* nil)

(defun empty-cache ()
  (setf *requested-jlo-objects* nil))

(defun jlo-register-gc (jlo)
  (let ((id (id jlo)))
    (assert (not (eql 0 id)) () "Cannot garbage collect jlo ids with id 0")
    (pushnew (cons id (trivial-garbage:make-weak-pointer jlo))
             *requested-jlo-objects*
             :key #'car :test #'eql)
    jlo))

(defun jlo-unregister-gc (id)
  (setf *requested-jlo-objects*
        (delete id *requested-jlo-objects* :key #'car)))

(defun jlo-gc ()
  (let ((removed-ids
         (loop for (id . val) in *requested-jlo-objects*
            unless (trivial-garbage:weak-pointer-value val)
            collecting (prog1 id
                         (unless (eql id 1)
                           ;; We are not allowed to del the world, so
                           ;; just avoid the call to save some
                           ;; resources.
                           (handler-case (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "del"
                                                       :query (make-instance 'vision_msgs-msg:<partial_lo>
                                                                :id id))
                             (error (e)
                               (declare (ignore e))
                               nil)))))))
    (setf *requested-jlo-objects* (delete-if (rcurry #'member removed-ids) *requested-jlo-objects* :key #'car))))
