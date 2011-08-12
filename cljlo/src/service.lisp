
(in-package :jlo)

(define-condition jlo-query-error (simple-error) ())

(defun query-jlo (&key id name)
  (prog1
      (let ((result (cond ((and id name) (error "Either name or id must be passed."))
                          (id (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "idquery"
                                            :query (make-instance 'vision_msgs-msg:<partial_lo> :id id)))
                          (name (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "namequery"
                                              :query (make-instance 'vision_msgs-msg:<partial_lo> :name name))))))
        (unless (equal (vision_srvs-srv:error result) "")
          (error 'jlo-query-error
                 :format-control "jlo call for `~a' failed. ~a"
                 :format-arguments (list id (vision_srvs-srv:error result))))
        (vision_srvs-srv:answer result))
    (jlo-gc)))

(defun frame-query-ids (parent-id id)
  (prog1
      (cond ((eql id parent-id)
             nil)
            (t
             (let* ((result (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "framequery"
                                          :query (make-instance 'vision_msgs-msg:<partial_lo>
                                                   :id id :parent_id parent-id))))
                     (unless (equal (vision_srvs-srv:error result) "")
                 (error 'jlo-query-error
                        :format-control "jlo call for `~a' in `~a' failed. ~a"
                        :format-arguments (list id parent-id (vision_srvs-srv:error result))))
               (vision_srvs-srv:answer result))))
    (jlo-gc)))

(defun update-jlo (&key id (name "") (parent-id 1) partial-lo)
  (assert (or partial-lo id name) () "Either id, name or partial-lo must be passed.")
  (let ((result (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "update"
                              :query (or partial-lo
                                         (make-instance 'vision_msgs-msg:<partial_lo>
                                           :id id
                                           :name name
                                           :parent_id parent-id
                                           :pose (make-array 16 :initial-contents '(1 0 0 0
                                                                                    0 1 0 0
                                                                                    0 0 1 0
                                                                                    0 0 0 1)))))))
    (unless (equal (vision_srvs-srv:error result) "")
      (error 'jlo-query-error
             :format-control "jlo call failed. ~a"
             :format-arguments (list (vision_srvs-srv:error result))))
    (prog1 (vision_srvs-srv:answer result)
      (jlo-gc))))

(defun delete-id (id)
  (assert (not (eql id 1)) () "Cannot delete the world!")
  (let ((result (call-service "/located_object" 'vision_srvs-srv:srvjlo :command "del"
                              :query (make-instance 'vision_msgs-msg:<partial_lo>
                                       :id id))))
    (unless (equal (vision_srvs-srv:error result) "")
      (error 'jlo-query-error
             :format-control "jlo call failed. ~a"
             :format-arguments (list (vision_srvs-srv:error result)))))
  nil)
