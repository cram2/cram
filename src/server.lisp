;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :json-prolog)

(defun prolog-bdgs->json (bdgs)
  (format nil "{~{~{\"~a\": ~a~}~^,~}} "
          (mapcar (lambda (bdg)
                    (destructuring-bind (var . val)
                        bdg
                      (list (subseq (symbol-name var) 1)
                            (let ((strm (make-string-output-stream)))
                              (yason:encode (jsonify-exp val) strm)
                              (get-output-stream-string strm)))))
                  bdgs)))

(defun start-prolog-server (ns &key (package *package*))
  (let ((open-queries (make-hash-table :test 'equal)))
    (init-type-atoms)
    (flet ((query (request)
             (handler-case
                 (with-fields ((id id)
                               (query query))
                     request
                   (assert (not (gethash id open-queries)) ()
                           "Query with id `~a' is already open." id)
                   (setf (gethash id open-queries)
                         (cram-prolog:prolog
                          (json->prolog query)))
                   (make-response 'json_prolog_msgs-srv:PrologQuery
                                  :ok t
                                  :message ""))
               (error (e)
                 (make-response 'json_prolog_msgs-srv:PrologQuery
                                :ok nil
                                :message (format nil "Got error: ~a" e)))))
           (simple-query (request)
             (handler-case
                 (with-fields ((id id)
                               (query query))
                     request
                   (assert (not (gethash id open-queries)) ()
                           "Query with id `~a' is already open." id)
                   (setf (gethash id open-queries)
                         (cram-prolog:prolog
                          (let ((*read-eval* nil)
                                (*package* package))
                            (replace-complex-types (read-from-string query)))))
                   (make-response 'json_prolog_msgs-srv:PrologQuery
                                  :ok t
                                  :message ""))
               (error (e)
                 (make-response 'json_prolog_msgs-srv:PrologQuery
                                :ok nil
                                :message (format nil "Got error: ~a" e)))))
           (next-solution (request)
             (with-fields ((id id))
                 request
               (multiple-value-bind (bdgs found?)
                   (gethash id open-queries)
                 (cond ((not found?)
                        (make-response 'json_prolog_msgs-srv:PrologNextSolution
                                       :status (symbol-code 'json_prolog_msgs-srv:<prolognextsolution-response>
                                                           :wrong_id)))
                       ((not bdgs)
                        (prog1
                            (make-response 'json_prolog_msgs-srv:PrologNextSolution
                                           :status (symbol-code 'json_prolog_msgs-srv:<prolognextsolution-response>
                                                                :no_solution))
                          (remhash id open-queries)))
                       (t
                        (prog1
                            (make-response 'json_prolog_msgs-srv:PrologNextSolution
                                           :status (symbol-code 'json_prolog_msgs-srv:<prolognextsolution-response>
                                                               :ok)
                                           :solution (prolog-bdgs->json (lazy-car bdgs)))
                          (setf (gethash id open-queries)
                                (lazy-cdr bdgs))))))))
           (finish (request)
             (with-fields ((id id))
                 request
               (remhash id open-queries)
               (make-response 'json_prolog_msgs-srv:PrologFinish))))
      (register-service-fn (concatenate 'string ns "/query")
                           #'query 'json_prolog_msgs-srv:PrologQuery)
      (register-service-fn (concatenate 'string ns "/simple_query")
                           #'simple-query 'json_prolog_msgs-srv:PrologQuery)
      (register-service-fn (concatenate 'string ns "/next_solution")
                           #'next-solution 'json_prolog_msgs-srv:PrologNextSolution)
      (register-service-fn (concatenate 'string ns "/finish")
                           #'finish 'json_prolog_msgs-srv:PrologFinish))))
