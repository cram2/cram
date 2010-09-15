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

(defvar *finish-marker* nil)
(defvar *service-namespace* "/json_prolog")

(defun make-query-id ()
  (symbol-name (gensym (format nil "QUERY-~20,10f-" (ros-time)))))

(defun prolog-result->bdgs (query-id result &key (lispify nil) (package *package*))
  (unless (json_prolog-srv:ok-val result)
    (error 'simple-error
           :format-control "Prolog query failed: ~a."
           :format-arguments (list (json_prolog-srv:message-val result))))
  (lazy-list ()
    (cond (*finish-marker*
           (call-service (concatenate 'string *service-namespace* "/finish")
                         'json_prolog-srv:PrologFinish
                         :id query-id)
           nil)
          (t
           (let ((next-value
                  (call-service (concatenate 'string *service-namespace* "/next_solution")
                                'json_prolog-srv:PrologNextSolution
                                :id query-id)))
             (ecase (car (rassoc (json_prolog-srv:status-val next-value)
                                 (symbol-codes 'json_prolog-srv:<prolognextsolution-response>)))
               (:no_solution nil)
               (:wrong_id (error 'simple-error
                                 :format-control "We seem to have lost our query. ID invalid."))
               (:query_failed (error 'simple-error
                                     :format-control "Prolog query failed: ~a"
                                     :format-arguments (list (json_prolog-srv:solution-val next-value))))
               (:ok (cont (json-bdgs->prolog-bdgs (json_prolog-srv:solution-val next-value)
                                                  :lispify lispify
                                                  :package package)))))))))

(defun prolog (exp &key (prologify t) (lispify nil) (package *package*))
  (let ((query-id (make-query-id)))
    (prolog-result->bdgs
     query-id
     (call-service (concatenate 'string *service-namespace* "/query")
                   'json_prolog-srv:PrologQuery
                   :id query-id
                   :query (prolog->json exp :prologify prologify))
     :lispify lispify :package package)))

(defun prolog-1 (exp &key (prologify t) (lispify nil) (package *package*))
  "Like PROLOG but closes the query after the first solution."
  (let ((bdgs (prolog exp :prologify prologify :lispify lispify :package package)))
    (finish-query bdgs)))

(defun prolog-simple (query-str &key (lispify nil) (package *package*))
  "Takes a prolog expression (real prolog, not the lispy version) and
evaluates it."
  (let ((query-id (make-query-id)))
    (prolog-result->bdgs
     query-id
     (call-service (concatenate 'string *service-namespace* "/simple_query")
                   'json_prolog-srv:PrologQuery
                   :id query-id
                   :query query-str)
     :lispify lispify :package package)))

(defun prolog-simple-1 (query-str &key (lispify nil) (package *package*))
  "Like PROLOG-SIMPLE but closes the query after the first solution."
  (let ((bdgs (prolog-simple query-str :lispify lispify :package package)))
    (finish-query bdgs)))
  
(defun finish-query (result)
  (let ((*finish-marker* t))
    (lazy-cdr (last result))
    result))

(defun wait-for-prolog-service (&optional timeout)
  (wait-for-service
   (concatenate 'string *service-namespace* "/query")
   timeout))
