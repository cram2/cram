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


(in-package :prolog)

(defclass production-node ()
  ((beta-node :reader production-beta-node :initarg :beta-node)
   (tokens :reader production-tokens :initarg :tokens :initform nil)
   (callback :reader production-callback :initarg :callback
             :documentation "The callback of the production. It is
                             called with each token passed by the beta
                             node and the corresponding operation.")))

(defun make-production-node (beta-node callback)
  (let ((new-node (make-instance 'production-node
                    :beta-node beta-node
                    :callback callback)))
    (pushnew new-node (slot-value beta-node 'connections))
    (mapc (rcurry callback :assert) (slot-value beta-node 'matches))
    new-node))

(defmethod propagate ((node production-node) token operation)
  (with-slots (tokens callback) node
    (case operation
      (:assert
       (push token tokens))
      (:retract
       (remove token tokens)))
    (funcall callback token operation)))

(defmethod gc-node ((node production-node))
  (setf (slot-value (production-beta-node node) 'connections)
        (remove node (slot-value (production-beta-node node) 'connections)))
  (gc-node (production-beta-node node)))

(defvar *productions* (make-hash-table :test 'eq))

(defun clear-productions ()
  "Clears the beta network. All productions will be lost."
  (setf *productions* (make-hash-table :test 'eq)))

(defstruct production-definition
  (name nil)
  (var-bindings nil)
  (body nil)
  (node nil)
  (callbacks nil))

(defun register-production (name definition)
  (when (gethash name *productions*)
    (remove-production name)
    (alexandria:simple-style-warning "Redefining production `~S'." name))
  (setf (gethash name *productions*)
        (make-production-definition :name name
                                    :var-bindings (vars-in definition)
                                    :body definition)))

(defun remove-production (name)
  "Removes a production."
  (let ((production (gethash name *productions*)))
    (when production
      (remhash name *productions*)
      (gc-node (production-definition-node production)))))

(defmacro def-production (name &body definition)
  "Defines a new production. `name' is the name of the production.
   All variables within `definition' are passed to callbacks."
  `(register-production ',name ',definition))

(defmacro with-productions (productions &body body)
  "Executes `body' with productions specified. `productions is of the form

  ((name definition)*)

  similar to def-production. Please note that productions have dynamic
  extent, i.e. redefining a production shadows previously defined
  productions with the same name."
  `(let ((*productions* (alexandria:copy-hash-table *productions*)))
     (unwind-protect
          (progn
            (handler-bind ((alexandria:simple-style-warning #'muffle-warning))
              ,@(loop for (name . definition) in productions
                   collecting `(register-production ',name ',definition)))
            ,@body)
       ,@(loop for (name . production) in productions
            collecting `(remove-production ',name)))))

(defun production-node-type (definition)
  "Returns the type of definition. It can (currently) be either :prolog or :beta-join.
   It is used to select the correct node type. (In the future, there
   should also be a :beta-not type:"
  (cond ((find-prolog-fact definition)
         :prolog)
        (t
         :beta-join)))

(defun create-production-network (production callback)
  (labels ((get-definition-variables (definition)
             "Returns the variables of a single definition, with a
             position designator."
             (loop
                for field in definition
                for i from 0
                when (is-var field) collecting (cons field i)))
           (get-local-variable-accessors (accessor-generator variables accessors)
             "Creates variable accessors for just one set of
              variables, i.e. only for one wme pattern.  Please note
              that the wme pattern is not necessary to create the
              accessors. Instead `acessor-generator' is used. It must
              be bound to a function that gets the variable index as
              input and returns a function that gets the caller-index
              and the token as input and returns the value of the
              variable and the index.
               
              `variables' is the alist of variables with corresponding
              indices in the pattern. `accessors' is the alist of
              variable accessors created so far."
             (cond ((not variables)
                    accessors)
                   (t
                    (destructuring-bind (var . var-index) (car variables)
                      (get-local-variable-accessors
                       accessor-generator
                       (cdr variables)
                       (if (assoc var accessors)
                           accessors
                           (cons
                            `(,var . ,(funcall accessor-generator var-index))
                            accessors)))))))
           (get-variable-accessors (production-definition &optional (index 0) (accessors nil))
             "Collects accessor functions to get the value of a
              variable out of a token. The accessor function gets two
              parameters, an index and the current token."
             (cond ((null production-definition)
                    accessors)
                   (t
                    (let ((variables (get-definition-variables (car production-definition))))
                      (get-variable-accessors
                       (cdr production-definition)
                       (1+ index)
                       (get-local-variable-accessors (lambda (var-index)
                                                       (lambda (caller-index token)
                                                         (elt (alpha-memory-node-pattern
                                                               (elt token (- caller-index index)))
                                                              var-index)))
                                                     variables accessors))))))
           (get-fact (definition)
             (loop for field in definition
                if (is-var field) collecting '?
                else collecting field))
           (add-beta-node (definition parent-beta-node callback)
             (case (production-node-type definition)
               (:prolog
                (make-prolog-node parent-beta-node callback))
               (:beta-join
                (let ((alpha-node (get-alpha-node (get-fact definition))))
                  (make-beta-join-node parent-beta-node alpha-node callback)))))
           (build-network (definitions accessors &optional (parent-beta-node nil) (index 0))
             "Builds up the network and returns the last beta node"
             (let ((beta-node (add-beta-node (car definitions) parent-beta-node
                                             (lambda (token)
                                               (loop
                                                  for field in (car definitions)
                                                  for value in (alpha-memory-node-pattern (token-wme token))
                                                  when (and (is-var field)
                                                            (not (eql value (funcall (cdr (assoc field accessors))
                                                                                     index token))))
                                                  return nil
                                                  finally (return t))))))
               (if (cdr definitions)
                   (build-network (cdr definitions) accessors beta-node (1+ index))
                   beta-node)))
           (call-with-keyword-vars (fun var-bdgs)
             "Calls `fun' with `vars' as keyword arguments."
             (apply fun (mapcan (lambda (var-bdg)
                                  (destructuring-bind (var value) var-bdg
                                    (let* ((var-param-name (intern (symbol-name var) (find-package :keyword))))
                                      `(,var-param-name ,value))))
                                var-bdgs))))
    (let* ((accessors (get-variable-accessors production))
           (last-beta-node (build-network production accessors)))
      (make-production-node
       last-beta-node
       (lambda (token operation)
         (let ((var-bdgs (loop for (var . accessor) in accessors 
                            collecting `(,var ,(funcall accessor (1- (length token)) token)))))
           (call-with-keyword-vars (curry callback operation) var-bdgs)))))))

(defun register-production-handler (production-name callback)
  "Registers a new production handler. A production handler is
   executed whenever a production holds or stops to hold, i.e. is
   asserted or retracted.

   Callback is a function with a lambda-list of the form

   (operation &key [VAR]*) 

   where VAR is a variable inside the production.  Please note that
   &allow-other-keys needs to be used when not all variables inside
   the production are specified."
  (let ((production (gethash production-name *productions*)))
    (unless production
      (error 'simple-error
             :format-control "Production `~a' unknown."
             :format-arguments (list production-name)))
    (pushnew callback (production-definition-callbacks production))
    (unless (production-definition-node production)
      (setf (production-definition-node production)
            (create-production-network (production-definition-body production)
                                       (lambda (&rest args)
                                         (loop for callback in (production-definition-callbacks production)
                                            do (etypecase callback
                                                 (function (apply callback args))
                                                 (symbol (apply (symbol-function callback) args))))))))
    (production-definition-node production)))

(defun remove-production-handler (production-name callback)
  (let ((production (gethash production-name *productions*)))
    (unless production
      (error 'simple-error
             :format-control "Production `~a' unknown."
             :format-arguments production-name))
    (setf (production-definition-callbacks production)
          (delete callback (production-definition-callbacks production)))))

(defmacro with-production-handlers (handler-definitions &body body)
  "With WITH-PRODUCTION-HANDLERS it is possible to define handlers for
   productions in the current lexial scope of
   `body'. `handler-definitions' is of the form

  ((<production-name> <lambda-list> &body body)*)

  The lambda-list is similar to REGISTER-PRODUCTION-HANDLER."
  (let ((handler-syms (loop for (name . rest) in handler-definitions
                         collecting `(,name . ,(gensym (format nil "~S-" name))))))
    `(flet ,(loop for (name lambda-list . body) in handler-definitions
               collecting `(,(cdr (assoc name handler-syms)) ,lambda-list
                             ,@body))
       (unwind-protect
            (progn
              ,@(loop for (name . fun) in handler-syms
                   collecting `(register-production-handler ',name #',fun))
              ,@body)
         ,@(loop for (name . fun) in handler-syms
              collecting `(remove-production-handler ',name #',fun))))))

;;; Productions are specifications of the form
;;; (<fact 1>
;;;  <fact 2>
;;;  <fact 3> ...)
;;;
;;; where a fact is an ordinary list of symbols. It can also contain
;;; variables which are symbols prefixed with a '?', e.g. ?A.
;;; Productions are defined with the macro def-production:
;;;
;;; (def-production 'stacked-objects
;;;   (on ?a ?b)
;;;   (on ?b ?c)
;;;
;;; When it is asserted that ?A is on ?B and ?B is on ?C, this
;;; production holds and all its callbacks are executed. To define a
;;; callback, the function REGISTER-PRODUCTION-HANDLER is used. The
;;; callback's lambda list has the form (OPERATION &KEY [VAR]*) with
;;; VAR being a variable in the production. An example of a handler
;;; for the stacked-objects production defined above might be the
;;; following:
;;;
;;; (defun assert-implicitly-stacked (op &key ?a ?b ?c)
;;;   (declare (ignore ?b))
;;;   (ecase op
;;;     (:assert (rete-assert `(on ,?a ,?c)))
;;;     (:retract (rete-retract `(on ,?a ,?c)))))
;;; (register-production-handler 'stacked-objects #'assert-implicitly-stacked)
;;;
;;; It should also be possible to define handlers in the current
;;; lexical scope with the macro 'WITH-PRODUCTION-HANDLER'. The
;;; example above would be:
;;; (with-production-handlers
;;;     ((stacked-objects (operation &key ?a ?c &ignore-other-keys)
;;;        (ecase operation
;;;         (:assert (rete-assert '(on ?a ?c)))
;;;         (:retract (rete-retract '(on ?a ?c)))))
;;;   ...)

