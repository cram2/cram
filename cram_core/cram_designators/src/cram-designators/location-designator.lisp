;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :desig)

;;; Location designator resolution is based on two concepts:
;;; 
;;;  * Generation of solutions: Generator functions are functions that
;;;    take one parameter, the designator to be resolved and return a
;;;    possibly infinite (lazy) list of possible
;;;    solutions. Generators are ordered wrt. a priority value and
;;;    their solutions are appended to form the lazy list of
;;;    solutions. Lower priority means that the corresponding
;;;    generator is evaluated before generators with higher
;;;    priority. Generators are defined with the macro
;;;    DEF-LOCATION-GENERATOR.
;;;  
;;;  * Validation of solutions: After generation, a solution is
;;;    verified by a sequence of validation functions. Validation
;;;    functions are functions that get two parameters, the designator
;;;    and the generated solution.
;;;
;;;    Validation functions must return one of the following values:
;;;
;;;      - :ACCEPT the solution will be accepted unless another
;;;         validation function rejects it.
;;;
;;;      - :MAYBE-REJECT the solution will be rejected unless at least
;;;         validation function accepts it.
;;;
;;;      - :UNKNOWN the decision is completely left to the other
;;;         validation functions.
;;;
;;;      - anything else will cause the solution to be rejected.
;;;       
;;;    If the solution is accepted, it is used as the designator
;;;    reference. Otherwise, a new solution is taken from the
;;;    generated sequence of solutions and the validation functions
;;;    are executed again. The variable
;;;    *LOCATION-GENERATOR-MAX-RETRIES* indicates how often this
;;;    process can be repeated without finding a solution before an
;;;    error is signaled. Validation functions are declared with the
;;;    macro DEF-LOCATION-VALIDATION-FUNCTION
;;; 

(defstruct location-resolution-function
  function
  priority
  documentation)

(defvar *location-generators* nil
  "The list of instances of type LOCATION-RESOLUTION-FUNCTION that are
  used to generate solutions.")

(defvar *location-validation-functions* nil
  "The list of instances of type LOCATION-RESOLUTION-FUNCTION that are
  used to verify solutions.")

(defvar *disabled-location-generators* nil
  "List of location generators that should not be used although registered.")

(defvar *disabled-validation-functions* nil
  "List of validation function names that should not be used although
  registered.")

(defparameter *location-generator-max-retries* 30)

(defparameter *print-location-validation-function-results* nil
  "Enable this to get a text output by each rejected location designator solution.")

(defun register-location-resolution-function (place priority function &optional documentation)
  "Internal function used by the macro REGISTER-LOCATION-GENERATOR."
  `(eval-when (:compile-toplevel :load-toplevel :execute)
     (setf ,place
           (cons (make-location-resolution-function
                  :function ',function
                  :priority ,priority
                  :documentation ,documentation)
                 (remove ',function ,place :key #'location-resolution-function-function)))))

(defmacro register-location-generator (priority function &optional documentation)
  "Registers a location generator function. `priority' is a fixnum
used to order all location generators. Solutions generated with
functions with smaller priorities are used first. `function' is a
symbol naming the function that generates a list of solutions and that
takes exactly one argument, the designator. `documentation' is an
optional doc string."
  (declare (type fixnum priority)
           (type symbol function)
           (type (or null string) documentation))
  ;; (register-location-resolution-function
  ;;  '*location-generators* priority function documentation)
  `(setf *location-generators*
         (cons (make-location-resolution-function
                :function ',function
                :priority ,priority
                :documentation ,documentation)
               (remove ',function *location-generators*
                       :key #'location-resolution-function-function))))

(defmacro register-location-validation-function (priority function &optional documentation)
  "Registers a location validation function. `priority' is a fixnum
that indicates the evaluation order of all validation
functions. `function' is a symbol naming a function that takes exactly
two arguments, the designator and a solution and returns
either :ACCEPT, :REJECT, :MAYBE-REJECT or :UNKNOWN."
  (declare (type fixnum priority)
           (type symbol function)
           (type (or null string) documentation))
  ;; (register-location-resolution-function
  ;;  '*location-validation-functions* priority function documentation)
  `(setf *location-validation-functions*
         (cons (make-location-resolution-function
                :function ',function
                :priority ,priority
                :documentation ,documentation)
               (remove ',function *location-validation-functions*
                       :key #'location-resolution-function-function))))

(defun location-resolution-function-list (functions)
  (mapcar (compose #'symbol-function #'location-resolution-function-function)
          (sort (map 'list #'identity functions) #'<
                :key #'location-resolution-function-priority)))

(defun list-location-generators ()
  (location-resolution-function-list *location-generators*))

(defun list-location-validation-functions ()
  (location-resolution-function-list *location-validation-functions*))

(defclass location-designator (designator designator-id-mixin equate-notification-mixin)
  ((current-solution :reader current-solution :initform nil)))

(register-designator-class :location location-designator)

(defmethod reference ((desig location-designator) &optional (role *default-role*))
  (with-slots (data current-solution) desig
    (unless current-solution
      (setf data (resolve-designator desig role))
      (unless (and data (lazy-car data))
        (error 'designator-error
               :format-control "Unable to resolve designator `~a'"
               :format-arguments (list desig)
               :designator desig))
      (setf current-solution (lazy-car data)))
    current-solution))

(defmethod next-solution ((desig location-designator))
  ;; Make sure that we initialized the designator properly
  (unless (slot-value desig 'current-solution)
    (reference desig))
  (with-slots (data) desig
    (or (successor desig)
        (let ((new-desig (make-designator :location (description desig))))
          (when (lazy-cdr data)
            (setf (slot-value new-desig 'data) (lazy-cdr data))
            (setf (slot-value new-desig 'current-solution) (lazy-car (lazy-cdr data)))
            (equate desig new-desig))))))

(defmethod resolve-designator ((desig location-designator) (role t))
  (let* ((generators (location-resolution-function-list
                      (remove-if (lambda (generator)
                                   (member generator *disabled-location-generators*))
                                 *location-generators*
                                 :key #'location-resolution-function-function)))
         (solutions (lazy-mapcan (lambda (fun)
                                   (funcall fun desig))
                                 generators)))
    (lazy-mapcan (let ((retries *location-generator-max-retries*))
                   (lambda (solution)
                     (cond ((validate-location-designator-solution desig solution)
                            (setf retries *location-generator-max-retries*)
                            (list solution))
                           ((= retries 0)
                            ;; We are inside a lazy-list and if we
                            ;; reached the maximal retry count, let's
                            ;; stop iterating over possible solutions.
                            (invoke-restart :finish))
                           (t (decf retries) nil))))
                 solutions)))

(defun validate-location-designator-solution (designator solution)
  (declare (type location-designator designator))
  (labels ((validate (validation-functions designator solution
                      &optional (result :unknown))
             (cond
               ((not validation-functions)
                (ecase result
                  (:accept t)
                  (:maybe-reject nil)
                  (:unknown t)))
               (t
                (let ((validation-result
                        (funcall (car validation-functions) designator solution)))
                  (when *print-location-validation-function-results*
                    (format t "validator ~a says: ~a~%"
                            (car validation-functions) validation-result))
                  (case validation-result
                    (:accept
                     (validate
                      (cdr validation-functions) designator solution
                      :accept))
                    (:unknown
                     (validate
                      (cdr validation-functions) designator solution
                      result))
                    (:maybe-reject
                     (validate
                      (cdr validation-functions) designator solution
                      (if (eq result :accept) :accept :maybe-reject)))))))))
    (let ((validation-functions
            (location-resolution-function-list
             (remove-if (lambda (validation-function)
                          (member validation-function *disabled-validation-functions*))
                        *location-validation-functions*
                        :key #'location-resolution-function-function))))
      (block nil
        (restart-case
            (validate validation-functions designator solution)
          (accept-solution ()
            :report "Accept this designator solution"
            (return t))
          (reject-solution ()
            :report "Refuse this designator solution"
            (return nil)))))))

(defun delete-location-generator-function (function-name)
  "Delete a generator-function from the list of registered generator functions.
  `function-name' needs to be a symbol representing the function to delete."
  (setf *location-generators*
        (remove function-name *location-generators*
                :key #'location-resolution-function-function))
  (setf *disabled-location-generators*
        (remove function-name *disabled-location-generators*)))

(defun delete-location-validation-function (function-name)
  "Delete a validation-function from the list of registered validation functions.
   `function-name' needs to be a symbol representing the function to delete."
  (setf *location-validation-functions*
        (remove function-name *location-validation-functions*
                :key #'location-resolution-function-function))
  (setf *disabled-validation-functions*
        (remove function-name *disabled-validation-functions*)))

(defun enable-location-generator-function (function-name)
  (setf *disabled-location-generators*
        (remove function-name *disabled-location-generators*)))

(defun disable-location-generator-function (function-name)
  (pushnew function-name *disabled-location-generators*))

(defun enable-location-validation-function (function-name)
  (setf *disabled-validation-functions*
        (remove function-name *disabled-validation-functions*)))

(defun disable-location-validation-function (function-name)
  (pushnew function-name *disabled-validation-functions*))



(def-fact-group location-designator-generators (location-grounding)
  (<- (location-grounding ?desig ?solution)
    (assert-type ?desig location-designator "default location-grounding")
    (fail))

  ;; (<- (location-grounding ?desig ?solution)
  ;;   (desig-prop ?desig (:of ?obj))
  ;;   (obj-desig? ?obj)
  ;;   (lisp-fun current-desig ?obj ?curr-obj)
  ;;   (desig-prop ?curr-obj (:at ?loc))
  ;;   (designator-groundings ?loc ?locations)
  ;;   (member ?solution ?locations))
  )

(defun location-prolog-generator (desig)
  (lazy-mapcar (curry #'var-value '?solution)
               (prolog `(location-grounding ,desig ?solution))))

(defun location-prolog-validator (designator solution)
  (let ((solutions (location-prolog-generator designator)))
    (cond (solutions
           (lazy-dolist (candidate solutions :maybe-reject)
             (when (designator-solutions-equal candidate solution)
               (return :accept))))
          (t :unknown))))

(register-location-generator
 10 location-prolog-generator
 "Returns all solutions generated by the predicate (location-grounding ?desig ?solution)")

(register-location-validation-function
 10 location-prolog-validator
 "Makes sure that solutions generated by LOCATION-GROUNDING predicate
reject all invalid solutions.")




