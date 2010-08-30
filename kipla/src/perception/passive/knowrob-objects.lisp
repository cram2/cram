
(in-package :kipla)

;;; The functionality in this file is pretty badly designed. The
;;; correct way to go would be to hook into the perception process
;;; module and implement something like context. For instance, if we
;;; need a location to see the object, it is fine to use queried
;;; objects, but if we actually want to grasp the object, we need to
;;; stand at a location where we can see the object (coming from the
;;; query) and execute cop. That means, the functionality in this file
;;; will be changed. GET-QUERIED-OBJECTS will return a list of
;;; queried-objects instances and the process module will care about
;;; calling the right function, depending on context.

(defvar *desig-owl-type-mapping* nil)

(defun knowrob-pre-initialize-desig (desig)
  (cond ((not (desig-prop-value desig 'owl-type))
         nil)
        (t
         (let ((new-desig (car (get-queried-objects desig))))
           (when new-desig
             (equate desig new-desig))))))

(defun get-queried-objects (desig)
  "Takes a designator and returns the list of all known matching
  objects as instances of QUERIED-OBJECT."
  ;; First, update the cache of objects.
  (json-prolog:prolog-1 '(objects-on-table ?t ?o))
  (let ((perceptions (crs:query-var '?objs #'json-prolog:prolog-1
                                    `(latest-perceptions-of-types
                                      ,(desig-type->owl-type (desig-prop-value desig 'owl-type))
                                      ?objs))))
    (mapcar (lambda (perception)
              (let* ((obj (crs:query-var '?obj #'json-prolog:prolog-1
                                         `(rdf-has ,perception
                                                   "'http://ias.cs.tum.edu/kb/knowrob.owl#objectActedOn'"
                                                   ?obj)))
                     (lo-id (crs:query-var '?loid #'json-prolog:prolog-1
                                           `(rdf-has ,perception "knowrob:loID"
                                                     (literal
                                                      (type
                                                       "'http://www.w3.org/2001/XMLSchema#int'"
                                                       ?loid)))))
                     (cop-id (crs:query-var '?loid #'json-prolog:prolog-1
                                           `(rdf-has ,obj "knowrob:copID"
                                                     (literal
                                                      (type
                                                       "'http://www.w3.org/2001/XMLSchema#int'"
                                                       ?loid)))))
                     (timestamp (crs:query-var '?ts #'json-prolog:prolog-1
                                               `(detection-starttime ,perception ?ts)))
                     (shape (crs:query-var '?shape #'json-prolog:prolog-1
                                           `(rdf-has
                                             ,obj
                                             "'http://ias.cs.tum.edu/kb/knowrob.owl#objectShapeType'"
                                             ?shape)))
                     (properties `((shape ,shape))))
                (perceived-object->designator
                 desig 
                 (make-instance 'queried-object
                                :pose (jlo:make-jlo :id lo-id)
                                :cop-id cop-id
                                :timestamp timestamp
                                :object-properties properties))))
            perceptions)))

(defun register-owl-type (owl-type)
  "Takes the owl type, splits it, registers the mapping between the
  type without namespace and the full url and returns the
  corresponding designator type field."
  (flet ((pl-sym->desig-type (sym)
           (check-type sym symbol)
           (intern (string-upcase
                    (remove #\'
                            (substitute #\- #\_  (symbol-name sym)))))))
    (let ((desig-type (pl-sym->desig-type
                       (crs:query-var '?loc #'json-prolog:prolog-1
                                      `(rdf-split-url ?ns ?loc ,owl-type)))))
      (pushnew (cons desig-type (etypecase owl-type
                                 (string owl-type)
                                 (symbol (symbol-name owl-type))))
               *desig-owl-type-mapping* :key #'car)
      desig-type)))

(defun desig-type->owl-type (desig-type)
  (cdr (assoc desig-type *desig-owl-type-mapping*)))

(defun owl-type->desig-type (type)
  (etypecase type
   (string (car (rassoc type *desig-owl-type-mapping* :test #'equal)))
   (symbol (owl-type->desig-type (symbol-name type)))))
