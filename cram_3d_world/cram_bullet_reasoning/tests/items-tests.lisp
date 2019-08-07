;;;
;;; Copyright (c) 2016, Thomas Lipps <tlipps@uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :btr-tests)


(define-test attach-object-bidirectional-and-attachment-type
  ;; Attaches two objects and check their attached objects and the attachment-type of these
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))

  ;; connect o1 and o2
  (btr:attach-object (btr:object btr:*current-bullet-world* 'o1)
                     (btr:object btr:*current-bullet-world* 'o2)
                     :attachment-type :test-type)
  ;; these are connected: o1 <-> o2
  (lisp-unit:assert-equal 'o2 (car (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1)))))
  (lisp-unit:assert-equal 'o1 (car (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2)))))
  ;; and the attachment-type fits too
  (lisp-unit:assert-equal :test-type
                          (btr::attachment-attachment
                           (first (car (cdr (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1))))))))
  (lisp-unit:assert-equal :test-type
                          (btr::attachment-attachment
                           (first (car (cdr (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2))))))))
  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2))

(define-test attach-object-bidirectional-more-than-two-objects
  ;; Attaches three objects and 'o1 is connected with both objects
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o3 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))

  ;;connect objects o2 <-> o1 <-> o3
  (btr:attach-object (btr:object btr:*current-bullet-world* 'o1)
                     (btr:object btr:*current-bullet-world* 'o2))
  (btr:attach-object (btr:object btr:*current-bullet-world* 'o1)
                     (btr:object btr:*current-bullet-world* 'o3))
  
  ;;these are connected o2 <-> o1 <-> o3
  (lisp-unit:assert-equal 'o2 (car (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1)))))
  (lisp-unit:assert-equal 'o3 (car (assoc 'o3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1)))))
  (lisp-unit:assert-equal 'o1 (car (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2)))))
  (lisp-unit:assert-equal 'o1 (car (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o3)))))
  ;;these are not connected to each other: o2 o3
  (lisp-unit:assert-false (equal 'o2 (car (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o3))))))
  (lisp-unit:assert-false (equal 'o3 (car (assoc 'o3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2))))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2)
  (btr:remove-object btr:*current-bullet-world* 'o3))

(define-test attach-object-more-objects-connected-bidirectional-to-one-object-in-one-call
  ;; Attaches three objects and 'o1 is connected with both objects in one call
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o3 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))

  ;;connect objects o2 <-> o1 <-> o3
  (btr:attach-object (list
                      (btr:object btr:*current-bullet-world* 'o2)
                      (btr:object btr:*current-bullet-world* 'o3))
                     (btr:object btr:*current-bullet-world* 'o1))
  
  ;;these are connected o2 <-> o1 <-> o3
  (lisp-unit:assert-equal 'o2 (car (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1)))))
  (lisp-unit:assert-equal 'o3 (car (assoc 'o3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1)))))
  (lisp-unit:assert-equal 'o1 (car (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2)))))
  (lisp-unit:assert-equal 'o1 (car (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o3)))))
  ;;these are not connected to each other: o2 o3
  (lisp-unit:assert-false (equal 'o2 (car (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o3))))))
  (lisp-unit:assert-false (equal 'o3 (car (assoc 'o3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2))))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2)
  (btr:remove-object btr:*current-bullet-world* 'o3))

(define-test attach-object-more-objects-connected-unidirectional-to-one-object-in-one-call
  ;; Attaches three objects and 'o1 is connected loose with both objects in one call
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o3 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))

  ;;connect objects o2 -> o1 <- o3
  (btr:attach-object (list
                      (btr:object btr:*current-bullet-world* 'o2)
                      (btr:object btr:*current-bullet-world* 'o3))
                     (btr:object btr:*current-bullet-world* 'o1)
                     :loose T)

  ;;object 'o2 is normal attached to 'o1, so not loose attached.
  (lisp-unit:assert-equal nil
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2))))))))
  ;;object 'o1 is loose attached to 'o2
  (lisp-unit:assert-equal T
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1))))))))
  ;;object 'o3 is normal attached to 'o1, so not loose attached.
  (lisp-unit:assert-equal nil
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o3))))))))
  ;;object 'o1 is loose attached to 'o3
  (lisp-unit:assert-equal T
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'o3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1))))))))
  ;;these are not connected to each other: o2 o3
  (lisp-unit:assert-false (equal 'o2 (car (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o3))))))
  (lisp-unit:assert-false (equal 'o3 (car (assoc 'o3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2))))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2)
  (btr:remove-object btr:*current-bullet-world* 'o3))

(define-test attach-object-unidirectional
  ;; Attaches two objects unidirectional and check if loose attachment was properly saved in the attachment
  (btr-utils:spawn-object 'o1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'o2 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))
  ;;attach object loose: o2 <- o1
  (btr:attach-object (btr:object btr:*current-bullet-world* 'o1)
                     (btr:object btr:*current-bullet-world* 'o2)
                     :loose t)

  ;;object 'o1 is normal attached to 'o2, so not loose attached.
  (lisp-unit:assert-equal nil
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1))))))))
  ;;object 'o2 is loose attached to 'o1
  (lisp-unit:assert-equal T
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2))))))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2))

(define-test attach-object-loose-attachments-are-removed-if-placed-whereelse
  ;; Lets image we have a mug, which we attach to a tray loose, so that if we move
  ;; the tray the mug moves along, but if we move the mug the tray stays still.
  ;; Now we wanna place the mug loose on another object and check if the loose connection
  ;; with the tray was removed.
  ;; To make things even more interesting we take the handle of the mug, which is attached
  ;; bidirectional with the mug and attach it loose to a hook. Therefore the loose attachment of
  ;; the mug and tray-1 should be removed and the hook and handle should be in an loose
  ;; connection.
  (btr-utils:spawn-object 'handle :mug :pose         ;;
                          '((-1 0.0 0.92)(0 0 0 1))) ;;  
  (btr-utils:spawn-object 'mug :mug :pose            ;;                                                  (X)
                          '((-1 0.0 0.92)(0 0 0 1))) ;;    mug <-> handle                         ---------------> mug <-> handle
  (btr-utils:spawn-object 'tray-1 :mug :pose         ;;     ^                     -->             |                          ^
                          '((-1 0.0 0.92)(0 0 0 1))) ;;     |              move handle to hook    |                          |
  (btr-utils:spawn-object 'hook :mug :pose           ;;   tray_1                                tray_1                      hook
                          '((-1 0.0 0.92)(0 0 0 1))) ;;
                                                     ;; WITH ATTACHING THE HANDLE TO THE HOOK THE MARKED LOOSE ATTACHMENT SHOULD BE REMOVED
  (btr:attach-object 'handle 'mug)
  (btr:attach-object 'tray-1 'mug :loose T) ;; tray-1 -> mug
  ;;object 'mug is normal attached to 'handle, so not loose attached.
  (lisp-unit:assert-equal nil
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'mug (btr:attached-objects (btr:object btr:*current-bullet-world* 'handle))))))))
  (lisp-unit:assert-equal nil
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'handle (btr:attached-objects (btr:object btr:*current-bullet-world* 'mug))))))))
  ;;object 'mug is loose attached to 'tray-1
  (lisp-unit:assert-equal T
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'tray-1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'mug))))))))
  (lisp-unit:assert-equal nil
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'mug (btr:attached-objects (btr:object btr:*current-bullet-world* 'tray-1))))))))

  ;;now we put the handle on the hook
  (btr:attach-object 'hook 'handle :loose T)

  ;;so the connection between 'mug and 'tray-1 is gone
  (lisp-unit:assert-false (car (assoc 'tray-1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'mug)))))
  (lisp-unit:assert-false (car (assoc 'mug (btr:attached-objects (btr:object btr:*current-bullet-world* 'tray-1)))))

  ;;handle is loose attached to 'hook
  (lisp-unit:assert-equal T
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'hook (btr:attached-objects (btr:object btr:*current-bullet-world* 'handle))))))))
  (lisp-unit:assert-equal nil
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'handle (btr:attached-objects (btr:object btr:*current-bullet-world* 'hook))))))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'mug)
  (btr:remove-object btr:*current-bullet-world* 'tray-1)
  (btr:remove-object btr:*current-bullet-world* 'handle)
  (btr:remove-object btr:*current-bullet-world* 'hook))

(define-test detach-object-simple-and-bidirectional-attachments
 ;; Detaches two bidirectional conntected objects
  (btr-utils:spawn-object 'oo1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))
    
  (btr:attach-object (btr:object btr:*current-bullet-world* 'oo1)
                     (btr:object btr:*current-bullet-world* 'oo2))
  ;; oo1 and oo2 are connected like this: oo1 <-> oo2
  (lisp-unit:assert-equal 'oo2 (car (assoc 'oo2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1)))))
  (lisp-unit:assert-equal 'oo1 (car (assoc 'oo1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  ;; detach oo1 and oo2 
  (btr:detach-object (btr:object btr:*current-bullet-world* 'oo1)
                     (btr:object btr:*current-bullet-world* 'oo2))
  (lisp-unit:assert-false (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1)))
  (lisp-unit:assert-false (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))
  
  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2))

(define-test detach-object-dont-remove-other-attachments-and-unidirectional
 ;; Detaches two objects where one object has one attached object
  (btr-utils:spawn-object 'oo1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo3 :mug :pose 
                          '((-1 0.75 0.92)(0 0 0 1)))
  
  ;; oo1, oo2 and oo3 are connected like this: oo3 <-> oo1 -> oo2
  (btr:attach-object (btr:object btr:*current-bullet-world* 'oo1)
                     (btr:object btr:*current-bullet-world* 'oo2)
                     :loose t)
  (btr:attach-object (btr:object btr:*current-bullet-world* 'oo1)
                     (btr:object btr:*current-bullet-world* 'oo3))

  ;;object 'oo1 is normal attached to 'oo2, so not loose attached.
  (lisp-unit:assert-equal nil
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'oo2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1))))))))
  ;;object 'oo2 is loose attached to 'oo1
  (lisp-unit:assert-equal T
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'oo1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2))))))))
  ;; detach oo1 and oo2 
  (btr:detach-object (btr:object btr:*current-bullet-world* 'oo1)
                     (btr:object btr:*current-bullet-world* 'oo2))
  ;; 'oo1 and 'oo2 should be attached anymore, but oo3 and oo1 still
  (lisp-unit:assert-false (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))
  (lisp-unit:assert-equal 'oo3 (car (assoc 'oo3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1)))))
  (lisp-unit:assert-number-equal 1 (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1))))
  (lisp-unit:assert-equal 'oo1 (car (assoc 'oo1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo3)))))
  (lisp-unit:assert-number-equal 1 (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo3))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2)
  (btr:remove-object btr:*current-bullet-world* 'oo3))  



(define-test detach-all-objects-star-connection
  ;; Detaches all attachments of one object x and check if the attachments of attached objects from x still exists
  (btr-utils:spawn-object 'oo1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo3 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo4 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo5 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  ;;                                                         x       x
  ;;attaching objects so these are like this connected: oo3 <-> oo1 <-> oo2 <-> oo5
  ;;                                                             ^
  ;;                                                             |x
  ;;                                                             v
  ;;                                                            oo4
  ;;
  ;; the marked attachments (x) should be removed after calling detach-all-objects with oo1
  (btr:attach-object 'oo1 'oo2)
  (btr:attach-object 'oo1 'oo3)
  (btr:attach-object 'oo1 'oo4)
  (btr:attach-object 'oo2 'oo5)
  (lisp-unit:assert-number-equal 3 (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1))))

  ;; Detaches 'oo1 with objects oo2, oo3 and oo4, but not oo5
  (btr:detach-all-objects (btr:object btr:*current-bullet-world* 'oo1))
  (lisp-unit:assert-number-equal 0 (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1))))
  (lisp-unit:assert-number-equal 1 (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2))))
  (lisp-unit:assert-equal 'oo5 (car (assoc 'oo5 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal 'oo2 (car (assoc 'oo2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo5)))))
  (lisp-unit:assert-number-equal 0 (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo3))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2)
  (btr:remove-object btr:*current-bullet-world* 'oo3)
  (btr:remove-object btr:*current-bullet-world* 'oo4)
  (btr:remove-object btr:*current-bullet-world* 'oo5))

(define-test get-loose-attached-objects-simple
  ;; Returns the loose attachments of an object
  (btr-utils:spawn-object 'oo1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo3 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo4 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo5 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))


  ;; Attaches objects to each other
  ;;          oo4
  ;;           ^
  ;;           |
  ;;           v
  ;; 'oo1 -> 'oo2 <-> 'oo3
  ;;      x    ^
  ;;           |x
  ;;          oo5
  ;;
  ;;The marked (x) attachments are loose/unidirectional attachments
  (btr:attach-object 'oo1 'oo2 :loose T)
  (btr:attach-object 'oo2 'oo3)
  (btr:attach-object 'oo2 'oo4)
  (btr:attach-object 'oo5 'oo2 :loose T :skip-removing-loose T) ;; the :skip-removing-loose key is used so the loose attachement between
                                                                ;; 'oo2 and 'oo1 won't get removed

  (let ((loose-attached-obj-names (btr::get-loose-attached-objects (btr:object btr:*current-bullet-world* 'oo2))))
    (lisp-unit:assert-equal 'oo5 (first loose-attached-obj-names))
    (lisp-unit:assert-equal 'oo1 (second loose-attached-obj-names)))

  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2)
  (btr:remove-object btr:*current-bullet-world* 'oo3)
  (btr:remove-object btr:*current-bullet-world* 'oo4)
  (btr:remove-object btr:*current-bullet-world* 'oo5))

(define-test remove-loose-attachment-for-simple
  ;; Removes the loose attachments of an object
  (btr-utils:spawn-object 'oo1 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo3 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo4 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))
  (btr-utils:spawn-object 'oo5 :mug :pose 
                          '((-1 0.0 0.92)(0 0 0 1)))


  ;; Attaches objects to each other
  ;;          oo4
  ;;           ^
  ;;           |
  ;;           v
  ;; 'oo1 -> 'oo2 <-> 'oo3
  ;;      x    ^
  ;;           |x
  ;;          oo5
  ;;
  ;;The marked (x) attachments are loose/unidirectional attachments
  (btr:attach-object 'oo1 'oo2 :loose T)
  (btr:attach-object 'oo2 'oo3)
  (btr:attach-object 'oo2 'oo4)
  (btr:attach-object 'oo5 'oo2 :loose T :skip-removing-loose T) ;; the :skip-removing-loose key is used so the loose attachement between
                                                                ;; 'oo2 and 'oo1 won't get removed

  (btr::remove-loose-attachment-for (btr:object btr:*current-bullet-world* 'oo2))
  (lisp-unit:assert-equal 'oo3 (car (assoc 'oo3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal 'oo4 (car (assoc 'oo4 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-number-equal 2 (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2))))

  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2)
  (btr:remove-object btr:*current-bullet-world* 'oo3)
  (btr:remove-object btr:*current-bullet-world* 'oo4)
  (btr:remove-object btr:*current-bullet-world* 'oo5))

(define-test setf-pose-item-with-attached-items
  ;; Attaches a new pose to an item with attached items,
  ;; so the pose of the attached items should change relative to the
  ;; item they are attached to too.
  (btr-utils:spawn-object 'oo1 :mug :pose 
                          '((-1 0.0 0.9)(0 0 0 1)))
  (btr-utils:spawn-object 'oo2 :mug :pose 
                          '((-1 0.0 0.8)(0 0 0 1)))
  (btr-utils:spawn-object 'oo3 :mug :pose 
                          '((-1 0.0 0.7)(0 0 0 1)))
  ;;Objects are attached: 'oo2 <-> 'oo1 <-> 'oo3
  ;; where z_oo2 = z_oo1 - 0.1
  ;; and   z_oo3 = z_oo1 - 0.2
  ;; are the differences to 'oo1 position
  (btr:attach-object 'oo1 'oo2)
  (btr:attach-object 'oo1 'oo3)

  ;;set new pose to 'oo1: move 'oo1 0.1m upwards
  (setf (btr:pose (btr:object btr:*current-bullet-world* 'oo1)) (cl-transforms:make-pose
                                                                 (cl-transforms:make-3d-vector -1 0.0 1)
                                                                 (cl-transforms:make-identity-rotation)))
  (sleep 0.1) ;; idk why, but else the position of any attachment are not set yet

  
  ;; new position of the item 'oo1 is set properly
  (lisp-unit:assert-number-equal -1.0d0 (cl-tf:x (cl-tf:origin (btr:pose (btr:object btr:*current-bullet-world* 'oo1)))))
  (lisp-unit:assert-number-equal 0.0d0 (cl-tf:y (cl-tf:origin (btr:pose (btr:object btr:*current-bullet-world* 'oo1)))))
  (lisp-unit:assert-number-equal 1.0d0 (cl-tf:z (cl-tf:origin (btr:pose (btr:object btr:*current-bullet-world* 'oo1)))))
  ;; Changed values of z in the position of the attached items:
  ;; z_oo2 + 0.1 = 0.9
  (lisp-unit:assert-number-equal 0.9d0 (cl-tf:z (cl-tf:origin (btr:pose (btr:object btr:*current-bullet-world* 'oo2)))))
  ;; z_oo3 + 0.1 = 0.8
  (lisp-unit:assert-number-equal 0.8d0 (cl-tf:z (cl-tf:origin (btr:pose (btr:object btr:*current-bullet-world* 'oo3)))))
  ;; Still the same values x,y in the position of the attached items:
  ;; x
  (lisp-unit:assert-number-equal -1.0d0 (cl-tf:x (cl-tf:origin (btr:pose (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-number-equal -1.0d0 (cl-tf:x (cl-tf:origin (btr:pose (btr:object btr:*current-bullet-world* 'oo3)))))
  ;; y
  (lisp-unit:assert-number-equal 0.0d0 (cl-tf:y (cl-tf:origin (btr:pose (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-number-equal 0.0d0 (cl-tf:y (cl-tf:origin (btr:pose (btr:object btr:*current-bullet-world* 'oo3)))))

  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2)
  (btr:remove-object btr:*current-bullet-world* 'oo3))


