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
  ;; Attaches three objects and one is connection with both objects
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
  ;;these are not connected: o2 o3
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
  ;;attach object loosly: o2 <- o1
  (btr:attach-object (btr:object btr:*current-bullet-world* 'o1)
                     (btr:object btr:*current-bullet-world* 'o2)
                     :loose t)

  ;;object 'o1 is normal attached to 'o2, so not loosly attached.
  (lisp-unit:assert-equal nil
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'o2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o1))))))))
  ;;object 'o2 is loosly attached to 'o1
  (lisp-unit:assert-equal T
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'o1 (btr:attached-objects (btr:object btr:*current-bullet-world* 'o2))))))))

  ;; recreate begin state for next test case
  (btr:remove-object btr:*current-bullet-world* 'o1)
  (btr:remove-object btr:*current-bullet-world* 'o2))

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
 ;; Detaches two objects where one object is has one attached object
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

  ;;object 'oo1 is normal attached to 'oo2, so not loosly attached.
  (lisp-unit:assert-equal nil
                          (btr::attachment-loose
                           (first (car (cdr (assoc 'oo2 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo1))))))))
  ;;object 'oo2 is loosly attached to 'oo1
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
  (btr:attach-object 'oo5 'oo2 :loose T :skip-removing-loose T)

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
  (btr:attach-object 'oo5 'oo2 :loose T :skip-removing-loose T)

  (btr::remove-loose-attachment-for (btr:object btr:*current-bullet-world* 'oo2))
  (lisp-unit:assert-equal 'oo3 (car (assoc 'oo3 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-equal 'oo4 (car (assoc 'oo4 (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2)))))
  (lisp-unit:assert-number-equal 2 (list-length (btr:attached-objects (btr:object btr:*current-bullet-world* 'oo2))))

  (btr:remove-object btr:*current-bullet-world* 'oo1)
  (btr:remove-object btr:*current-bullet-world* 'oo2)
  (btr:remove-object btr:*current-bullet-world* 'oo3)
  (btr:remove-object btr:*current-bullet-world* 'oo4)
  (btr:remove-object btr:*current-bullet-world* 'oo5))



