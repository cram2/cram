(in-package :su-demos)







;;@author Felix Krause
;;max-objects dictates how many times the main loop from the table to the shelf should be performed.
;;open-shelf dictates if the shelf door should be opened. open-shelf = 0 -> skip door, open-shelf > 0 -> open door.
;;skip-shelf-perception dictates if the contents of the shelf should be perceived. Useful for testing. skip-shelf-perception = T -> skip perception of shelf contents, skip-shelf-perception = NIL -> perceive shelf contents.
;;joint-angle: Dictates how far the door will be opened, Value range ~0.0 to ~1.2. A sensible value is 0.55. Positive value = open to the left, Negative value = open to the right.
;;collision-mode: Different options, most common is :allow-all or :avoid-all
(defun storing-groceries-demo (&key max-objects skip-open-shelf skip-shelf-perception joint-angle collision-mode)

  (let ((shelf "shelf:shelf:shelf_base_center")
        (table "left_table:table:table_front_edge_center")
        (handle-link "iai_kitchen/shelf:shelf:shelf_door_left:handle")
        (all-designator (desig:all object (type :everything))))



    ;;Init query for Knowledge.
    (with-knowledge-result ()
        `("init_storing_groceries")
      (print "Storing groceries plan started."))
  
    (park-robot)

   


    (cond ((equal skip-open-shelf NIL)

           ;;Move to the shelf to a perceive pose.
           (with-knowledge-result (result)
               `(and ("has_urdf_name" object ,shelf)
                     ("object_rel_pose" object "perceive" result))
             (move-hsr (make-pose-stamped-from-knowledge-result result)))
         
         (print "Performing sequence, door will be opened.")
         ;;Open the shelf.
         (let ((?handle-link handle-link)
               (?joint-angle joint-angle)
               (?collision-mode collision-mode))
    
           (exe:perform (desig:an action
                                  (type opening-door)
                                  (handle-link ?handle-link)
                                  (joint-angle ?joint-angle)
                                  (tip-link t)
                                  (collision-mode ?collision-mode))))
           (park-robot))
          ((equal skip-open-shelf T) 
           (print "Skipping sequence, shelf door wont be opened.")))
    
   
  (cond ((equal skip-shelf-perception NIL)
         ;;Perceive the contents of the shelf.
         ;;Saves all possible objects.
         ;;Objects are then created in Knowledge.

         
         ;;Move to the shelf.
         (with-knowledge-result (result)
             `(and ("has_urdf_name" object shelf)
                   ("object_rel_pose" object "perceive" result))
           (move-hsr (make-pose-stamped-from-knowledge-result result)))
         
         (let* ((?source-object-desig-shelf all-designator)
                (?object-desig-list-shelf
                  (exe:perform (desig:all action
                                          (type detecting)
                                          (object ?source-object-desig-shelf)))))

    
           (park-robot)
           
           (print ?object-desig-list-shelf)))
        ((equal skip-shelf-perception T)
         (print "Skipping sequence, shelf contents wont be perceived.")))


    ;;Move to the table to a perceive pose.
    ;;(move-to-table T table)
    (with-knowledge-result (result)
        `(and ("has_urdf_name" object ,table)
              ("object_rel_pose" object "perceive" result))
      (move-hsr (make-pose-stamped-from-knowledge-result result)))

    ;;(perc-robot)

    ;;Perceive the objects on the table. Put all objects into a list. 
    (let* ((?source-object-desig all-designator)
           (?list-of-objects
             (exe:perform (desig:all action
                                     (type detecting)
                                     (object ?source-object-desig)))))
        

;;=======================================MAIN=LOOP========================================================================
      (let* ((?place-poses (get-hardcoded-place-poses)))
  
        ;;Perform this loop max-objects amount of times.
        (dotimes (n max-objects)
          ;;Pick up the first object in the list.
          ;;TODO - Filter the objects somehow when current-object is picked that the optimal item is picked up.
          ;;TODO - Object size can be extracted from Perception.
          ;;TODO - Place pose can be extracted from Knowledge.
          ;;TODO - Its best if all properties of the current Designator are extracted here.
          ;;TODO - Extract: Object size, object height, place pose.
          ;;TODO - Next Object might not work like this, otherwise random order + extract object name.
          
            (let*  ((?collision-mode collision-mode)
                  ;;HARDCODED
                  (?object-size (cl-tf2::make-3d-vector 0.06 0.145 0.215));;(extract-size ?current-object))
                  (?object-height 0.23)
                  ;;DYNAMIC Elements
                  (?next-object (get-next-object-storing-groceries))
                  (?next-pick-up-pose (get-pick-up-pose ?next-object))
                  ;;(?next-place-pose (get-place-pose-in-shelf ?next-object))
                  ;;HARDCODED/OLD PLACE POSES
                  (?place-pose (pop ?place-poses))
                  (?current-object (pop ?list-of-objects))
                  (?current-object-pose (extract-pose ?current-object)))
            

            ;;Pick up the object.
            (exe:perform (desig:an action
                                   (type :picking-up)
                                   (object-pose ?next-pick-up-pose)
                                   (object-size ?object-size)
                                   (collision-mode ?collision-mode)))
           
           
            (park-robot)

            ;;Move to the shelf
            (with-knowledge-result (result)
                `(and ("has_urdf_name" object ,shelf)
                      ("object_rel_pose" object "perceive" result))
              (move-hsr (make-pose-stamped-from-knowledge-result result)))

            ;;Places the object currently held.
            (exe:perform (desig:an action
                                   (type :placing)
                                   (target-pose ?place-pose)
                                   (object-height ?object-height)
                                   (frontal-placing T)
                                   (neatly T)
                                   (collision-mode ?collision-mode)))

            
            (park-robot)

            ;;Move to the table to a perceive pose.         
            (with-knowledge-result (result)
                `(and ("has_urdf_name" object ,table)
                      ("object_rel_pose" object "perceive" result))
              (move-hsr (make-pose-stamped-from-knowledge-result result)))
            (print "Loop finished."))))

        (print "Demo finished."))))

;;@author Felix Krause
(defun extract-pose (object)
  (roslisp:with-fields 
      ((?pose
        (cram-designators::pose cram-designators:data))) 
      object    
    ?pose))

;;@author Felix Krause
(defun extract-type (object)
  (roslisp:with-fields 
      ((?type
        (cram-designators::object-identifier cram-designators:data))) 
      object    
     (intern (string-trim "-1" ?type) :keyword)))

;;@author Felix Krause
;;doesnt work for now
;; (defun extract-size (object)
;;   (roslisp:with-fields 
;;       ((?size
;;         (cram-designators::size cram-designators:description))) 
;;       object    
;;     ?size))


;;@author Felix Krause
(defun move-to-table (side table)
    (with-knowledge-result (result)
        `(and ("has_urdf_name" object ,table)
              ("object_rel_pose" object "perceive" result))
      (move-hsr (make-pose-stamped-from-knowledge-result result))))

;;@author Felix Krause
(defun move-to-shelf (side shelf)
  (with-knowledge-result (result)
      `(and ("has_urdf_name" object ,shelf)
            ("object_rel_pose" object "perceive" result))
    (move-hsr (make-pose-stamped-from-knowledge-result result))))

;;@author Felix Krause
(defun get-next-object-storing-groceries ()
  (with-knowledge-result (result)
      `("next_object" result)
    result))

;;@author Felix Krause
(defun get-place-pose-in-shelf (object)
    (with-knowledge-result (result)
      `("object_rel_pose" ,object "destination" (list) result)
      (make-pose-stamped-from-knowledge-result result)))

;;@author Felix Krause
(defun get-pick-up-pose (object)
  (with-knowledge-result (result)
      `("object_pose" ,object result)
    (make-pose-stamped-from-knowledge-result result)))


;;@author Felix Krause
(defun get-hardcoded-place-poses ()
  (list (cl-tf:make-pose-stamped "map" 0.0  (cl-tf:make-3d-vector 2.15 2.55 0.72) (cl-tf:make-quaternion 0 0 0 1))
          (cl-tf:make-pose-stamped "map" 0.0  (cl-tf:make-3d-vector 1.95 2.55 0.72) (cl-tf:make-quaternion 0 0 0 1))
    (cl-tf:make-pose-stamped "map" 0.0  (cl-tf:make-3d-vector 1.95 2.55 0.48) (cl-tf:make-quaternion 0 0 0 1))
    (cl-tf:make-pose-stamped "map" 0.0  (cl-tf:make-3d-vector 2.1 2.55 0.48) (cl-tf:make-quaternion 0 0 0 1))
    (cl-tf:make-pose-stamped "map" 0.0  (cl-tf:make-3d-vector 1.9 2.55 0.48) (cl-tf:make-quaternion 0 0 0 1))))

;;@author Felix Krause
(defun test-place ()

  (park-robot)
  
  (let* ((?collision-mode :allow-all)
         (?object-height 0.2)
         (?place-pose (first (get-hardcoded-place-poses))))
    (print ?place-pose)
  
    
    (exe:perform (desig:an action
                           (type :placing)
                           (target-pose ?place-pose)
                           (object-height ?object-height)
                           (frontal-placing T)
                           (collision-mode ?collision-mode)))))


(defun test-perception ()

  ;;(park-robot)


  ;; (with-knowledge-result (result)
  ;;     `(and ("has_urdf_name" object ,"left_table:table:table_front_edge_center")
  ;;           ("object_rel_pose" object "perceive" result))
  ;;   (move-hsr (make-pose-stamped-from-knowledge-result result)))


  
  (let* ((?source-object-desig
           (desig:all object
                     (type :everything)))
         (?object-desig
           (exe:perform (desig:all action
                                  (type detecting)
                                  (object ?source-object-desig)))))
    

      (roslisp:with-fields 
        ((?pose
          (cram-designators::pose cram-designators:data))) 
          ?object-desig

        (print ?pose)))

      ;; (let ((?object-size
      ;;         (cl-tf2::make-3d-vector 0.06 0.145 0.215)))
      ;;   (exe:perform (desig:an action
      ;;                          (type picking-up)
      ;;                          (object-pose ?pose)
      ;;                          (object-size ?object-size)
      ;;                          (collision-mode :allow-all))))))




  )


