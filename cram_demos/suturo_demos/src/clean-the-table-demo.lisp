(in-package :su-demos)

;; @author Tim Alexander Rienits
;;
;; Perceives and picks up items from a table and places them inside a diswasher object in the urdf file.
;; Thereafter, it places the dishwasher tab inside as well.
;;
;; "max-objects" dictates the number of objects to be picked and placed into the dishwasher.
;;
(defun clean-the-table-demo (&key max-objects)

  (let ((table "left_table:table:table_front_edge_center")
        (dishwasher "shelf:shelf:shelf_base_center")        ;; TODO: Mit echtem Dishwasher ersetzen.
        ;;(dishwasher-handle "shelf:shelf:shelf_base_center") ;; TODO: Mit echtem Dishwasher handle  ersetzen.
        )

    ;; Puts the HSR into its default pose.
    (park-robot)

    ;; Move to table to perceive objects.
    (urdf-move-to table)

    ;; Puts the HSR into a pose which is suited to perceive objects.
    (perc-robot)

    ;; Perceives the objects on the table, saves them in a list.
    (let* ((?source-object-desig (desig:all object (type :everything)))
           
           (?list-of-objects
            (exe:perform (desig:all action
                                  (type detecting)
                                  (object ?source-object-desig)))))

      ;; TODO: Create Objects in Knowledge.


      ;; Pick and place objects n times, depending on max-objects.
      (dotimes (n max-objects)

        ;; Sets all necessary variables, using the list-of-objects we just perceived. Pick&Place first object in list until list ends.
        ;; TODO: Pick up optimal item based on current dishwasher.
        ;; TODO: Object size (and maybe height) can be extracted from Perception.
        ;; TODO: Place pose can be extracted from Knowledge.
        ;;
        (let* ((?current-object (pop ?list-of-objects)) ;;retrieve first element of perceived list and remove it after.
               (?current-object-pose (extract-pose ?current-object))
               (?object-size (cl-tf2::make-3d-vector 0.06 0.145 0.215)) ;;(extract-size ?current-object))
               (?object-height 0.22d0)
               (?place-poses (get-hardcoded-place-poses)) ;; Hardcoded poses inside the dishwasher.
               (?place-pose (pop ?place-poses)))


          ;; Pick up current object in perceived list.
          (su-pick-up ?current-object-pose ?object-size)

          (park-robot)

          (urdf-move-to dishwasher)

          ;; Place current object in perceived list into hardcoded pose inside the dishwasher.
          (su-place ?place-pose ?object-height)

          (park-robot)

          (urdf-move-to table)

          (print "I placed an object into the dishwasher. Will do the next one.")
            
          )))

    (print "I finished putting all the items inside the dishwasher. Will now put the dishwasher tab inside.")

   ;; ======================================= DISHWASHER TAB =======================================================================
    
    (perc-robot)
    
    ;; Perceive the dishwasher tab.
    (let* ((?source-object-desig (desig:an object (type :dishwashertab))) ;; TODO: type :tab
           
           (?dishwasher-tab
            (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig)))))
      
      ;; Same as above, but only for the tab instead.
      (let* ((?dishwasher-tab-pose (extract-pose ?dishwasher-tab))
             (?dishwasher-tab-size (cl-tf2::make-3d-vector 0.06 0.145 0.215)) ;; TODO: Insert Tab size.
             (?dishwasher-tab-height 0.22d0)                                  ;; TODO: Insert Tab Height.
             (?tab-place-pose (get-hardcoded-tab-pose)))



        (su-pick-up ?dishwasher-tab-pose ?dishwasher-tab-size)
        
        (park-robot)

        (urdf-move-to dishwasher)

        (su-place ?tab-place-pose ?dishwasher-tab-height)

        (print "I placed the tab into the dishwasher. Demo finished.")))
    
    ))

;;@author Tim Rienits
;; Tests the functionality of placing the dishwasher tab inside le dishwasher.
(defun test-tab ()

   (let ((table "left_table:table:table_front_edge_center")
        (dishwasher "shelf:shelf:shelf_base_center")        ;; TODO: Mit echtem Dishwasher ersetzen.
        ;;(dishwasher-handle "shelf:shelf:shelf_base_center") ;; TODO: Mit echtem Dishwasher handle ersetzen.
        )

    (urdf-move-to table)  
  
    (perc-robot)
    
    ;; Perceive the dishwasher tab.
    (let* ((?source-object-desig (desig:an object (type :dishwashertab))) ;; TODO: type :tab
           
           (?dishwasher-tab
            (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig)))))


      (print ?dishwasher-tab)

      (let* ((?dishwasher-tab-pose (extract-pose ?dishwasher-tab))
             (?dishwasher-tab-size (cl-tf2::make-3d-vector 0.06 0.145 0.215)) ;; TODO: Insert Tab size.
             (?dishwasher-tab-height 0.22d0)                                  ;; TODO: Insert Tab Height.
             (?tab-place-pose (get-hardcoded-tab-pose)))



        (su-pick-up ?dishwasher-tab-pose ?dishwasher-tab-size)
        
        (park-robot)

        (urdf-move-to dishwasher)

        (su-place ?tab-place-pose ?dishwasher-tab-height)

        (print "I placed the tab into the dishwasher. Demo finished.")))
     ))

;;@author Tim Rienits
;; Tests the functionality of placing the dishwasher tab inside le dishwasher.
(defun test-object ()

   (let ((table "left_table:table:table_front_edge_center")
        (dishwasher "imaginary_dishwasher:dishwasher_door")        ;; TODO: Mit echtem Dishwasher ersetzen.
        ;;(dishwasher-handle "shelf:shelf:shelf_base_center") ;; TODO: Mit echtem Dishwasher handle ersetzen.
        )

    ;;(urdf-move-to table)  
  
    ;;(perc-robot)
    
    ;; Perceive the dishwasher tab.
    (let* ((?source-object-desig (desig:an object (type :cerealbox))) ;; TODO: type :tab
           
           (?dishwasher-tab
            (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?source-object-desig)))))


      (print ?dishwasher-tab)

      (let* ((?dishwasher-tab-pose (extract-pose ?dishwasher-tab))
             (?dishwasher-tab-size (cl-tf2::make-3d-vector 0.06 0.145 0.215)) ;; TODO: Insert Tab size.
             (?dishwasher-tab-height 0.22d0)                                  ;; TODO: Insert Tab Height.
             (?tab-place-pose (get-hardcoded-tab-pose)))



        (su-pick-up ?dishwasher-tab-pose ?dishwasher-tab-size)
        
        (park-robot)

        ;;(urdf-move-to dishwasher)

        (move-hsr (cl-tf2::make-pose-stamped "map" 0 (cl-tf2::make-3d-vector -0.5d0 -1.0d0 0d0) (cl-tf2::make-quaternion 0.0 0.0 0.0 1.0)))
        
        (su-place ?tab-place-pose ?dishwasher-tab-height)

        (print "I placed the tab into the dishwasher. Demo finished.")))
 ))

;;@author Tim Rienits
;;
;; Uses an action Designator to pick up an object.
;;
;; ?pose - The pose of the object to be picked up.
;;
;; ?size - The size of the object to be picked up.
;;
(defun su-pick-up (?pose ?size)
   (exe:perform (desig:an action
                          (type picking-up)
                          (object-pose ?pose)
                          (object-size ?size)
                          (collision-mode :allow-all)))

  )

;;@author Tim Rienits
;;
;; Uses an action Designator to place an object.
;;
;; ?pose - The pose where the object should be placed.
;;
;; ?height - The height of the object to be placed.
;;
(defun su-place (?pose ?height)
    (exe:perform (desig:an action
                           (type :placing)
                           (target-pose ?pose)
                           (object-height ?height)
                           (collision-mode :allow-all)))

  )

;;@author Felix Krause
;; Extracts the pose from an Object Designator.
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

;;@author Tim Rienits
;; The hardcoded poses, where objects are to be placed into the dishwasher.
(defun get-hardcoded-place-poses ()
  (list (cl-tf:make-pose-stamped "map" 0.0  (cl-tf:make-3d-vector 0.15 1.73 0.725) (cl-tf:make-quaternion 0 0 0 1))
        (cl-tf:make-pose-stamped "map" 0.0  (cl-tf:make-3d-vector 0.15 1.73 0.47) (cl-tf:make-quaternion 0 0 0 1))
        (cl-tf:make-pose-stamped "map" 0.0  (cl-tf:make-3d-vector 0.15 1.73 0.11) (cl-tf:make-quaternion 0 0 0 1))))

;;@author Tim Rienits
;; The hardcoded pose where the dishwasher tab should be placed.
(defun get-hardcoded-tab-pose ()
  (cl-tf:make-pose-stamped "map" 0.0  (cl-tf:make-3d-vector -1.0 1.25 0.52) (cl-tf:make-quaternion 0 0 0 1)))


;;@author Tim Rienits
;; Moves the HSR to urdf-object by querying the pose of it in Knowledge.
(defun urdf-move-to (urdf-object)
  
 (with-knowledge-result (result)
        `(and ("has_urdf_name" object ,urdf-object)
              ("object_rel_pose" object "perceive" result))
      (move-hsr (make-pose-stamped-from-knowledge-result result)))
  )

;;@author Tim Rienits
;; Tests if object is cutlery (spoon, fork, knife) and instead of picking it up, it gets it handed instead.
(defun do-you-even-cutlery (object)
  
  (when (or (equal (extract-type object) :spoon)
            (equal (extract-type object) :fork)
            (equal (extract-type object) :knife))

    (wait-for-human-signal)

    )
  )

  
