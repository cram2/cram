cram_btr_spatial_relations_costmap
==================================

CRAM package with costmaps and keywords needed to describe spatial relations

_Costmap Keywords_  
-    _Position based_ : __:in, :on, :side__  
-    _Height based_ : __:level, :level-invert__
-    _Orientation based_ : __:orientation__  


Usages
------

__:level__  
  Refers to different shelves/levels inside a container. Supports 4 values: _:topmost_, _:bottommost_, _:middle_
  or _a number [from 1 to total number of levels inside the container]_. The numbering starts from bottom to top.  
  ``` lisp

(a location  
	(in  
		(an object  
			(type drawer)
			(urdf-name oven-area-area-right-drawer-main)
			(part-of kitchen)
			(level topmost)))) ;; Refers the topmost level inside the drawer
;; x denotes the height of the costmap inside the sample container represented
;;			|__x___|
;;			|______|
;;			|______|
;;			|______|
;;			|______|


(a location
	(in 
		(an object
			(type drawer)
			(urdf-name oven-area-area-right-drawer-main)
			(part-of kitchen)
			(level middle)))) ;; Refers the middle level inside the drawer
;; x denotes the height of the costmap inside the sample container represented
;;			|______|
;;			|______|
;;			|__x___|
;;			|______|
;;			|______|


(a location
	(in 
		(an object
			(type drawer)
			(urdf-name oven-area-area-right-drawer-main)
			(part-of kitchen)
			(level 2)))) ;; Refers the second level from the bottom inside the drawer
;; x denotes the height of the costmap inside the sample container represented
;;			|______|
;;			|______|
;;			|______|
;;			|___x__|
;;			|______|


  ``` 
__:level-invert__  
  Works exactly like __:level__, except the numbering of the levels inside the container is done from top to bottom. The behaviour
  only changes when referring levels with numbers, the other keywords behave and refer the same as __:level__
  ``` lisp
  (a location
	(in 
		(an object
			(type drawer)
			(urdf-name oven-area-area-right-drawer-main)
			(part-of kitchen)
			(level-invert 2)))) ;; Refers the second level from the top inside the drawer
;; x denotes the height of the costmap inside the sample container represented
;;			|______|
;;			|__x___|
;;			|______|
;;			|______|
;;			|______|
  ```

