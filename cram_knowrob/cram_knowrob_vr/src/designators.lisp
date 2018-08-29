(in-package :kvr)

;; object knowledge-------------------------------------------------------------
(prolog:def-fact-group kvr-object-knowledge (object-rotationally-symmetric
                                             orientation-matters
                                             obj-int:object-type-grasp)
  
  (prolog:<- (object-rotationally-symmetric ?object-type)
    (member ?object-type (:cup-eco-orange :weide-milch-small)))

  (prolog:<- (orientation-matters ?object-type)
    (member ?object-type (:fork-blue-plastic
                          :spoon-blue-plastic
                          :edeka-red-bowl
                          :koelln-muesli-knusper-honig-nuss)))
            
  (prolog:<- (obj-int:object-type-grasp :fork-blue-plastic :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :fork-blue-plastic :top))

  (prolog:<- (obj-int:object-type-grasp :spoon-blue-plastic :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :spoon-blue-plastic :top))
  
  (prolog:<- (obj-int:object-type-grasp :edeka-red-bowl :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :edeka-red-bowl :top))
  
  (prolog:<- (obj-int:object-type-grasp :cup-eco-orange :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :cup-eco-orange :back))
  (prolog:<- (obj-int:object-type-grasp :cup-eco-orange :front))
  (prolog:<- (obj-int:object-type-grasp :cup-eco-orange :left-side))
  (prolog:<- (obj-int:object-type-grasp :cup-eco-orange :right-side))

  (prolog:<- (obj-int:object-type-grasp :weide-milch-small :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :weide-milch-small :back))
  (prolog:<- (obj-int:object-type-grasp :weide-milch-small :left-side))
  (prolog:<- (obj-int:object-type-grasp :weide-milch-small :right-side))
  (prolog:<- (obj-int:object-type-grasp :weide-milch-small :front))
  
  (prolog:<- (obj-int:object-type-grasp :koelln-muesli-knusper-honig-nuss :human-grasp))
  (prolog:<- (obj-int:object-type-grasp :koelln-muesli-knusper-honig-nuss :back))
  (prolog:<- (obj-int:object-type-grasp :koelln-muesli-knusper-honig-nuss :front)))



