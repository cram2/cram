(in-package :kvr)

;; object knowledge-------------------------------------------------------------
(cram-prolog:def-fact-group kvr-object-knowledge (object-rotationally-symmetric
                                             orientation-matters
                                             man-int::object-type-grasp)

  (prolog:<- (orientation-matters ?object-type)
    (member ?object-type (:fork-blue-plastic
                          :spoon-blue-plastic
                          :edeka-red-bowl
                          :koelln-muesli-knusper-honig-nuss)))

  (prolog:<- (object-type-grasp :fork-blue-plastic :human-grasp))
  (prolog:<- (object-type-grasp :fork-blue-plastic :top))

  (prolog:<- (object-type-grasp :spoon-blue-plastic :human-grasp))
  (prolog:<- (object-type-grasp :spoon-blue-plastic :top))

  (prolog:<- (object-type-grasp :edeka-red-bowl :human-grasp))
  (prolog:<- (object-type-grasp :edeka-red-bowl :top))

  (prolog:<- (object-type-grasp :cup-eco-orange :human-grasp))
  (prolog:<- (object-type-grasp :cup-eco-orange :back))
  (prolog:<- (object-type-grasp :cup-eco-orange :front))
  (prolog:<- (object-type-grasp :cup-eco-orange :left-side))
  (prolog:<- (object-type-grasp :cup-eco-orange :right-side))

  (prolog:<- (object-type-grasp :weide-milch-small :human-grasp))
  (prolog:<- (object-type-grasp :weide-milch-small :back))
  (prolog:<- (object-type-grasp :weide-milch-small :left-side))
  (prolog:<- (object-type-grasp :weide-milch-small :right-side))
  (prolog:<- (object-type-grasp :weide-milch-small :front))

  (prolog:<- (object-type-grasp :koelln-muesli-knusper-honig-nuss :human-grasp))
  (prolog:<- (object-type-grasp :koelln-muesli-knusper-honig-nuss :back))
  (prolog:<- (object-type-grasp :koelln-muesli-knusper-honig-nuss :front)))



