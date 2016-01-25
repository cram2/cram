(in-package :tut)

(defstruct turtle-shape
  "represents an object in continuous space matching a symbolic description"
  radius
  edges)

(def-fact-group shape-actions (action-desig)
  ;; for each kind of shape, call MAKE-TURTLE-SHAPE with the right number of edges

  ;; triangle
  (<- (action-desig ?desig (draw-shape ?action))
    (desig-prop ?desig (:type :shape))
    (desig-prop ?desig (:shape :triangle))
    (desig-prop ?desig (:radius ?radius))
    (lisp-fun make-turtle-shape :radius ?radius :edges 3 ?action))

  ;; square
  (<- (action-desig ?desig (draw-shape ?action))
    (desig-prop ?desig (:type :shape))
    (desig-prop ?desig (:shape :square))
    (desig-prop ?desig (:radius ?radius))
    (lisp-fun make-turtle-shape :radius ?radius :edges 4 ?action))

  ;; pentagon
  (<- (action-desig ?desig (draw-shape ?action))
    (desig-prop ?desig (:type :shape))
    (desig-prop ?desig (:shape :pentagon))
    (desig-prop ?desig (:radius ?radius))
    (lisp-fun make-turtle-shape :radius ?radius :edges 5 ?action))

  ;; hexagon
  (<- (action-desig ?desig (draw-shape ?action))
    (desig-prop ?desig (:type :shape))
    (desig-prop ?desig (:shape :hexagon))
    (desig-prop ?desig (:radius ?radius))
    (lisp-fun make-turtle-shape :radius ?radius :edges 6 ?action)))


(def-fact-group goal-actions (action-desig)
  (<- (action-desig ?desig (go-to ?point))
    (desig-prop ?desig (:type :goal))
    (desig-prop ?desig (:goal ?point))))
