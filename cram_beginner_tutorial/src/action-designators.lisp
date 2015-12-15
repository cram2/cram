(in-package :tut)

(defstruct turtle-shape
  "represents an object in continuous space matching a symbolic description"
  radius
  edges)
  
  (cram-prolog:def-fact-group shape-actions (action-desig)
  ;; for each kind of shape, call make-turtle-shape with the right number of edges

  ;; triangle
  (cram-prolog:<- (action-desig ?desig (shape ?action))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape triangle))
    (desig-prop ?desig (radius ?radius))
    (cram-prolog:lisp-fun make-turtle-shape :radius ?radius :edges 3  ?action))

  ;; square
  (cram-prolog:<- (action-desig ?desig (shape ?action))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape square))
    (desig-prop ?desig (radius ?radius))
    (cram-prolog:lisp-fun make-turtle-shape :radius ?radius :edges 4  ?action))

  ;; pentagon
  (cram-prolog:<- (action-desig ?desig (shape ?action))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape pentagon))
    (desig-prop ?desig (radius ?radius))
    (cram-prolog:lisp-fun make-turtle-shape :radius ?radius :edges 5  ?action))

  ;; hexagon
  (cram-prolog:<- (action-desig ?desig (shape ?action))
    (desig-prop ?desig (type shape))
    (desig-prop ?desig (shape hexagon))
    (desig-prop ?desig (radius ?radius))
    (cram-prolog:lisp-fun make-turtle-shape :radius ?radius :edges 6  ?action)))
