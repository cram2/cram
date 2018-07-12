;;; Defines all the items (objects) which have been used within the Virtual
;;; Reality and the axis object, which are now added here, so that the same
;;; items can be used in the bullet world.
;;; The objects are NOT spawned at the 0 0 0 coordinate.
;;; They each have different spawning points.

(in-package :kvr)
;; appends the newly defined objects in the *mesh-files* variable to the
;; *mesh-files* of btr, so that they can be loaded
;; NOTE (Gaya)  If this does not work or compile, add the second argument --
;;      directory where to search for cad models. In your case -- "resource".
;; TODO the function call can be added directly to initialization and does not
;; need an extra function wrapped around it.
;; Beware renaming though.
(defun append-meshes-to-list ()
  "Appends the newly defined objects in the *mesh-files* variable to the
*mesh-files* of btr, so that they can be loaded in the bullet world."
     (btr:add-objects-to-mesh-list "lisp_ease"))



(defun add-bowl (&optional (?name 'edeka-red-bowl))
  "Adds an object of type edeka-red-bowl to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be
'edeka-red-bowl.
RETURNS: Lazy list containing the created edeka-red-bowl-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((0 1 3) (0 0 0 1))
                            :mass 0.2 :color (1 1 1) :mesh :edeka-red-bowl)))))

(defun add-cup (&optional (?name 'cup-eco-orange))
  "Adds an object of type cup-eco-orange to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be
'cup-eco-orange.
RETURNS: Lazy list containing the created cup-eco-orange-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((0 2 3) (0 0 0 1))
                            :mass 0.2 :color (1 1 0) :mesh :cup-eco-orange)))))

(defun add-muesli (&optional (?name 'koelln-muesli-knusper-honig-nuss))
  "Adds an object of type koelln-muesli-knusper-honig-nuss to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be
'koelln-muesli-knusper-honig-nuss.
RETURNS: Lazy list containing the created koelln-muesli-knusper-honig-nuss-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((0 3 3) (0 0 0 1))
                            :mass 0.2 :color (1 0 1) :mesh :koelln-muesli-knusper-honig-nuss)))))

(defun add-fork (&optional (?name 'fork-blue-plastic))
  "Adds an object of type fork-blue-plastic to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be 'fork-blue-plastic.
RETURNS: Lazy list containing the created fork-blue-plastic-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((0 4 3) (0 0 0 1))
                            :mass 0.2 :color (0 0 1) :mesh :fork-blue-plastic)))))

(defun add-milk (&optional (?name 'weide-milch-small))
  "Adds an object of type weide-milch-small to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be
'weide-milch-small.
RETURNS: Lazy list containing the created weide-milch-small-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((0 6 3) (0 0 0 1))
                            :mass 0.2 :color (1 0 0) :mesh :weide-milch-small)))))

(defun add-axes (&optional (?name 'axes))
  "Adds an object of type axes to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be 'axes.
RETURNS: Lazy list containing the created axes-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((2 2 1) (0 0 0 1))
                            :mass 0.2 :color (0 1 1) :mesh :axes)))))
