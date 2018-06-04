; Defines all the items (objects) which have been used within the Virtual Reality and the axis object,
; which are now added here, so that the same items can be used in the bullet world.
; All of these objetcs and their type are prefixed with 'ba-*' so that it can be differentiated between
; these objects and the objects otherwise present in the bullet world environment.
; The objects are NOT spawned at the 0 0 0 coordinate. They each have different spawning points.
(in-package :le)

; List of all the .stl files, of the newly added items. aka list of meshes. 
(defparameter *mesh-files*
  '((:ba-bowl "package://lisp_ease/resource/ba_bowl.stl" t)
    (:ba-cup "package://lisp_ease/resource/ba_cup.stl" t)
    (:ba-muesli "package://lisp_ease/resource/ba_muesli.stl" t)
    (:ba-fork "package://lisp_ease/resource/ba_fork.stl" t)
    (:ba-milk "package://lisp_ease/resource/ba_milk.stl" t)
    (:ba-axes "package://lisp_ease/resource/ba_axes.stl" t)))

; appends the newly defined objects in the *mesh-files* variable to the *mesh-files*
; of btr, so that they can be loaded
(defun append-meshes-to-list ()
  "Appends the newly defined objects in the *mesh-files* variable to the *mesh-files* of btr, so that they can be loaded in the bullet world."
     (btr:add-objects-to-mesh-list "lisp_ease"))



(defun add-bowl (&optional (?name 'ba-bowl))
  "Adds an object of type ba-bowl to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be 'ba-bowl.
RETURNS: Lazy list containing the created ba-bowl-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((0 1 3) (0 0 0 1))
                            :mass 0.2 :color (1 1 1) :mesh :ba-bowl)))))

(defun add-cup (&optional (?name 'ba-cup))
  "Adds an object of type ba-cup to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be 'ba-cup.
RETURNS: Lazy list containing the created ba-cup-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((0 2 3) (0 0 0 1))
                            :mass 0.2 :color (1 1 0) :mesh :ba-cup)))))

(defun add-muesli (&optional (?name 'ba-muesli))
  "Adds an object of type ba-muesli to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be 'ba-muesli.
RETURNS: Lazy list containing the created ba-muesli-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((0 3 3) (0 0 0 1))
                            :mass 0.2 :color (1 0 1) :mesh :ba-muesli)))))

(defun add-fork (&optional (?name 'ba-fork))
  "Adds an object of type ba-fork to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be 'ba-fork.
RETURNS: Lazy list containing the created ba-fork-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((0 4 3) (0 0 0 1))
                            :mass 0.2 :color (0 0 1) :mesh :ba-fork)))))

(defun add-milk (&optional (?name 'ba-milk))
  "Adds an object of type ba-milk to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be 'ba-milk.
RETURNS: Lazy list containing the created ba-milk-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((0 6 3) (0 0 0 1))
                            :mass 0.2 :color (1 0 0) :mesh :ba-milk)))))

(defun add-axes (&optional (?name 'ba-axes))
  "Adds an object of type ba-axes to the bullet world.
OPTIONAL NAME: Name of the object. If not set, the default name will be 'ba-axes.
RETURNS: Lazy list containing the created ba-axes-object."
  (prolog:prolog `(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ,?name ((2 2 1) (0 0 0 1))
                            :mass 0.2 :color (0 1 1) :mesh :ba-axes)))))
