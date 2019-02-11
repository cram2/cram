(in-package :kvr)

(defun demo-spawn-all-obj-in-place ()
  "Spawns all objects of the current Episode at their places without any offsets."
  (move-object-to-starting-pose 'muesli)
  (move-object-to-starting-pose 'milk)
  (move-object-to-starting-pose 'cup)
  (move-object-to-starting-pose 'bowl)
  (move-object-to-starting-pose 'fork))

(defun demo-spawn-objects-in-semantic ()
  "Spawns all objects of the current Episode on their original position in the
semantic map kitchen."
  (add-bowl :edeka-red-bowl2)
  (add-muesli :koelln-muesli-knusper-honig-nuss2)
  (add-fork :fork-blue-plastic2)
  (add-cup :cup-eco-orange2)
  (add-milk :weide-milch-small2)

  ;; the offset is the same offset as in the semantic map kitchen
  (move-obj-with-offset 0.0 -3.0 "IkeaBowl" :edeka-red-bowl2)
  (move-obj-with-offset 0.0 -3.0 "KoellnMuesliKnusperHonigNuss" :koelln-muesli-knusper-honig-nuss2)
  (move-obj-with-offset 0.0 -3.0 "PlasticBlueFork" :fork-blue-plastic2)
  (move-obj-with-offset 0.0 -3.0 "CupEcoOrange" :cup-eco-orange2)
  (move-obj-with-offset 0.0 -3.0 "MilramButtermilchErdbeere" :weide-milch-small2))
