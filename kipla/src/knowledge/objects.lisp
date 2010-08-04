
(in-package :kipla-reasoning)

(def-fact-group object-classes ()
  (<- (obj-subtype ?a ?b)
    (obj-direct-subtype ?a ?b))
  (<- (obj-subtype ?a ?b)
    (obj-direct-subtype ?a ?tmp)
    (obj-subtype ?tmp ?b))
  (<- (obj-direct-subtype object cluster))
  (<- (obj-direct-subtype cluster ?x)))
