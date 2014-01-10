;; NL Sentence: "Flip the pancake."

;;;; STEP 1: parse the NL sentence

;; Command to Daniel's system: parse NL sentence
;; Resulting string:
#|
actionRole(Flip-1, ActionVerb)
hasSense(Flip-1, Flip-1-8)
isA(Flip-1-8, flip.v.08)
actionRole(pancake-3, Theme)
hasSense(pancake-3, pancake-3-1)
isA(pancake-3-1, pancake.n.01)
|#

;; Lisp data structure for the above string
`((action-role flip-1      action-verb)
  (has-sense   flip-1      flip-1-8)
  (is-a        flip-1-8    flip.v.08)
  (action-role pancake-3   theme)
  (has-sense   pancake-3   pancake-3-1)
  (is-a        pancake-3-1 pancake.n.01))

;; action definition from the lisp data structure
`(an action
   (name some-nice-owl-connected-name)
   (action-verb flip.v.08)
   (theme (an object
            (name some-nice-owl-pancake)
            (type pancake.n.01))))

;; acion desig from the data structure
(cram-language-implementation:top-level (cram-designators:with-designators
     ((pancake-desig (cram-designator-properties:object `((name some-nice-owl-pancake)
                                                          (type pancake))))
      (flip-desig (cram-designator-properties:action `((name some-nice-owl-connected-name)
                                                       (action-verb flip)
                                                       (theme ,pancake-desig)))))))


;;;; STEP 2: ask probabilistic inference for missing information

;; Command to Daniel's system: what information's missing
;; Resulting string:
#|
actionRole(Skolem-Instrument, Instrument)
hasSense(Skolem-Instrument, spatula.n.01_senseID)
isA(spatula.n.01_senseID, spatula.n.01)
|#

;; List data structure:
`((action-role skolem-instrument    instrument)
  (has-sense   skolem-instrument    spatula.n.01_senseID)
  (is-a        spatula.n.01_senseID spatula.n.01))

;; action definition:
`(an action
   (name some-nice-owl-connected-name)
   (action-verb flip.v.08)
   (theme (an object
            (name some-nice-owl-pancake)
            (type pancake.n.01)))
   (instrument (an object
                 (name some-nice-owl-spatula)
                 (type spatula.n.01))))

;; action desig
(cram-language-implementation:top-level (cram-designators:with-designators
     ((pancake-desig (cram-designator-properties:object `((name some-nice-owl-pancake)
                                                          (type pancake))))
      (spatula-desig (cram-designator-properties:object `((name some-nice-owl-spatula)
                                                          (type spatula))))
      (flip-desig (cram-designator-properties:action `((name some-nice-owl-connected-name)
                                                       (action-verb flip)
                                                       (theme ,pancake-desig)
                                                       (instrument ,spatula-desig)))))))


;;;; STEP 3: ask knowrob for possible locations of objects using their names maybe
;; action description
`(an action
   (name some-nice-owl-connected-name)
   (action-verb flip.v.08)
   (theme (an object
            (name some-nice-owl-pancake)
            (type pancake.n.01)
            (location (a location
                        (name hoho-loc)
                        (on something)))))
   (instrument (an object
                 (name some-nice-owl-spatula)
                 (type spatula.n.01)
                 (location (a location
                             (name hohoho-loc)
                             (on something))))))

;; action desig
(cram-language-implementation:top-level (cram-designators:with-designators
     ((pancake-location-desig (cram-designator-properties:location `((name hoho-loc)
                                                                     (on something))))
      (spatula-location-desig (cram-designator-properties:location `((name hohoho-loc)
                                                                     (on something))))
      (pancake-desig (cram-designator-properties:object `((name some-nice-owl-pancake)
                                                          (type pancake)
                                                          (at ,pancake-location-desig))))
      (spatula-desig (cram-designator-properties:object `((name some-spatula)
                                                          (type spatula)
                                                          (at ,spatula-location-desig))))
      (flip-desig (cram-designator-properties:action `((name some-nice-owl-connected-name)
                                                       (action-verb flip)
                                                       (theme ,pancake-desig)
                                                       (instrument ,spatula-desig)))))))


;;;; STEP 4: convert it into a working plan

;; original Jan & Georg plan
(cram-language-implementation:top-level (cram-designators:with-designators
    ((on-table (location `((on Cupboard)
                           (name "kitchen_island"))))
     (oven (object `((type oven)
                     (at ,on-table))))
     (on-oven (location `((on ,oven))))
     (spatula-1 (object `((type spatula)
                          (at ,on-table))))
     (spatula-2 (object `((type spatula)
                          (at ,on-table))))
     (pancake (object `((type pancake)
                        (at ,on-oven)))))
  (cram-plan-knowledge:achieve `(object-flipped ,pancake ,spatula-1 ,spatula-2))))

;; simplified for now with 1 spatula
(cram-language-implementation:top-level (cram-designators:with-designators
    ((on-table (location `((on Cupboard)
                           (name "kitchen_island"))))
     (oven (object `((type oven)
                     (at ,on-table))))
     (on-oven (location `((on ,oven))))
     (spatula (object `((type spatula)
                        (at ,on-table))))
     (pancake (object `((type pancake)
                        (at ,on-oven)))))
  (cram-plan-knowledge:achieve `(object-flipped ,pancake ,spatula))))


