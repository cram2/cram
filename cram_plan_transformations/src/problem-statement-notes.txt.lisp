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

;; Better data structure
`((action-verb flip.v.08) (theme pancake.n.01))

;; action definition from the lisp data structure
`(a goal
   (action-verb (an action
                  (name owl-name-for-flip.v.08)
                  (type flip)))
   (theme (an object
            (name owl-name-for-pancake.n.01)
            (type pancake))))
;; TODO define owl stuff for flip and pancake.n.01 (just hardcode)

;; use AN object and THE object in designators... o_O


;;;; STEP 2: ask probabilistic inference for missing information

;; Command to Daniel's system: what information's missing
;; Resulting string:
#|
actionRole(Skolem-Instrument, Instrument)
hasSense(Skolem-Instrument, spatula.n.01_senseID)
isA(spatula.n.01_senseID, spatula.n.01)
|#

;; Lisp data structure:
`((action-role skolem-instrument    instrument)
  (has-sense   skolem-instrument    spatula.n.01_senseID)
  (is-a        spatula.n.01_senseID spatula.n.01))

;; Plist
`((instrument spatula.n.01))

;; goal definition:
`(a goal
   (action-verb (an action
                  (name owl-name-for-flip.v.08)
                  (type flip)))
   (theme (an object
            (name owl-name-for-pancake.n.01)
            (type pancake)))
   (instrument (an object
                 (name owl-name-for-spatula.n.01)
                 (type spatula))))

;; ask Andrei: if it's pouring ask "i want a circular pancake" what's the height and tilt: a range not a number

;; if info in action definition is inconsistent, we need to choose one

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
;; (a location (or (table cupboards... )))

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
(cram-language-implementation:top-level
 (cram-designators:with-designators
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
(cram-language-implementation:top-level
 (cram-designators:with-designators
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

;; the flipping plan itself
(def-goal (achieve (object-flipped (?obj ?tool-1 ?tool-2 ?flipping-parameters)))
  "grasp inside flipping-params"
  (with-retry-counters ((find-object-retry-count 3))
    (achieve `(object-in-hand ,?tool-1))
    (achieve `(object-in-hand ,?tool-2))
    (with-failure-handling
        ((object-not-found (f)
           (declare (ignore f))
           (ros-warn
            (achieve plan-lib)
            "Failed to perceive the object to flip.")
           (do-retry find-object-retry-count
             (ros-warn (achieve plan-lib) "Retrying.")
             (retry))))
      (ros-info (achieve plan-lib) "Perceiving object")
      (setf ?obj (perceive-object 'a ?obj)))
    (with-designators
        ((reach-object-location (location `((to reach)
                                            (obj ,?obj)
                                            ;; TODO(winkler): Replace
                                            ;; these fixed side
                                            ;; prameters by the
                                            ;; prolog-resolved sides
                                            ;; in which ?tool-1 and
                                            ;; ?tool-2 are held.
                                            (sides (:left :right)))))
         (obj-look-location (location `((of ,?obj)))))
      (at-location (reach-object-location)
        ;; TODO(winkler): Add a variant of `looking-at' that accepts
        ;; an object-designator as a parameter
        (achieve `(looking-at ,(reference obj-look-location)))
        (let* ((parameterization (description ?flipping-parameters))
               (new-description `((to flip)
                                  (obj ,?obj)
                                  (tools (,?tool-1 ,?tool-2))))
               (appended-description
                 (append (loop for param in parameterization
                               when (not (find param new-description
                                               :test (lambda (x y)
                                                       (eql (car x)
                                                            (car y)))))
                                 collect param)
                         new-description)))
          (with-designators
              ((flip-action (action appended-description)))
            (perform flip-action)))))))