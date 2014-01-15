;;; todo license

(in-package :cram-plan-transformations)

;;; STEP 0.5
(defun transform-into-plist (a-tree &optional (a-plist nil))
  "Transforms a tree into a property list. `a-tree' is something like
   `((action-role flip-1      action-verb)
     (has-sense   flip-1      flip-1-8)
     (is-a        flip-1-8    flip.v.08)
     (action-role pancake-3   theme)
     (has-sense   pancake-3   pancake-3-1)
     (is-a        pancake-3-1 pancake.n.01))
   for which the result would be
   `((THEME PANCAKE.N.01) (ACTION-VERB FLIP.V.08))"
  (let ((tripple (car a-tree)))
   (if (null a-tree)
       a-plist
       (cond ((member 'action-role tripple)
              (transform-into-plist
               (cdr a-tree)
               (cons (list (third tripple) (second tripple)) a-plist)))
             ((or (member 'has-sense tripple)
                  (member 'is-a tripple))
              (transform-into-plist
               (cdr a-tree)
               (mapcar #'(lambda (x) ; maybe it could stop at the first occurrence
                         (if (eql (car (cdr x)) (second tripple))
                             (list (car x) (third tripple))
                             x)) a-plist)))))))

;;; STEP 1
(defun transform-into-action-definition (a-plist)
  )