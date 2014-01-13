;; todo

(in-package :cram-plan-transformations)

;; STEP 0.5
(defun transform-into-alist (a-tree &optional an-alist)
  "Transforms a tree into an association list. `a-tree' is something like
   `((action-role flip-1      action-verb)
     (has-sense   flip-1      flip-1-8)
     (is-a        flip-1-8    flip.v.08)
     (action-role pancake-3   theme)
     (has-sense   pancake-3   pancake-3-1)
     (is-a        pancake-3-1 pancake.n.01))
   for which the result would be
   `((THEME . PANCAKE.N.01) (ACTION-VERB . FLIP.V.08))"
  (let ((tripple (car a-tree)))
   (if (null a-tree)
       an-alist
       (cond ((member 'action-role tripple)
              (transform-into-alist
               (cdr a-tree)
               (cons (cons (third tripple) (second tripple)) an-alist)))
             ((or (member 'has-sense tripple)
                  (member 'is-a tripple))
              (transform-into-alist
               (cdr a-tree)
               (mapcar #'(lambda (x)
                         (if (eql (cdr x) (second tripple))
                             (cons (car x) (third tripple))
                             x)) an-alist)))))))

;; STEP 1
(defun transform-into-action-definition (an-alist))