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
(defun get-owl-name (wordnet-name)
  (gensym "owl-bla"))

(defun transform-into-goal-description (a-plist)
  "`a-plist' is something like `((THEME PANCAKE.N.01) (ACTION-VERB FLIP.V.08)).
   The result is something like:
   (A GOAL
    ((THEME (A OBJECT (NAME #:|owl-bla1268|) (TYPE PANCAKE)))
     (ACTION-VERB (A ACTION (NAME #:|owl-bla1269|) (TYPE FLIP)))))"
  (flet ((extract-the-actual-name (name-with-id-stuff)
           "e.g. flip.v.08 -> flip"
           (let* ((a-string (symbol-name name-with-id-stuff))
                  (index (position #\. a-string :from-end t :end (position #\. a-string :from-end t)))
                  (a-substring (subseq a-string 0 index)))
             (intern a-substring))))
    (let ((wordnet-action-role-to-class '(action-verb action theme object instrument object)))
      `(a goal ,(mapcar #'(lambda (x)
                            `(,(car x)
                              (a ,(getf wordnet-action-role-to-class (car x))
                                (name ,(get-owl-name (car (cdr x))))
                                (type ,(extract-the-actual-name (car (cdr x)))))))
                        a-plist)))))

