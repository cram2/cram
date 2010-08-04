
(in-package :kipla-reasoning)

(liswip:def-swi-predicate desig-prop (desig prop-name prop-value)
  (prolog `(desig-prop ,desig (,prop-name ,prop-value))))

(liswip:def-swi-predicate desig-prop (desig prop-name prop-value timestamp)
  (prolog `(desig-prop ,desig (,prop-name ,prop-value) ,timestamp)))

(liswip:def-swi-predicate desig (desig)
  (prolog `(desig ,desig)))

(liswip:def-swi-predicate desig-class (desig type)
  (prolog `(desig-class ,desig ,type)))

(liswip:def-swi-predicate desig-value (desig value)
  (prolog `(desig-value ,desig ,value)))

(liswip:def-swi-predicate desig-value (desig value timestamp)
  (prolog `(desig-value ,desig ,value ,timestamp)))
