(in-package :reas-inf)

(defvar *transform-listener*)

(defun set-transform-listener ()
  "Sets the transform listener to a new instance."
  (out-info "Setting *transform-listener*")
  (defparameter *transform-listener* (make-instance 'cl-tf2:buffer-client)))

(defun get-elapsed-time ()
  (out-info"Elapsed time: ~a ms" (- (get-internal-real-time) *start-time*)))

(defun color-msg-to-list (msg)
  "Returns a list with the RGB values from a given message `msg' (`std_msgs/ColorRGBA')"
  (with-fields (r g b) msg
    `(,r ,g ,b)))

(defun make-color-msg (r g b)
  "Returns a `std_msgs/ColorRGBA' message with the given RGB values `r', `g', `b'."
  (make-msg "std_msgs/ColorRGBA" :r r :g g :b b))

(defun make-pose-stamped-msg (frame position-x position-y position-z
                              &optional (rotation-x 0) (rotation-y 0) (rotation-z 0) (rotation-w 1))
  "Returns a `geometry_msgs/PoseStamped' message with the given values `position-x', `position-y', `position-z',
`rotation-x', `rotation-y', `rotation-z', `rotation-w'."
  (make-msg "geometry_msgs/PoseStamped"
            (:frame_id :header) frame
            (:x :position :pose) position-x
            (:y :position :pose) position-y
            (:z :position :pose) position-z
            (:x :orientation :pose) rotation-x
            (:y :orientation :pose) rotation-y
            (:z :orientation :pose) rotation-z
            (:w :orientation :pose) rotation-w))
            
(defun pose-stamped->pose-stamped-fixed (pose-stamped)
  "Transforms the stamped pose `pose-stamped' to `designators-ros:*fixed-frame*' and returns it."
  (let ((frame_id (cl-tf-datatypes:frame-id pose-stamped)))
    (if (string= (cl-tf2:unslash-frame frame_id) (cl-tf2:unslash-frame designators-ros:*fixed-frame*))
        pose-stamped
        (let ((result (cl-tf2:transform-pose
                       *transform-listener*
                       :pose pose-stamped
                       :target-frame designators-ros:*fixed-frame*)))
          (out-info "transform result: ~a" result)
          (if result
              result)))))

(defun sort-non-destructive (sequence predicate)
  "Non destructive sorting for sequences."
  (sort (copy-list sequence) predicate))

(defun get-approximated-bounding-box-to-bounding-box-pose (box-1 box-2)
  (out-debug " get-approximated-bounding-box-to-bounding-box-pose()")
  (let* ((obj-box-dim (cl-bullet:bounding-box-dimensions box-1))
         (obj-box-dim-tup `((:x . ,(cl-tf:x obj-box-dim))
                            (:y . ,(cl-tf:y obj-box-dim))
                            (:z . ,(cl-tf:z obj-box-dim))))
         (obj-box-dim-tup-sorted (sort-non-destructive obj-box-dim-tup (lambda (t1 t2) (< (cdr t1) (cdr t2)))))
         (box-dim (cl-bullet:bounding-box-dimensions box-2))
         (box-dim-tup `((:x . ,(cl-tf:x box-dim))
                        (:y . ,(cl-tf:y box-dim))
                        (:z . ,(cl-tf:z box-dim))))
         (box-dim-tup-sorted (sort-non-destructive  box-dim-tup (lambda (t1 t2) (< (cdr t1) (cdr t2)))))
         (axes-mappings (remove-duplicate-tuples
                         `((,(caar obj-box-dim-tup-sorted) . ,(caar box-dim-tup-sorted))
                           (,(caadr obj-box-dim-tup-sorted) . ,(caadr box-dim-tup-sorted))
                           (,(caaddr obj-box-dim-tup-sorted) . ,(caaddr box-dim-tup-sorted)))
                         :car-eq-cdr t))
         (deg90 (* 0.5 pi))
         (box-pose (cl-bullet:bounding-box-center box-2)))
    (cond
      ((null axes-mappings)
       (out-debug "Objects seems to be aligned just fine.")
       nil)
      ((list-contains-exactly-tuples '((:y . :z) (:z . :y)) axes-mappings)
       (out-debug "Interchanged axes: y -> z, z -> y. Rotating around x.")
       (rotate-pose-multiple box-pose `((:x . ,deg90))))
      ((list-contains-exactly-tuples '((:x . :y) (:y . :x)) axes-mappings)
       (out-debug "Interchanged axes: x -> y, y -> x. Rotating around z.")
       (rotate-pose-multiple box-pose `((:z . ,deg90))))
      ((list-contains-exactly-tuples '((:x . :y) (:y . :z) (:z . :x)) axes-mappings)
       (out-debug "Interchanged axes: x -> y, y -> z, z -> x. Rotating around y and x.")
       (rotate-pose-multiple box-pose `((:y . ,deg90) (:x . ,deg90))))
      ((list-contains-exactly-tuples '((:x . :z) (:y . :x) (:z . :y)) axes-mappings)
       (out-debug "Interchanged axes: x -> z, y -> x, z -> y. Rotating around x and y.")
       (rotate-pose-multiple box-pose `((:x . ,deg90) (:y . ,deg90))))
      ((list-contains-exactly-tuples '((:x . :z) (:z . :x)) axes-mappings)
       (out-debug "Interchanged axes: x -> z, z -> x. Rotating around y.")
       (rotate-pose-multiple box-pose `((:y . ,deg90))))
      (t (out-error "Unhandled axes mapping.")
         nil))))

(defun approximate-object-to-bounding-box (name box &optional (world btr:*current-bullet-world*))
  "Approximates the pose of object `name' in `world' to the pose og bounding box `box'.
Therefore it checks for interchanged axes and corrects it by rotating the object.
If there are no interchaged axes, this function does not do anything."
  (out-info "approximate-object-to-bounding-box()")
  (let ((new-pose (get-approximated-bounding-box-to-bounding-box-pose (get-object-bounding-box name world) box)))
    (out-debug "new-pose: ~a" new-pose)
    (if new-pose
        (move-object-by-fixed-pose name new-pose :world world)
        t)))

(defun vector-length (v)
  "Returns the length (euclidean norm) of a vector `v'."
  (sqrt (+ (expt (cl-tf:x v) 2) (expt (cl-tf:y v) 2) (expt (cl-tf:z v) 2))))

(defun rotation-between-vectors (v1 v2)
  "Returns the rotation between vector `va' and vector `v2' as a quaternion."
  (let* ((crossproduct (cl-tf:cross-product v1 v2))
         (dotproduct (cl-tf:dot-product v1 v2))
         (v1-length-pow-2 (expt (vector-length v1) 2))
         (v2-length-pow-2 (expt (vector-length v2) 2))
         (k (sqrt (* v1-length-pow-2 v2-length-pow-2)))
         (scalar (+  k dotproduct)))
    (if (= (/ dotproduct k) -1)
        (let* ((x-axis (cl-tf:make-3d-vector 1 0 0))
               (y-axis (cl-tf:make-3d-vector 0 1 0))
               (some-vector
                 (if (< (abs
                         (cl-tf:dot-product v1 x-axis))
                        1.0)
                     x-axis
                     y-axis))
               (cross (cl-tf:normalize-vector
                       (cl-tf:cross-product v1 some-vector))))
          (cl-tf:normalize
           (cl-tf:axis-angle->quaternion cross pi)))
        (cl-tf:normalize (cl-tf:make-quaternion (cl-tf:x crossproduct)
                                                (cl-tf:y crossproduct)
                                                (cl-tf:z crossproduct)
                                                scalar)))))
         
(defun rotate-object (name axis angle &optional (world btr:*current-bullet-world*))
  "Rotates the object `name' in `world' with given `angle' around `axis'."
  (out-debug "Rotating object ~a around axis ~a by ~a degree." name axis (* angle (/ 180.0 pi)))
  (let* ((old-pose (get-object-pose name :world world))
         (new-pose (rotate-pose old-pose axis angle)))
    (out-debug "old-pose: ~a" old-pose)
    (out-debug "new-pose: ~a" new-pose)
    (move-object-by-fixed-pose name new-pose :world world)))
                                             
(defun rotate-pose-multiple (pose rotations)
  "Rotates `pose' with given angles around given axes.
The format of `roations' is expected to be `(list (:axis . angle) (:axis . angle) ...)'"
  (loop for (axis . angle) in rotations
        do (setf pose (rotate-pose pose axis angle)))
  pose)

(defun rotate-pose (pose axis angle)
  "Rotates `pose' with given `angle' around `axis'."
  (out-debug "Rotating pose ~a around axis ~a by ~a degree." pose axis (* angle (/ 180.0 pi)))
  (let* ((origin (cl-tf:origin pose))
         (orientation (cl-tf:orientation pose))
         (add-rotation (cond
                         ((eq axis :x) (cl-tf:make-quaternion (sin (/ angle 2.0)) 0.0 0.0 (* -1 (cos (/ angle 2.0)))))
                         ((eq axis :y) (cl-tf:make-quaternion 0.0 (sin (/ angle 2.0)) 0.0 (cos (/ angle 2.0))))
                         ((eq axis :z) (cl-tf:make-quaternion 0.0 0.0 (sin (/ angle 2.0)) (cos (/ angle 2.0))))
                         (t (out-error "Unhandled axis: ~a" axis))))
         (new-orientation (cl-transforms:q* add-rotation orientation)))
    (out-debug "add-rotation: ~a" add-rotation)
    (out-debug "old-orientation: ~a" orientation)
    (out-debug "new-orientation: ~a" new-orientation)
    (cl-tf:make-pose origin new-orientation)))

(defun get-all-x-from-solution (variable-name solution &optional (type :list))
  "Returns all possible values for variable `variable-name' in prolog soulution `solution'.
If `type' is `:list', the result will be returned as list.
If `type' is `:vector', the result will be returned as list."
  (let ((results (alexandria:flatten
                  (mapcar (lambda (list) (remove-if #'null (mapcar (lambda (tuple) (if (eq (car tuple) variable-name) (cdr tuple)))
                                                                   list)))
                          solution))))
    (if (eq type :vector)
        (list-to-vector results)
        results)))

(defun get-all-y-for-x-from-solution (x-variable-name y-variable-name solution &key ignore)
  (let ((hash (make-hash-table)))
    (loop for list in solution
          do (let ((x-var-val (list-contains-tuple-car x-variable-name list)))
               (when x-var-val
                 (setf (gethash x-var-val hash)
                       (let ((result-list))
                         (loop for tuple in list
                               do (when (eq (car tuple) y-variable-name) (setf result-list (remove-if
                                                                                            (lambda (elem) (member elem ignore))
                                                                                            (force-ll (cdr tuple))))))
                         result-list)))))
    hash))

(defun is-in-solution (variable-name symbol solution)
  "Returns `t' if the solution `solution' contains a tuple with `variable-name' as its car nad `symbol' as its cdr."
  (find t (mapcar (lambda (list) (list-contains-tuple `(,variable-name . ,symbol) list)) solution)))

(defun list-contains-tuple (tuple list &key ((:permutation perm)))
  "Returns `t' if the list `list' contains a tuple with the same car and cdr values as `tuple'. `list' is expected to be a list of tuples.
If `permutation'x is `t', the tuple's permutation is being evaluated, too."
  (let ((car-val (car list))
        (cdr-val (cdr list)))
    (if (tuple-contains (car car-val) (cdr car-val) tuple :permutation perm)
        t
        (if cdr-val
            (list-contains-tuple tuple cdr-val :permutation perm)))))

(defun list-contains-tuple-car (car-val list)
  "Returns the `cdr' for the tuple in list `list' which contains the same car values as `car-val'. `list' is expected to be a list of tuples."
  (let ((car-v (car list))
        (cdr-v (cdr list)))
    (if (eq car-val (car car-v))
        (cdr car-v)
        (if cdr-v
            (list-contains-tuple-car car-val cdr-v)))))

(defun list-contains-tuples (tuple-list list &key ((:permutation perm)))
  "Returns `t' if the list `list' contains all tuples with the same car and cdr values as given in `tuple-list'.
`list' and `tuple-list' are expected to be a list of tuples.
If `permutation' is `t', the tuples' permutation are being evaluated, too."
  (notany #'null (loop for tup in tuple-list
                      collect (list-contains-tuple tup list :permutation perm))))

(defun list-contains-only-tuples (tuple-list list &key ((:permutation perm)))
  "Returns `t' if the list `list' contains only tuples with the same car and cdr values as given in `tuple-list'.
`list' and `tuple-list' are expected to be a list of tuples.
If `permutation' is `t', the tuples' permutation are being evaluated, too."
  (null (remove-if (lambda (tup)
                     (list-contains-tuple tup tuple-list :permutation perm))
                   list)))

(defun list-contains-exactly-tuples (tuple-list list &key ((:permutation perm)))
  "Returns `t' if the list `list' contains all tuples (with
the same car and cdr values) --  and only them -- as given in `tuple-list'.
`list' and `tuple-list' are expected to be a list of tuples.
If `permutation' is `t', the tuples' permutation are being evaluated, too."
  (out-debug "list-contains-exactly-tuples(): tuple-list: ~a, list: ~a" tuple-list list)
  (loop for tup in (copy-list tuple-list)
        do (if (find tup list :test (lambda (t-1 t-2) (tuple-eq t-1 t-2 :permutation perm)))
             (progn
               (setf tuple-list (remove-if (lambda (tu) (tuple-eq tu tup :permutation perm)) tuple-list))
               (setf list (remove-if (lambda (tu) (tuple-eq tu tup :permutation perm)) list)))))
  (and (null tuple-list) (null list)))
   
(defun tuple-contains (car-val cdr-val tuple &key ((:permutation perm)))
  "Returns `t' if the tuple `tuple' contains `car-val' in its car and `cdr-val' in its cdr."
  (if perm
      (or (and (eq (car tuple) cdr-val) (eq (cdr tuple) car-val))
          (and (eq (car tuple) car-val) (eq (cdr tuple) cdr-val)))
      (and (eq (car tuple) car-val) (eq (cdr tuple) cdr-val))))

(defun tuple-eq (tup-a tup-b &key ((:permutation perm)))
  "Returns `t' if `tup-a' and `tup-b' contain the same values.
If `permutation'x is `t', the tuples' permutations are being evaluated, too."
  (let ((a (car tup-a))
        (b (cdr tup-a)))
    (tuple-contains a b tup-b :permutation perm)))

(defun tuple-car-eq-cdr (tuple)
  "Returns `t' if tuple's `car' equals tuple's `cdr', `nil' otherwise."
  (eq (car tuple) (cdr tuple)))

(defun remove-duplicate-tuples (list &key ((:permutation perm)) car-eq-cdr)
  "Removes all duplicate tuples from `list'.
If `permutation' is `t', the tuples' permutation are being evaluated, too."
  (out-debug "remove-duplicate-tuples(): ~a" list)
  (let ((l (if car-eq-cdr
               (remove-if #'tuple-car-eq-cdr list)
               list)))
    (remove-duplicates l :test (lambda (t-1 t-2) (tuple-eq t-1 t-2 :permutation perm)))))

(defun elt-d (sequence index &optional (default nil))
  "Returns the element number `index' from sequence `sequence' if it exists, `default' otherwise."
  (if (> index (- (length sequence) 1))
      default
      (elt sequence index)))

(defun update-message-array (idx array &rest modifications)
  "Updates (modifies) element number `idx' in `array'.
`array' is expected to be a vector of messages.
`modifications' is expected to be of the form '`message-field' `value' ...'."
  (out-debug "update-message-array()")
  (out-debug "modifications: ~a" modifications)
  (eval `(setf (elt ,array ,idx) (modify-message-copy  (elt ,array ,idx) ,@modifications))))

(defun update-message-arrays (idx modifications &rest arrays)
  "Updates (modifies) element number `idx' in all arrays given in `arrays'.
  `arrays' is expected to be of the form '`array' `boolean-indicating-if-array-should-be-modified-or-left-alone''
  `modifications' is expected to be of the form '(`message-field' `value' ...)'."
  (out-debug "update-message-arrays()")
  (loop
    for array in arrays by #'cddr
    for modify in (cdr arrays) by #'cddr
    do (when modify
         (apply #'update-message-array idx array modifications))))

(defun vector-to-list (vec)
  "Converts the vector `vec' into a list and returns it."
  (loop for elem across vec collect elem))

(defun list-to-vector (list)
  "Converts the list `list' into a vector and returns it."
  (loop for elem in list
        with vec = (make-array (length list) :fill-pointer 0)
        finally (return vec)
        do (vector-push elem vec)))

(defun make-keyword (name)
  "Creates a keyword from the string `name' and returns it."
  (cond
   ((typep name 'integer) (values (intern (write-to-string name) "KEYWORD")))
   ((typep name 'string) (values (intern name "KEYWORD")))))

(defun resolve-keyword (keyword &optional (type 'integer))
  (cond
    ((eq type 'integer) (multiple-value-bind (int) (read-from-string (symbol-name keyword)) int))
    ((eq type 'string (symbol-name keyword)))))
  

(defun get-object-type (type)
  "Returns the object type for the given object type constant `type'."
  (out-info "get-object-type()")
  (let ((bowl-const (get-object-type-constant ':BOWL))
        (nesquik-const (get-object-type-constant ':NESQUIK))
        (mondamin-const (get-object-type-constant ':MONDAMIN))
        (fruit-apple-const (get-object-type-constant ':FRUIT_APPLE))
        (fruit-orange-const (get-object-type-constant ':FRUIT_ORANGE))
        (sugar-const (get-object-type-constant ':SUGAR)))
    (cond
      ((eq type bowl-const) 'spatial-relations-demo::bowl)
      ((eq type nesquik-const) 'spatial-relations-demo::cereal)
      ((eq type mondamin-const) 'spatial-relations-demo::mondamin)
      ((eq type fruit-apple-const) 'spatial-relations-demo::apple)
      ((eq type fruit-orange-const) 'spatial-relations-demo::orange)
      ((eq type sugar-const) 'spatial-relations-demo::sugar-box)
      (t (out-error "Unhandled object-type: ~a" type)))))

(defun get-error-level-constant (error-name)
  "Returns the error level constant for the given error `error-name'."
  (let ((ret (roslisp-msg-protocol:symbol-code 'bullet_reasoning_interface-srv:<interaction-request> error-name)))
    (out-debug "error-level-constant ~a: ~a" error-name ret)
    ret))

(defun get-operation-constant (name)
  "Returns the operation constant for the given operation `name'."
  (let ((ret (roslisp-msg-protocol:symbol-code 'bullet_reasoning_interface-msg:ObjectOperation name)))
    (out-debug "operation-constant ~a: ~a" name ret)
    ret))

(defun get-world-type-constant (name)
  "Returns the world type constant for the given world-type `name'."
  (let ((ret (roslisp-msg-protocol:symbol-code 'bullet_reasoning_interface-msg:WorldIntel name)))
    (out-debug "world-type-constant ~a: ~a" name ret)
    ret))

(defun get-object-type-constant (name)
  "Returns the object type constant for the given object type `name'."
  (let ((ret (roslisp-msg-protocol:symbol-code 'bullet_reasoning_interface-msg:ObjectIntel name)))
    (out-debug "type-constant ~a: ~a" name ret)
    ret))

