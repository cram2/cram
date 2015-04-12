(in-package :reas-inf)

(defvar *transform-listener*)

(defun set-transform-listener ()
  "Sets the transform listener to a new instance."
  (out-info "Setting *transform-listener*")
  (defparameter *transform-listener* (make-instance 'cl-tf2:buffer-client)))

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

(defun is-in-solution (variable-name symbol solution)
  "Returns `t' if the solution `solution' contains a tuple with `variable-name' as its car nad `symbol' as its cdr."
  (find t (mapcar (lambda (list) (list-contains-tuple `(,variable-name . ,symbol) list)) solution)))

(defun list-contains-tuple (tuple list)
  "Returns `t' if the list `list' contains a tuple with the same car and cdr values as `tuple'. `list' is expected to be a list of tuples."
  (let ((car-val (car list))
        (cdr-val (cdr list)))
    (if (tuple-contains (car car-val) (cdr car-val) tuple)
        t
        (if cdr-val
            (list-contains-tuple tuple cdr-val)))))

(defun tuple-contains (car-val cdr-val tuple)
  "Returns `t' if the tuple `tuple' contains `car-val' in its car and `cdr-val' in its cdr."
  (and (eq (car tuple) car-val) (eq (cdr tuple) cdr-val)))

(defun elt-d (sequence index &optional (default nil))
  "Returns the element number `index' from sequence `sequence' if it exists, `default' otherwise."
  (if (> index (- (length sequence) 1))
      default
      (elt sequence index)))

(defun update-message-array (idx array &rest modifications)
  "Updates (modifies) element number `idx' in `array'.
`array' is expected to be a vector of messages.
`modifications' is expected to be of the form '`message-field' `value' ...'."
  (out-info "update-message-array()")
  (out-debug "modifications: ~a" modifications)
  (eval `(setf (elt ,array ,idx) (modify-message-copy  (elt ,array ,idx) ,@modifications))))

(defun update-message-arrays (idx modifications &rest arrays)
  "Updates (modifies) element number `idx' in all arrays given in `arrays'.
  `arrays' is expected to be of the form '`array' `boolean-indicating-if-array-should-be-modified-or-left-alone''
  `modifications' is expected to be of the form '(`message-field' `value' ...)'."
  (out-info "update-message-arrays()")
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
  (values (intern name "KEYWORD")))

(defun get-object-type (type)
  "Returns the object type for the given object type constant `type'."
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
      ((eq type sugar-const) 'spatial-relations-demo::sugar-box))))

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

