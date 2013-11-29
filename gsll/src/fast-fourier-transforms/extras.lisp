;; Extras for FFT.
;; Sumant Oemrawsingh, Wed Nov 25 2009 - 22:03
;;
;; Copyright 2009 Sumant Oemrawsingh
;; Distributed under the terms of the GNU General Public License
;;
;; This program is free software: you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.
(in-package :gsll)

(defun fft-frequency-step (size &optional (sample-spacing 1))
  "Return the frequency step for the FFT of a vector with the given size and
  assuming the given sample spacing."
  (/ (* size sample-spacing)))

(defun fft-highest-frequency (size &optional (sample-spacing 1))
  "Return the highest frequency for the FFT of a vector with the given size and
  assuming the given sample spacing."
  (if (evenp size)
    (values (/ sample-spacing 2) (/ size 2))
    (- (/ sample-spacing 2) (/ (fft-frequency-step size sample-spacing) 2))))

(defun fft-frequency-split (size)
  "Return the index where the highest frequency component is located in a
  vector of the given size, after applying an FFT.  The second value that is
  returned, is the most negative frequency divided by the sample frequency
  step."
  (if (evenp size)
    (let ((n (/ size 2)))
      (values n (- 1 n)))
    (let ((n (/ (- size 1) 2)))
      (values n (- n)))))

(export 'fft-frequency-vector)
(defun fft-frequency-vector (element-type size &key (sample-spacing 1) shifted)
  "Make and return a vector that contains the sample frequencies of an FFT
  that has been applied to a vector with the given size and :sample-spacing.
  
  If :shifted is T, then the vector will contain the sample frequencies after
  FFT-SHIFT has been applied to the result of an FFT."
  (let* ((df (fft-frequency-step size sample-spacing))
        pos-f
        neg-f)
    (multiple-value-bind (max-f-index min-f-index)
      (fft-frequency-split size)
      (setf pos-f
            (loop for i from 0 to max-f-index
                  collect (coerce (* i df) element-type))
            neg-f
            (loop for i from min-f-index below 0
                  collect (coerce (* i df) element-type)))
      (grid:make-foreign-array element-type
                   :initial-contents (if shifted
                                       (nconc neg-f pos-f)
                                       (nconc pos-f neg-f))))))

(export 'fft-shift)
(defun fft-shift (vector &key (stride 1))
  "Return a copy of a vector that is the result of an FFT, with the zero and
  positive frequencies shifted to the center and end, so that the data is
  suitable for e.g. plotting."
  (let* ((size (size vector))
         (n (floor size stride))
         (shifted (copy vector))
         (split (fft-frequency-split n))
         (zero-pos (if (evenp n) (- split 1) split)))
    ;; Positive frequencies
    (loop for i from zero-pos to (- n 1)
          do (setf (grid:gref shifted (* i stride))
                   (grid:gref vector (* (- i zero-pos) stride))))
    ;; Negative frequencies
    (loop for i from (+ split 1) to (- n 1)
          do (setf (grid:gref shifted (* (- i split 1) stride))
                   (grid:gref vector (* i stride))))
    shifted))

(export 'fft-inverse-shift)
(defun fft-inverse-shift (vector &key (stride 1))
  "Return a copy of a vector where the zero and positive frequency components
  are shifted to the beginning, i.e. ordered so that it is suitable for an
  inverse FFT."
  (let* ((size (size vector))
         (n (floor size stride))
         (inv-shifted (copy vector))
         (split (fft-frequency-split n))
         (zero-pos (if (evenp n) (- split 1) split)))
    ;; Positive frequencies
    (loop for i from zero-pos to (- n 1)
          do (setf (grid:gref inv-shifted (* (- i zero-pos) stride))
                   (grid:gref vector (* i stride))))
    ;; Negative frequencies
    (loop for i from (+ split 1) to (- n 1)
          do (setf (grid:gref inv-shifted (* i stride))
                   (grid:gref vector (* (- i split 1) stride))))
    inv-shifted))
