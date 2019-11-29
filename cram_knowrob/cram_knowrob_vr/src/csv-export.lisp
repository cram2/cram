;;;
;;; Copyright (c) 2019, Thomas Lipps <tlipps@uni-bremen.de>
;;;
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :kvr)

(defvar *episode-names*
  (list "human-muesli-1" "human-muesli-5" "rob-muesli-10"
        "rob-muesli-4" "rob-muesli-8"
        "human-muesli-2" "rob-muesli-11" "rob-muesli-5" "rob-muesli-9"
        "human-muesli-3" "rob-muesli-2" "rob-muesli-6"
        "human-muesli-4" "rob-muesli-1" "rob-muesli-3" "rob-muesli-7"
        "right_side_table_muesli_1"
        "right_side_table_muesli_2" "right_side_table_muesli_3"
        "right_side_table_muesli_4" "right_side_table_muesli_5"
        "right_side_table_muesli_6" "right_side_table_muesli_7"
        "right_side_table_muesli_8" "right_side_table_muesli_9"
        "full_breakfast_setup_2" "full_breakfast_setup_3"
        "full_breakfast_setup_4" "full_breakfast_setup_5"
        "full_breakfast_setup_6" "full_breakfast_setup_7"
        "full_breakfast_setup_1" "full_breakfast_setup_8"
        "full_breakfast_setup_9" "full_breakfast_setup_10"
        "full_breakfast_setup_11"))

(defun export-vr-data-in-csv (&optional (kitchen :kitchen) (name :thomas) (context :table-setting))
  (let* ((muesli (list
                  ;; cuterly
                  "SpoonSoup" "SpoonDessert"
                  "KnifeTable" ;; "KnifeButter" <- dont find any at end
                  ;; Bowls and Plates
                  "BowlLarge" "Bowl"
                  "PlateClassic28"
                  ;; Cups and glasses
                  "GlassTall" "GlassRound" "Cup"
                  ;; food     
                  "KoellnMuesliKnusperHonigNuss" "JaNougatBits"
                  "KoellnMuesliCranberry"
                  ;; "KellogsCornFlakesOriginal"  <- dont find any
                  ;; drinks
                  "BaerenMarkeFrischeAlpenmilch38" "HohesCOrange"
                  ;; other
                  "Tray"))
         (kitchen-name-context (mapcar #'keyword-to-string
                                       (list kitchen name context)))
         (samples (format-samples
                   (mapcar (alexandria:curry #'append kitchen-name-context) ;; addr more features
                           (apply #'append ;; objects types in one list
                                  (loop for object-type in muesli collect
                                                                  (samples-for-object-type object-type)))))))
    (create-csv samples)))

(defun create-csv (samples)
  (with-open-file (csv-stream "/home/thomas/nameisthiscsvname.csv" :direction :output :if-exists :supersede)
    (write-string "kitchen_name,human_name,context,object-type,from-location,to-location,x,y,arm,object-orient-from-cam" csv-stream)
    (loop for sample in samples do
      (print sample)
      (fresh-line csv-stream)
      (loop for feature in sample do
        (write-string feature csv-stream)
        (unless (equal (first (last sample)) feature)
          (write-char #\, csv-stream))))))

(defun format-samples (samples)
  "removes chars ' and | and : in samples"
  (mapcar (lambda (sample)
            (loop for feature in sample collect
                                        (remove-chars-in-given-string feature)))
          samples))

(defun keyword-to-string (kw)
  (if (stringp kw)
      kw
      (remove-chars-in-given-string (write-to-string kw))))

(defun remove-chars-in-given-string (string &optional (ugly-chars '(#\| #\' #\:)))
  (let* ((splitted-string (split-sequence:split-sequence #\| (feature-to-string string))))
    (reduce (alexandria:curry #'concatenate 'string)
            (loop for elem in splitted-string collect
                                               (string-trim ugly-chars elem)))))
          
(defun feature-to-string (feature)
  (if (stringp feature)
      feature
      (if (numberp feature)
          (double-to-string feature)
          (write-to-string feature))))

(defun double-to-string (double)
  (let ((start 0)
        (end (if (> double 0) 6 7)))
    (subseq (first
             (split-sequence:split-sequence #\d (write-to-string double))) start end)))
