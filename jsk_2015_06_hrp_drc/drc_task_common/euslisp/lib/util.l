(require "models/arrow-object")

(defun arrow-object (&optional (scale 1.0))
  (let ((obj (instance arrow-object :init)))
    (send-all (send obj :bodies) :magnify scale)
    obj))

(defun range (max)
  (let ((ret nil))
    (dotimes (i max)
      (setq ret (append ret (list i)))
      )
    ret))

(defun arange (from to step)
  (let ((ret nil)
        (current from))
    (while (< current to)
      (setq ret (append ret (list current)))
      (setq current (+ current step))
      )
    ret))

(defun c* (a b)
  (send a :transform b)
  )

(defun z0 (pos)
  (float-vector (elt pos 0) (elt pos 1) 0))
(defun zset (pos z)
  (float-vector (elt pos 0) (elt pos 1) z))

(defun load-robot-model (robot-type)
  (if (eq robot-type :hrp2)
      (require "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2jsknt-interface.l")
    (require "package://hrpsys_ros_bridge_tutorials/euslisp/jaxon_red-interface.l")
    ))

(defun string-join (prefix strs)
  (reduce #'(lambda (a v)
              (concatenate string a prefix v))
          strs)
  )

(defun string-split (splitter str)
  "hoge,fuga,piyo => (hoge fuga piyo)"
  (string-split-impl splitter str nil))

(defun string-split-impl (splitter str result)
  (cond ((string= str "")
         result)
        ((position splitter str)
         (let ((pos (position splitter str)))
           (string-split-impl splitter (subseq str (1+ pos) (length str))
                              (append result (list (subseq str 0 pos))))))
        (t
         (append result (list str)))))


(defun transpose-list-array (data)
  ;; data := '((a0 b0 c0) (a1 b1 c1) .. (aN bN cN))
  ;; output := '((a0 a1 .. aN) ... (c0 c1 .. cN))
  (let ((row (length data))
        (column (length (elt data 0))))
    (mapcar #'(lambda (i)
                (mapcar #'(lambda (d)
                            (elt d i))
                        data))
            (range column))
    ))

(defun mean (data)
  (/ (reduce #'+ data) (length data)))