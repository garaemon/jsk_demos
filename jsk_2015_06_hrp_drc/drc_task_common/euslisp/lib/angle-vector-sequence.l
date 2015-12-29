(load "package://drc_task_common/euslisp/lib/periodic-rate.l")

(defclass angle-vector-sequence
  :super propertied-object
  :slots (angle-vectors))

(defmethod angle-vector-sequence
  (:init ()
    )
  (:add (av)
    (setq angle-vectors (append angle-vectors (list av))))
  (:angle-vectors () angle-vectors)
  (:filter-look-at-hand (robot arm)
    (dotimes (i (length angle-vectors))
      (send robot :angle-vector (elt angle-vectors i))
      (send robot :head :look-at (send robot arm :end-coords :worldpos))
      (setf (elt angle-vectors i) (send robot :angle-vector))
      )
    )
  (:at (i)
    (elt angle-vectors i))
  (:duration-list 
   (robot-model start-angle-vector &optional (time-scale 1.0))
   (setf (get self :initial-pose) start-angle-vector) ;store start-angle-vector for profiling
   (let ((times nil)
         (prev-time 0)
         (prev-angle start-angle-vector)
         (angle-vectors-with-initial (cons start-angle-vector (butlast angle-vectors))))
     (dotimes (i (length angle-vectors))
       (let ((tm (send self :angle-vector-duration 
                       (elt angle-vectors-with-initial i)
                       (elt angle-vectors i)
                       prev-angle
                       prev-time
                       robot-model
                       time-scale)))
         (setq times (append times (list tm)))
         (setq prev-time tm)
         (setq prev-angle (elt angle-vectors-with-initial i))
       ))
     times))
  (:compute-joint-time-from-angle-vector-difference
   (js diffs robot)
   ;; compute time from the fastest joint
   (apply #'max (mapcar #'(lambda (j diff)
                            (send self :compute-joint-time j diff robot))
                        js diffs)))
  (:compute-joint-time
   (j d robot)
   (/ (if (derivedp j linear-joint)
          (* 0.001 (abs d))
        (deg2rad (abs d)))
      (send self :max-joint-velocity robot j)))
  (:find-fastest-joint 
   (start end robot-model)
   (let* ((joint-list (send robot-model :joint-list)))
     (let ((diff (coerce (v- end start) cons)))
       (caar (sort (mapcar #'(lambda (j d)
                               (cons j (send self :compute-joint-time j d)))
                           joint-list diff)
                   #'(lambda (jd0 jd1) (> (cdr jd0) (cdr jd1))))))))
  (:find-fastest-joint-vel
   (start end robot-model)
   (let* ((joint-list (send robot-model :joint-list)))
     (let ((diff (coerce (v- end start) cons)))
       (caddr (car (sort (mapcar #'(lambda (j d)
                                     (list j (send self :compute-joint-time j d)
                                           (/ d (send self :compute-joint-time j d))))
                               joint-list diff)
                         #'(lambda (jd0 jd1) (> (elt jd0 1) (elt jd1 1)))))))))
  (:angle-vector-duration 
   (start end prev-angle prev-time robot-model
          time-scale
          &optional
          (end-effector-max-position-velocity 1200)
          (angle-max-acc (rad2deg 3.0)))
   ;; start      ... t
   ;; end        ... t+1
   ;; prev-angle ... t-1
   (let ((joint-list (send robot-model :joint-list)))
     (let ((diff-t (coerce (v- end start) cons))
           (diff-t-1 (coerce (v- start prev-angle) cons)))
       (let* ((native-t (send self :compute-joint-time-from-angle-vector-difference
                              joint-list diff-t robot-model))
              (vel-t (mapcar #'(lambda (v) (/ v native-t)) diff-t))
              (vel-t-1 (mapcar #'(lambda (v)
                                   (if (= prev-time 0)
                                       0
                                     (/ v prev-time))) diff-t-1)))
         ;;(let* ((acc-t (mapcar #'(lambda (vt vt-1) (abs (* 1 (/ (- vt vt-1) native-t))))
         (let* ((acc-t (mapcar #'(lambda (vt vt-1) (abs (* 1 (/ (- vt vt-1) 1.0))))
                               vel-t vel-t-1))
                (ee-vel-t (send self :end-effector-speed start end robot-model native-t))
                (ee-pos-vel-t (mapcar #'norm (mapcar #'car ee-vel-t))))
           ;; (format t "vel-t-1: ~A~%" vel-t-1)
           ;; (format t "vel-t: ~A~%" vel-t)
           ;; (format t "acc-t: ~A~%" acc-t)
           (let ((max-acc-t (apply #'max acc-t))
                 (max-pos-vel-t (apply #'max ee-pos-vel-t)))
             ;; (format t "max-acc-t: ~A~%" max-acc-t)
             ;; (format t "native-t: ~A~%" native-t)
             ;; decide reason...n
             (cond ((> max-acc-t angle-max-acc)
                    (warn "--> angle acc limit. acc=~A~%" max-acc-t)
                    (* time-scale (/ max-acc-t angle-max-acc) native-t))
                   ((> max-pos-vel-t end-effector-max-position-velocity)
                    (warn "--> ee vel limit. val=~A~%" max-pos-vel-t)
                    (* time-scale (/ max-pos-vel-t end-effector-max-position-velocity) native-t))
                   (t
                    (* time-scale native-t)))))))))
  (:end-effector-speed 
   (start end robot tm)
   ;; compute end-effector speed
   (send robot :angle-vector start)
   (let ((before-end-coords
          (send-all (send robot :arms :end-coords) :copy-worldcoords))
         (before-coords (send robot :foot-midcoords)))
     (send robot :angle-vector end)
     (send robot :fix-leg-to-coords before-coords)
     (let ((after-end-coords
            (send-all (send robot :arms :end-coords) :copy-worldcoords)))
       (mapcar #'(lambda (before after)
                   (cons (scale (/ 1.0 tm) (send before :difference-position after))
                         (scale (/ 1.0 tm) (send before :difference-rotation after))))
               before-end-coords
               after-end-coords)
       )))
  (:profile-csv
   (robot &key (time-scale 1.0) (start-av (get self :initial-pose)))
   (let ((joint-list (send robot :joint-list)))
     (format t "tm,~A~%" (string-join "," (send-all joint-list :name)))
     (format t "0,~A~%" (string-join "," (mapcar #'string (coerce start-av cons))))
     (let ((angle-vectors-with-initial (cons start-av angle-vectors))
           (start-date 0))
       (dotimes (i (length angle-vectors))
       (let* ((start (elt angle-vectors-with-initial i))
              (end (elt angle-vectors-with-initial (1+ i)))
              (tm (send self :angle-vector-duration start
                        end
                        robot
                        time-scale)))
         (format t "~A,~A~%" (+ start-date tm) (string-join "," (mapcar #'string (coerce end cons))))
         (setq start-date (+ start-date tm))
         )
       )
     )))
  (:profile-duration 
   (robot &key (time-scale 1.0) (start-av (get self :initial-pose)))
   (format t "Profiling angle-vectors with time-scale=~A~%" time-scale)
   (format t "  time-scale: ~A~%" time-scale)
   (format t "  number of angle vectors: ~A~%" (length angle-vectors))
   (let ((angle-vectors-with-initial (cons start-av angle-vectors))
         (prev-ee-speed (list (cons (float-vector 0 0 0) (float-vector 0 0 0))
                              (cons (float-vector 0 0 0) (float-vector 0 0 0)))))
     (dotimes (i (length angle-vectors))
       (format t "    sequence: ~A~%" i)
       (let* ((start (elt angle-vectors-with-initial i))
              (end (elt angle-vectors-with-initial (1+ i)))
              (tm (send self :angle-vector-duration start
                        end
                        robot
                        time-scale)))
         (format t "      fastest joint: ~A~%" 
                 (send (send self :find-fastest-joint
                             start end robot) :name))
         (format t "      fastest join velt: ~A~%" 
                 (send self :find-fastest-joint-vel
                       start end robot))
         (format t "      time: ~A~%" tm)
         (let ((end-effector-speed (send self :end-effector-speed start end robot tm))
               (limbs '("larm" "rarm")))
           
           (dotimes (i (length end-effector-speed))
             (format t "      ~A:~%" (elt limbs i))
             (format t "        pos vel: ~A (~A)~%" 
                     (car (elt end-effector-speed i))
                     (norm (car (elt end-effector-speed i))))
             (format t "        rot vel: ~A (~A)~%"
                     (cdr (elt end-effector-speed i))
                     (norm (cdr (elt end-effector-speed i))))
             (let ((acc-pos (scale (/ 1.0 tm) (v- (car (elt end-effector-speed i)) (car (elt prev-ee-speed 0)))))
                   (acc-rot (scale (/ 1.0 tm) (v- (cdr (elt end-effector-speed i)) (cdr (elt prev-ee-speed 0))))))
             (format t "        pos acc: ~A (~A -- ~A)~%" 
                     acc-pos (norm acc-pos)
                     (* (send (elt (send *robot* :arms :end-coords :parent) i) :weight)
p                        (norm acc-pos) 0.001 0.001))
             (format t "        rot acc: ~A (~A -- ~A)~%" acc-rot (norm acc-rot)
                     (* (send (elt (send *robot* :arms :end-coords :parent) i) :weight)
                        (norm acc-rot)  0.001 0.001))
             )
           )
         )
       )
     )))
  (:send-angle-vector-sequence-test 
   (ri robot time-scale)
   (let ((ts (send self :duration-list robot (send ri :state :potentio-vector) time-scale)))
     (format t "ts: ~A~%" ts)
     (format t "avs: ~A~%" angle-vectors)
     (print `(send *ri* :angle-vector ,angle-vectors ,ts))
     (send ri :angle-vector-sequence angle-vectors (mapcar #'(lambda (x) (* 1000 x)) ts))
     ts))
  (:send-angle-vector-test 
   (ri time index)
   (send ri :angle-vector (send self :at index) time))
  (:max-joint-velocity
   (robot j)
   (or (send self :customized-joint-velocity (send robot :name) j)
       (send j :max-joint-velocity)))
  (:customized-joint-velocity 
   (robot-name j)
   (cond ((string= robot-name "JAXON_RED")
          (cond ((string= (send j :name) "CHEST_JOINT2")
                 1.0)
                ((or (string= (send j :name) "RARM_JOINT6")
                     (string= (send j :name) "LARM_JOINT6"))
                 2.0)
                ((or (string= (send j :name) "RARM_JOINT5")
                     (string= (send j :name) "LARM_JOINT5"))
                 2.0)
                ((or (string= (send j :name) "RARM_JOINT4")
                     (string= (send j :name) "LARM_JOINT4"))
                 2.0)
                ((or (string= (send j :name) "RARM_JOINT0")
                     (string= (send j :name) "LARM_JOINT0"))
                 1.5)
                ((or (string= (send j :name) "RARM_JOINT1")
                     (string= (send j :name) "LARM_JOINT1"))
                 1.5)
                ((or (string= (send j :name) "RARM_JOINT2")
                     (string= (send j :name) "LARM_JOINT2"))
                 1)
                ((or (string= (send j :name) "RARM_JOINT3")
                     (string= (send j :name) "LARM_JOINT3"))
                 1.5)
                ((eps= (send j :max-joint-velocity) 4.0)
                 2.0)
                ))))
  (:animate-angle-vector 
   (robot-model &optional (r 1.0))
   (let ((pr (instance periodic-rate :init r)))
     (dolist (av angle-vectors)
       (send robot-model :angle-vector av)
       (send *irtviewer* :draw-objects)
       (x::window-main-one)
       (send pr :sleep))
     )
   )
  )

