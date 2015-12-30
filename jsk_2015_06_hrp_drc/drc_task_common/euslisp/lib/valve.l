(require "models/arrow-object")
(load "package://drc_task_common/euslisp/lib/collision-link-parameter.l")
(load "package://drc_task_common/euslisp/lib/ik-solver.l")
(load "package://drc_task_common/euslisp/test-drc-door-task.l")
(load "package://drc_task_common/euslisp/lib/periodic-rate.l")
(load "package://drc_task_common/euslisp/lib/util.l")
(load "package://drc_task_common/euslisp/lib/trajectory.l")
(load "package://drc_task_common/euslisp/lib/zmp.l")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")

(defun move-valve-model-from-recog-result (valve origin->recog robot-coords)
  (send valve :newcoords (send origin->recog :copy-worldcoords)))

(defun test-valve-recognition-callback (msg)
  (print 'callback)
  ;; msg := jsk_recognition_msgs::Torus
  (let ((header->valve (ros::tf-pose->coords (send msg :pose)))
        (origin->header (send *tf-listener* :lookup-transform 
                              "odom"
                              (send msg :header :frame_id)
                              (send msg :header :stamp)))
        (origin->ground (send *tf-listener* :lookup-transform "odom" "ground" (ros::time 0))))
    (when (and origin->header origin->ground)
      (setq *perception-succeeded* t)
      (let ((origin->valve (send (send origin->header :copy-worldcoords)
                                 :transform header->valve)))
        (send *robot* :fix-leg-to-coords origin->ground)
        (send *recog-result* :newcoords (send origin->valve :copy-worldcoords))
        (move-valve-model-from-recog-result *valve* origin->valve origin->ground)
        (send *irtviewer* :draw-objects)
        (x::window-main-one)
        (let ((marker (send *traj* :to-marker "odom")))
          (ros::publish "trajectory_marker" marker))
        ))
    ))

(defun valve-manipulation-trajectory (valve-model &key (robot :hrp2) (z-offset -100))
  (let* ((valve-tmp (valve-model)))
    (let ((initial-theta 45)
          ;;(angle -270))
          (angle -90))
      ;; radius ... 200 mm
      (let ((theta initial-theta)
            (coords-list nil))
        (while (> theta angle)
          (setq coords-list (append coords-list (list (coords-on-valve theta))))
          (setq theta (- theta 20))     ;20 degree
          )
        (instance manipulation-trajectory :init
                  :via-coords
                  (append (list
                           (make-cascoords 
                            :coords
                            (send (send (car coords-list) :copy-worldcoords)
                                  :translate (float-vector -250 0 0))))
                          (mapcar #'(lambda (c i)
                                      (send c :rotate (* (/ (float i) (length coords-list))  (deg2rad -15)) :z)
                                      (make-cascoords :coords (send (send c :copy-worldcoords)
                                                                    :translate (float-vector -100 0 0))))
                                  coords-list (range (length coords-list)))
                          (list
                           (make-cascoords 
                            :coords
                            (send (send (car (last coords-list)) :copy-worldcoords)
                                  :translate (float-vector -250 0 0))
                            )
                           (make-cascoords 
                            :coords
                            (send (send (car (last coords-list)) :copy-worldcoords)
                                  :translate (float-vector -350 150 -100)))
                           )
                          )
                  :parent-object valve-model)))))

(defun coords-on-valve (theta &optional (r 180))
  (let ((c (make-cascoords :pos (point-on-valve theta r))))
    (send c :rotate pi/2 :z)
    (send c :rotate -pi/2 :y)
    (send c :rotate (deg2rad (+ theta)) :x)
    c))

(defun point-on-valve (theta &optional (r 180))
  (float-vector (* r (cos (deg2rad theta)))
                (* r (sin (deg2rad theta)))
                0))

(defun valve-model ()
  (make-ring 200 20 :segments 16))

(defun test-run-valve-recognition ()
  (setq *perception-succeeded* nil)
  (ros::spin-once)
  (setq *perception-succeeded* nil)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    (if *perception-succeeded*
        (return-from test-run-valve-recognition nil))
    ))

(defun test-valve ()
  (load-robot-model :jaxon)
  (unless (boundp '*robot*)
    (setq *robot* (instance jaxon_red-robot :init)))
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *valve* (valve-model))
  (setq *traj* (valve-manipulation-trajectory *valve*))
  (send *valve* :newcoords (make-coords :pos (float-vector 900 -300 1200)))
  (send *valve* :rotate (deg2rad 90) :y)
  (send *valve* :rotate (deg2rad -90) :z)
  (objects (list *robot* *valve* *traj*))
  )

(defun test-valve-recognition (robot-type)
  (ros::roseus "test_valve_recognition")
  (setq *tf-listener* (instance ros::transform-listener :init))
  (load-robot-model :jaxon)
  (unless (boundp '*ri*)
    (jaxon_red-init))
  (unless (boundp '*robot*)
    (setq *robot* (instance jaxon_red-robot :init)))
  (setq *valve* (valve-model))
  (setq *traj* (valve-manipulation-trajectory *valve*))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *valve* *recog-result* *traj*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::subscribe "/valve_detection/icp_registration/output_pose" 
                  geometry_msgs::PoseStamped
                  #'test-valve-recognition-callback)
  ;; run recognition until perception succeeded
  (test-run-valve-recognition)
  )

(defun test-valve-motion ()
  ;;(send *ri* :start-impedance :arms :k-p 300 :d-p 800 :m-p 10 :m-r 2 :d-r 80 :k-r 40)
  ;;(send *ri* :set-interpolation-mode :linear)
  (setq *col* (instance collision-link-parameter :init *robot*))
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              ;;:ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (send *robot* :reset-manip-pose)
  (send *col* :init-distance-table)
  (send *ik-solver* :solve
               :trajectory-resolution 20
               :stop 50
               :collision-num 10 :debug-view nil)
  )

(defun valve-motion-profile ()
  (test-valve)
  (setq *col* (instance collision-link-parameter :init *robot*))
  (send *col* :init-distance-table)
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              ;;:ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (send *ik-solver* :profile)
  )

(defun test-valve-zmp ()
  (ros::roseus "foo")
  (format t "setup environment~%")
  (test-valve)                          ;setup env
  (format t  "generate motion~%")
  (let* ((avs (test-valve-motion))
         (zmp-robot (instance jaxon_red-robot :init))
         (zmp (instance zmp-evaluator :init zmp-robot)))
    (format t "generating zmp trajectory~%")
    (let ((zmp-traj (send zmp :evaluate-zmp :angle-vector-sequence avs
                              :start-angle-vector (send *robot* :reset-manip-pose)
                              :dt 0.01
                              :speed-factor 10.0
                              :debug-view nil)))
      (send zmp-traj :publish-zmp-plot)
      (send zmp-traj :evaluate)
      )))

;; (load "package://drc_task_common/euslisp/lib/valve.l")
;; (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
;; (test-valve-recognition :jaxon)
;; (test-run-valve-recognition)
;; (setq hoge (test-valve-motion))
;; (send hoge :send-angle-vector-sequence-test *ri* *robot* 10.0)
;; (send hoge :send-angle-vector-sequence-test *ri* *robot* 5.0)
;; (send hoge :send-angle-vector-sequence-test *ri* *robot* 3.0)
;; (send hoge :send-angle-vector-sequence-test *ri* *robot* 2.0)
;; (send hoge :send-angle-vector-sequence-test *ri* *robot* 1.0)
;; (do-until-key (test-run-valve-recognition) (setq hoge (test-valve-motion)) (send hoge :filter-look-at-hand *robot* :rarm) (send hoge :send-angle-vector-sequence-test *ri* *robot* 2.0) (send *ri* :wait-interpolation) (unix:sleep 10))
