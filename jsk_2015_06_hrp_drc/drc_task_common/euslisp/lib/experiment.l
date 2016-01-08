;; for experiment.l
(load "package://drc_task_common/euslisp/lib/valve.l")
(load "package://drc_task_common/euslisp/lib/door.l")

(defun init ()
  ;; only support jaxon
  (load-robot-model :jaxon)
  (unless (boundp '*ri*)
    (jaxon_red-init))
  (ros::rate 10)
  (setq *tf-listener* (instance ros::transform-listener :init))
  (unless (boundp '*robot*)
    (setq *robot* (instance jaxon_red-robot :init)))
  )

(defun wait-for-recognition ()
  (let ((start-time (ros::time-now)))
    (setq *perception-succeeded* nil)
    (while (not (and *perception-succeeded*
                     (ros::time> *perception-succeeded-time*  start-time)))
      (ros::spin-once)
      (ros::sleep)
      )))

(defun run-valve (dx collision-num trajectory-num speed-factor)
  ;; dx := mm
  ;; steps
  ;; 0. setup ros stuff
  ;; 1. set octree resolution
  ;; 2. clear laser cache
  ;; 3. wait for recognition
  ;; 4. plan motion
  ;; 5. exec motion
  
  ;; 0. setup ros stuff
  ;; reset robot
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *valve* (valve-model))
  (setq *traj* (valve-manipulation-trajectory *valve*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (setq *perception-succeeded-time* (ros::time-now))
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              ;;:ik-type :fullbody-with-standcoords
                              :move-target (send *robot* :rarm :end-coords)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :reset-manip-pose)))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *valve* *recog-result* *traj*))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/door_detection/door_handle")
  (ros::subscribe "/valve_detection/icp_registration/output_pose"
                  geometry_msgs::PoseStamped
                  #'test-valve-recognition-callback)
  ;; 1. set octree resolution
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/valve_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx))
  ;; 2. clear cache
  (ros::ros-info "clearing laser cache")
  (bench
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  ;; 4. plan motion
  (bench
   (ros::ros-info "planning motion")
   (setq *avs* (send *ik-solver* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
  (bench
   (if *avs*
       ;; 5. exec motion
       (progn
         (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
           (ros::ros-info "execution: ~A" tm))
         (send *ri* :wait-interpolation))
       ;;nil
     (ros::ros-warn "Failed to solve ik")))
  (send *ri* :angle-vector (send *robot* :reset-manip-pose) 5000)
  (send *ri* :wait-interpolation)
  )

(defun run-door (dx collision-num trajectory-num speed-factor)
  (send *ri* :start-impedance :arms :moment-gain (float-vector 0 0 0) :k-p 3000 :d-p 3000)
  (send *robot* :reset-pose)
  (send *robot* :fix-leg-to-coords (make-coords))
  (setq *door* (instance param-door :init 700 60 100 20
                         (float-vector 0 -640 950)
                         (float-vector -60 -600 950)
                         :use-leg t
                         :handle-l/r :right))
  (setq *traj* (door-manipulation-trajectory *door* :robot :jaxon))
  (setq *recog-result* (instance arrow-object :init))
  (objects (list *robot* *traj* *door* *recog-result*))
  (unless (boundp '*col*)
    (send *robot* :reset-manip-pose)
    (setq *col* (instance collision-link-parameter :init *robot*))
    (send *col* :init-distance-table))
  (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
  (send *ri* :start-impedance :arms :m-p 10 :d-p 800 :k-p 300 :m-r 2.0 :d-r 80 :k-r 40)
  (setq *ik-solver* (instance ik-solver
                              :init *robot*
                              :rarm
                              :ik-type :use-torso
                              :move-target (get (send *robot* :rarm :end-coords) :door-manipulation-move-target)
                              :trajectory *traj*
                              :collision-parameter *col*
                              :prepare-angle-vector (send *robot* :angle-vector)))
  (ros::advertise "trajectory_marker" visualization_msgs::Marker)
  (ros::unsubscribe "/valve_detection/icp_registration/output_pose")
  (ros::subscribe "/door_detection/door_handle"
                  geometry_msgs::PoseStamped
                  #'test-door-recognition-callback)
  ;; 1. set octree resolution
  
  (ros::ros-info "set octree resolution")
  (ros::set-dynamic-reconfigure-param "/door_detection/octree_voxel_grid" "resolution" :double (* 0.001 dx))
  ;; 2. clear cache
  (ros::ros-info "clearing laser cache")
  (bench
   (setq *perception-succeeded-time* (ros::time-now))
   (call-empty-service "/tilt_laser_listener/clear_cache")
   ;; 3. wait for recognition
   (ros::ros-info "waiting for recognition")
   (wait-for-recognition)
   (ros::ros-info "done"))
  (bench
   (ros::ros-info "planning motion")
   (setq *avs* (send *ik-solver* :solve
                     :trajectory-resolution trajectory-num
                     :stop 50
                     :collision-num collision-num :debug-view nil))
   (ros::ros-info "done"))
    (bench
   (if *avs*
       ;; 5. exec motion
       (progn
         (let ((tm (send *avs* :send-angle-vector-sequence-test *ri* *robot* speed-factor)))
           (ros::ros-info "execution: ~A" tm))
         (send *ri* :wait-interpolation))
       ;;nil
     (ros::ros-warn "Failed to solve ik")))
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (send *robot* :rarm :move-end-pos (float-vector 500 0 0))
      (robot-move-height *robot* 100)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    (let ((foot-midcoords (send *robot* :foot-midcoords)))
      (send *robot* :reset-manip-pose)
      (send *robot* :fix-leg-to-coords foot-midcoords))
    (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
    (send *ri* :wait-interpolation)
    )

;; (load "experiment.l")
