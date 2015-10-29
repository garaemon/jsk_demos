#!/usr/bin/env roseus

(load "package://drc_task_common/euslisp/robot-util.l")
(load "package://drc_task_common/euslisp/door-open/subscribe_checkerboard.l")

(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")

(setq *robot-name* (unix::getenv "ROBOT"))
(load (robot-file (unix::getenv "ROBOT")))
(init-robot-from-name (unix::getenv "ROBOT"))

(objects (list *robot*))
;; (send *ri* :set-joint-angles-of-group "head_for_teleop" 
;;       (coerce (mapcar #'deg2rad (coerce (send *robot* :head :angle-vector) cons)) float-vector) 
;;       4)
(defun result-callback (msg)
  (if (> (length (send msg :boxes)) 0)
      (let ((box (elt (send msg :boxes) 0)))
        (if (send *tf-listener* :wait-for-transform "BODY" (send box :header :frame_id) (send box :header :stamp)
                  0.1)
            (let* ((coords (send *tf-listener* :lookup-transform "BODY"
                                 (send box :header :frame_id) (send box :header :stamp)))
                   (box-local-coords (ros::tf-pose->coords (send box :pose)))
                   (box-global-coords (send coords :transform box-local-coords)))
              (send *robot* :head :look-at (send box-global-coords :worldpos))
              (send *robot* :head :neck-p :joint-angle -20 :relative t)
              (send *irtviewer* :draw-objects)
              (x::window-main-one)
              (send *ri* :set-joint-angles-of-group "head_for_teleop" 
                    (coerce (mapcar #'deg2rad (coerce (send *robot* :head :angle-vector) cons)) float-vector) 
                    1)
              (print 'wait-interpolation)
              (send *ri* :wait-interpolation-of-group "head_for_teleop")
              )))
    ))

(setq *tf-listener* (instance ros::transform-listener :init))

(send *ri* :add-joint-group "head_for_teleop" (send-all (send *robot* :head :joint-list) :name))
(ros::subscribe "/plane_supported_cuboid_estimator/output/result" jsk_recognition_msgs::BoundingBoxArray #'result-callback)
(ros::spin)

