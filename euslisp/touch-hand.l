(load "package://pr2eus/pr2-interface.l")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "control_msgs")

(defun l_touch_cb (msg)
  (ros::ros-info "~A" (send msg :data))
  (when (eq (send msg :data) 1)
    (setq *l_touch* t)))

(defun r_touch_cb (msg)
  (ros::ros-info "~A" (send msg :data))
  (when (eq (send msg :data) 1)
    (setq *r_touch* t)))

(defun touch_cb (msg)
  (ros::ros-info "~A" (send msg :data))
  (when (eq (send msg :data) 1)
    (setq *touch* t)))

(defun touch-hand-translate (distance)
  (let ((fingertips (list "l_fingertip" "r_fingertip")) result
        (controller-name "/r_arm_controller/follow_joint_trajectory"))
    (dolist (fingertip fingertips)
      (setq topic-name (format nil "/pfs/r_gripper/~A/pfs_b_top/touch_state" fingertip))
      (if (string-equal fingertip "l_fingertip")
          (ros::subscribe topic-name std_msgs::UInt8 #'l_touch_cb)
        (ros::subscribe topic-name std_msgs::UInt8 #'r_touch_cb)))
    (ros::subscribe (format nil "~A/result" controller-name)
                    control_msgs::FollowJointTrajectoryActionResult
                    #'(lambda (msg) (setq result t)))
    (setq *l_touch* nil)
    (setq *r_touch* nil)    
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq end-coords (send *pr2* :rarm :end-coords :copy-worldcoords))
    (setq target-coords (send (send end-coords :copy-worldcoords) :translate (float-vector 0 0 (+ (- distance) 15)) :world))
    (setq touched nil)
    (unless (null (send *pr2* :rarm :inverse-kinematics target-coords :rotation-axis :x :revert-if-fail nil))
      (setq result nil)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 7000 :rarm-controller 0 :minjerk-interpolation nil)
      (loop
       (when *l_touch*
         (if *r_touch*
             (setq touched :both)
           (setq touched :left))
         (ros::ros-info "touch")
         (send *ri* :cancel-angle-vector :controller-type :rarm-controller)
         (return))
       (when *r_touch*
         (setq touched :right)
         (ros::ros-info "touch")
         (send *ri* :cancel-angle-vector :controller-type :rarm-controller)
         (return))
       (when result
         (ros::ros-info "get result")
         (return))
       (ros::spin-once)))
    (dolist (fingertip fingertips)
      (setq topic-name (format nil "/pfs/r_gripper/~A/pfs_b_top/touch_state" fingertip))
      (ros::unsubscribe topic-name))
    (ros::unsubscribe (format nil "~A/result" controller-name))
    touched
    ))

(defun touch-hand-rotate (fingertip)
  (setq topic-name (format nil "/pfs/r_gripper/~A/pfs_b_top/touch_state" fingertip))
  (setq controller-name "/r_arm_controller/follow_joint_trajectory")
  (setq *touch* nil)
  (ros::subscribe topic-name std_msgs::UInt8 #'touch_cb)
  (ros::subscribe (format nil "~A/result" controller-name)
                  control_msgs::FollowJointTrajectoryActionResult
                  #'(lambda (msg) (setq result t)))
  
  (setq end-coords (send *pr2* :rarm :end-coords :copy-worldcoords))
  (if (string-equal fingertip "l_fingertip")
      (send end-coords :rotate (deg2rad -10) :z :local)
    (send end-coords :rotate (deg2rad 10) :z :local))
  (send *pr2* :rarm :inverse-kinematics end-coords :rotation-axis :t)
  (setq result nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :rarm-controller 0 :minjerk-interpolation nil)
  (loop
   (when *touch*
     (send *ri* :cancel-angle-vector :controller-type :rarm-controller)
     (return))
   (when result
     (ros::ros-info "get result")
     (return))
   (ros::spin-once)
   )
  )

