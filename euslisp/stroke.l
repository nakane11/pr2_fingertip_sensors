(load "package://pr2eus/pr2-interface.l")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "control_msgs")

(defun l_release_cb (msg)
  (ros::ros-info "~A" (send msg :data))
  (when (eq (send msg :data) 2)
    (setq *l-release* t)))

(defun r_release_cb (msg)
  (ros::ros-info "~A" (send msg :data))
  (when (eq (send msg :data) 2)
    (setq *r-release* t)))

(defun stroke-horizontal()
  (cond
   ((eq *stroke-state* 0)
    (setq target-coords (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 30) :local))
    (send *pr2* :rarm :inverse-kinematics target-coords)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :rarm-controller 0 :minjerk-interpolation t)
    (setq *stroke-state* 1))
   ((eq *stroke-state* 1)
    (setq target-coords (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 -60) :local))
    (send *pr2* :rarm :inverse-kinematics target-coords)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000 :rarm-controller 0 :minjerk-interpolation t)
    (setq *stroke-state* 2))
   ((eq *stroke-state* 2)
    (setq target-coords (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 60) :local))
    (send *pr2* :rarm :inverse-kinematics target-coords)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 4000 :rarm-controller 0 :minjerk-interpolation t)
    (setq *stroke-state* 1))
   )
  )

(defun stroke()
  (let ((fingertips (list "l_fingertip" "r_fingertip"))
        (controller-name "/r_arm_controller/follow_joint_trajectory"))
    (dolist (fingertip fingertips)
      (setq topic-name (format nil "/pfs/r_gripper/~A/pfs_b_top/touch_state" fingertip))
      (if (string-equal fingertip "l_fingertip")
          (ros::subscribe topic-name std_msgs::UInt8 #'l_release_cb)
        (ros::subscribe topic-name std_msgs::UInt8 #'r_release_cb)))
    (ros::subscribe (format nil "~A/result" controller-name)
                    control_msgs::FollowJointTrajectoryActionResult
                    #'(lambda (msg) (setq result t)))
    (setq *l-release* nil)
    (setq *r-release* nil)
    (setq *stroke-state* 0)
    (setq flg nil)
    (loop
     (ros::ros-info "1")
     (setq result nil)
     (stroke-horizontal)
     (send *ri* :wait-interpolation)
     (loop
      (when (and *l-release* *r-release*)
        (send *ri* :cancel-angle-vector :controller-type :rarm-controller)
        (setq flg t)
        (return))
      (when result
        (ros::ros-info "get result")
        (return))
      (ros::spin-once)
      )
     (when flg
       (return))
     )
    (dolist (fingertip fingertips)
      (setq topic-name (format nil "/pfs/r_gripper/~A/pfs_b_top/touch_state" fingertip))
      (ros::unsubscribe topic-name))
    (ros::unsubscribe (format nil "~A/result" controller-name))
))
  
