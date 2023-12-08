(load "package://pr2eus/pr2-interface.l")
(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "control_msgs")

(defun touch-hand (distance)
  (let ((topic-name "touch_status") stop result
        (controller-name "/r_arm_controller/follow_joint_trajectory"))
    (ros::subscribe topic-name std_msgs::Bool
                    #'(lambda (msg)
                        (ros::ros-info "~A" (send msg :data))
                        (setq stop (send msg :data))))
    (ros::subscribe (format nil "~A/result" controller-name)
                    control_msgs::FollowJointTrajectoryActionResult
                    #'(lambda (msg) (setq result t)))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (setq end-coords (send *pr2* :rarm :end-coords :copy-worldcoords))
    (setq target-coords (send (send end-coords :copy-worldcoords) :translate (float-vector 0 0 (- distance)) :world))
    (unless (null (send *pr2* :rarm :inverse-kinematics target-coords :rotation-axis t))
      (setq result nil)
      (setq stop nil)
      (send *ri* :angle-vector (send *pr2* :angle-vector) 5000 :rarm-controller 0 :minjerk-interpolation t)
      (loop
       (when stop
         (ros::ros-info "cancel")
         (send *ri* :cancel-angle-vector :controller-type :rarm-controller)
         (return))
       (when result
         (ros::ros-info "get result")
         (return))
       (ros::spin-once)))
    (ros::unsubscribe topic-name)
    (ros::unsubscribe (format nil "~A/result" controller-name))
    ))