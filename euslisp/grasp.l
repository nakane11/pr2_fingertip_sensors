(load "package://pr2eus/pr2-interface.l")
(ros::roseus-add-msgs "std_msgs")

(defun grip-cb (msg)
  (setq *grip-state* (send msg :data)))

(defun wait-grasp()
  (ros::ros-info "waiting for grasped...")
  (setq topic-name "/pfs/r_gripper/grip_state")
  (setq *grip-state* nil)
  (ros::subscribe topic-name std_msgs::String #'grip-cb)
  (loop
   (when (string-equal *grip-state* "hoge")
     (return))
   (ros::spin-once))
  (ros::unsubscribe topic-name))

(defun force-cb (msg)
  (setq *force-state* (send msg :data)))

(defun hold-back()
  (setq topic-name "/pfs/r_gripper/force_state")
  (setq *force-state* nil)
  (ros::subscribe topic-name std_msgs::String #'force-cb)
  (loop
   (when (string-equal *force-state* "s")
     (ros::ros-info "s")
     ;; あとで接触するまでに変える
     (send *ri* :start-grasp)
     (unix::sleep 10))
   (when (string-equal *force-state* "w")
     (ros::ros-info "s")
     (send *ri* :stop-grasp)
     (unix::sleep 10))
   (ros::spin-once)
   )
  (ros::unscribe topic-name))
