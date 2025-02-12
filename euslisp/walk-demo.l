(load "package://pr2_fingertip_sensors/euslisp/place-hand.l")
(ros::roseus-add-srvs "pr2_fingertip_sensors")

;; 初期姿勢
(ros::roseus "demo")
(pr2-init)
(ros::wait-for-service "/hand_pose_server/start_holding")
(ros::wait-for-service "/hand_pose_server/stop_holding")
(pr2-tuckarm-pose :rarm)
(send *pr2* :torso_lift_joint :joint-angle 325)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :torso-controller 0 :minjerk-interpolation t))

(send *ri* :speak "Welcome! Thanks for dropping by." :wait t)
(send *ri* :speak "I'll guide you to our lab." :wait t)

;; 手を探す
(send *pr2* :head_tilt_joint :joint-angle 20)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :head-controller 0 :minjerk-interpolation nil))

(setq target (send *pr2* :rarm :end-coords :copy-worldcoords))
(send target :translate (float-vector 900 -30 100) :world))

;; 手の位置に対する相対位置を計算して手を指しだす
(send target :translate (float-vector -450 -100 -40) :world)
(send target :replace-rot (rpy-matrix 0 (deg2rad 80) 0))
(send *pr2* :rarm :inverse-kinematics target :rotation-axis :t)
(send *ri* :angle-vector (send *pr2* :angle-vector) 4000 :rarm-controller 0 :minjerk-interpolation t)

(send *ri* :speak "Wanna hold hands so you don't get lost?" :wait t)
;; 人からの手繋ぎ
(setq req (instance pr2_fingertip_sensors::HandPoseRequest :init))
(send req :continuous nil)
(send req :reset t)
(setq res (ros::service-call "/hand_pose_server/start_holding" req))
(ros::ros-info "~A" (send res :handpose))

(unix::sleep 1)

(send *ri* :speak "Hold on, okay? Let's go!" :wait t)

;; 現在の繋ぎ方と進行方向に応じて腕と台車を回転させる
(setq next-base-angle 200) ;;baseの回転量
(setq next-base-to-arm-angle -80) ;;baseに対するarmの角度
(setq base-to-arm-angle (send *pr2* :r_shoulder_pan_joint :joint-angle))

(when (> next-base-angle 0)
  (setq arm-move-angle1 (* 0.7 next-base-to-arm-angle))
  (setq arm-move-angle2 next-base-to-arm-angle)
  (setq base-move-angle1 (- base-to-arm-angle next-base-to-arm-angle))
  (setq base-move-angle2 (- next-base-angle base-move-angle1)))

(when (< next-base-angle 0)
  (setq arm-move-angle1 base-to-arm-angle)
  (setq arm-move-angle2 next-base-to-arm-angle)
  (setq base-move-angle1 next-base-angle)
  (setq base-move-angle2 0))

(send *pr2* :r_shoulder_pan_joint :joint-angle arm-move-angle1)
(setq tmp (send *pr2* :r_wrist_roll_joint :joint-angle))
(send *pr2* :r_wrist_roll_joint :joint-angle (- tmp arm-move-angle1))
;; 腕を縮める
(setq target (send *pr2* :rarm :end-coords :copy-worldcoords))
(setq a (send target :pos))
(send target :locate (float-vector (/ (elt a 0) 1.3) (/ (elt a 1) 1.3) (elt a 2)) :world)
(send *pr2* :rarm :inverse-kinematics target :rotation-axis :t :revert-if-fail t)

(setq av1 (send *pr2* :angle-vector))

(send *pr2* :r_shoulder_pan_joint :joint-angle arm-move-angle2)
(setq tmp (send *pr2* :r_wrist_roll_joint :joint-angle))
(send *pr2* :r_wrist_roll_joint :joint-angle (- tmp (* 0.8 arm-move-angle2)))
(setq av2 (send *pr2* :angle-vector))

(send *ri* :move-trajectory-sequence
  (list (float-vector 0 0 (deg2rad base-move-angle1))
        (float-vector 0 0 (deg2rad base-move-angle2)))
  (list 4000 4000)
:send-action t :wait nil)

(send *ri* :angle-vector-sequence (list av1 av2) (list 4000 4000)
        :rarm-controller)
(send *ri* :wait-interpolation)


;;開始地点まで移動
;; (send *ri* :go-pos-unsafe 0.6 0 0)
;; (setq a (send (send *ri* :state :worldcoords) :copy-worldcoords))
(setq b (make-coords :pos #f(-39.768 -28797.515 20000.0) :rpy #f(1.907 0.0 0.0)))
(send *ri* :move-to b)

;;部屋の前まで移動
(send *ri* :go-pos-unsafe 10.0 -0.2 0)
(send *ri* :go-pos-unsafe 1.0 -0.2 0)
(send *ri* :go-pos-unsafe 0.0 0.0 -60)
(send *ri* :go-pos-unsafe 1.0 0.0 0)

;; (send *ri* :clear-costmap)

(send *ri* :speak "Here is JSK Lab." :wait t)

(setq stop-req (instance pr2_fingertip_sensors::HandPoseRequest :init))
(setq res (ros::service-call "/hand_pose_server/stop_holding" stop-req))
