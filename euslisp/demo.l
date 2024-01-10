(load "models/arrow-object.l")
(load "package://pr2_fingertip_sensors/euslisp/place-hand.l")
(load "package://pr2_fingertip_sensors/euslisp/touch-hand.l")
(load "package://pr2_fingertip_sensors/euslisp/stroke.l")
(load "package://pr2_fingertip_sensors/euslisp/grasp.l")

(ros::roseus "demo")
(pr2-init)
(pr2-tuckarm-pose :free-arm :rarm)
(send *pr2* :torso_lift_joint :joint-angle 182)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :torso-controller 0 :minjerk-interpolation t))
(rarm-init-pose)
(unix::sleep 10)

;; 顔を向ける
(look-at-hand :wait nil)

;; 手を見つける
(setq target (search-hand 15))
(setq *a* (arrow))
(send *a* :newcoords target)
(objects (list *pr2* *a*))

;; 手を近づける
(send *ri* :start-grasp)
(place-hand target 250)
(setq result (touch-hand-translate 250))

;;接触するまでもう片方に傾ける
(when (eq result :left)
(touch-hand-rotate "r_fingertip"))
(when (eq result :right)
(touch-hand-rotate "l_fingertip"))

;; 離れるまで撫でる
(stroke)

;; グリッパが掴まれたら開く
(wait-grasp)
(send *ri* :stop-grasp :rarm)

;; 強く握られたらグリッパを狭める
(hold-back)
