(load "package://pr2_fingertip_sensors/euslisp/place-hand.l")

(setq simulation-mode t)
;; 初期姿勢
(ros::roseus "demo")
(pr2-init)
(pr2-tuckarm-pose :free-arm :rarm)
(send *pr2* :torso_lift_joint :joint-angle 182)
(send *ri* :angle-vector (send *pr2* :angle-vector) 1500 :torso-controller 0 :minjerk-interpolation t))

;; 手を探す
(if simulation-mode
  (progn
    (setq target (send *pr2* :rarm :end-coords :copy-worldcoords))
    (send target :translate (float-vector 900 -30 100) :world))
  (setq target (search-hand 15)))

;; 手の位置に対する相対位置を計算して手を指しだす
(send target :translate (float-vector -450 10 20) :world)
(send target :replace-rot (rpy-matrix 0 (deg2rad 80) 0))
(send *pr2* :rarm :inverse-kinematics target :rotation-axis :t)
(send *ri* :angle-vector (send *pr2* :angle-vector) 4000 :rarm-controller 0 :minjerk-interpolation t)
;;;; グリッパを少し開く
(send *ri* :move-gripper :rarm 0.015)

;; 人からの手繋ぎ
(setq *start-holding* t)
;; 手繋ぎ完了
(setq *start-holding* nil)

;; 現在の繋ぎ方と進行方向に応じて腕と台車を回転させる
(setq next-base-angle 200) ;;baseの回転量
(setq next-base-to-arm-angle -80) ;;baseに対するarmの角度
(setq base-to-arm-angle (send *pr2* :r_shoulder_pan_joint :joint-angle))

(when (> next-base-angle 0)
  (setq arm-move-angle1 next-base-to-arm-angle)
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
(setq av1 (send *pr2* :angle-vector))

(send *pr2* :r_shoulder_pan_joint :joint-angle arm-move-angle2)
(setq tmp (send *pr2* :r_wrist_roll_joint :joint-angle))
(send *pr2* :r_wrist_roll_joint :joint-angle (- tmp arm-move-angle2))
(setq av2 (send *pr2* :angle-vector))

;; 内側の指を掴んでいた場合さらに回転させる

(send *ri* :move-trajectory-sequence
  (list (float-vector 0 0 (deg2rad base-move-angle1))
        (float-vector 0 0 (deg2rad base-move-angle2)))
  (list 3000 3000)
  :send-action t)
(send *ri* :angle-vector-sequence (list av1 av2) (list 3000 3000)
        :rarm-controller)

;; 移動開始
(send *ri* :go-pos 0.5 0 0)

;; 握り方を推定
;; (when *start-holding*
;;;; 安定しないとき
;;;; 間に指を入れたそうなとき
;;;; -> グリッパを開く
;;;; -> 力が加わったら握り終わったと見なす　握力検知
;;;; -> 握り返す　接触検知
;;;; 安定している

;; 特定の方向に引く
;;;; 力センサの値を各基板ごとにだす
;;;; 閾値より大きければ方向を計算してグリッパを回転させながら方向転換


