(load "package://pr2eus/pr2-interface.l")

(defun move-vertical(d &optional (vel 80)) ;; 300mm 150mm/s 2000ms
  (setq time (* 1000 (/ d vel)))
  (setq target-coords (send (send *pr2* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 d) :world))
  (send *pr2* :rarm :inverse-kinematics target-coords :thre 0.01 :rthre (deg2rad 0.01))
  (send *ri* :angle-vector (send *pr2* :angle-vector) time :rarm-controller 0)
  (send *ri* :wait-interpolation)
  )

(pr2-init)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(setq target-coords (send *pr2* :rarm :end-coords :copy-worldcoords))
(send target-coords :replace-rot (rpy-matrix 0 (deg2rad 90) 0))
(send *pr2* :rarm :inverse-kinematics target-coords :thre 0.01 :rthre (deg2rad 0.01))
(send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :rarm-controller)
