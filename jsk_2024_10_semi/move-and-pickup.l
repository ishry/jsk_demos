;;実行前に一度 $rosservice call /look_forward_in_navigation/stop を実行することで、移動時に視線を保つことができる

(require "package://pr2eus/pr2-interface.l")
(require "package://pr2eus/pr2.l")
(load "package://pr2eus/pr2-interface.l")
(ros::roseus "pr2_send_joints")
(ros::load-ros-manifest "jsk_recognition_msgs")


;;検知したboxとラベルを対応させるクラス
(defclass box-label-synchronizer
  :super exact-time-message-filter
  :slots (target-label-list)
  )

(defmethod box-label-synchronizer
  (:callback (box-msg label-msg)
    (print (list box-msg label-msg))
    (print (send-all (list box-msg label-msg) :header :stamp))
    (box-cb box-msg label-msg target-label-list)
    ))

(defmethod box-label-synchronizer
  (:set-target-label-list (label-list)
			  (setq target-label-list label-list)))

;;コールバック関数
(defun box-cb (box-msg label-msg label-list)
  (ros::ros-info "received ~A boxes, ~A labels" (length (send box-msg :boxes)) (length (send label-msg :labels)))
  (dolist (msg-conbined (map cons #'(lambda (x y) (list  x y)) (send box-msg :boxes) (send label-msg :labels)))
    (let (box label)
      (setq box (car msg-conbined) label (cadr msg-conbined))
      (print (send label :name))
      (when (contains label-list (send label :name)) 
	(setq *target-coords* (send (ros::tf-pose->coords (send box :pose)) :copy-worldcoords))
	(setq *target-dimensions* (send box :dimensions))
	(format t "coords ~A, dimension ~A~%" (send *target-coords* :worldcoords) (* (send *target-dimensions* :x) (send *target-dimensions* :y) (send *target-dimensions* :z)))
	(when (and (< (elt (send *target-coords* :worldpos) 2) 400)
		   (> (elt (send *target-coords* :worldpos) 2) -400))
	  (send *target-box* :move-to *target-coords* :world)
	  (print "update target position")
	  )))))

;;listにwordが含まれていればt,含まれていなければnilを返す関数
(defun contains (list word)
  (let ((tmp nil))
    (while list
      (if (string= word (car list)) 
          (progn (setq tmp t) (return))
	(setq list (cdr list))))
    tmp))


(defun init-pose ()
  (send *pr2* :reset-pose)
  (send *ri* :stop-grasp :arms)
  (send *irtviewer* :draw-objects) 
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (pr2-tuckarm-pose)
  (send *irtviewer* :draw-objects) 
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation))

(defun tuck-pose ()
  (pr2-tuckarm-pose)
  (send *irtviewer* :draw-objects) 
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation))
  

(defun detect-socks-pos (pos) 
  (send *target-box* :locate #f(-10000 -10000 -10000) :world) ;;*target-box*をリセット
  (ros::ros-info "start waiting for target ... ~A" (send *target-box* :worldpos))

  ;;posを向く
  (send *pr2* :head :look-at (relative-to-world-pos pos))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (when isDebug 
    (send *target-box* :locate (v- (send *socks* :worldpos) (send *pr2* :worldpos))  :world))
  (ros::rate 10) ;; 10Hzで探索
  (while (or (> (elt (send *target-box* :worldpos) 2) 400)
	     (< (elt (send *target-box* :worldpos) 2) -400))
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector)) ;;?
    (send *irtviewer* :draw-objects)
    (x::window-main-one) ;;?
    (ros::spin-once)
    (ros::ros-info "waiting... ~A" (send *target-box* :worldpos))
    (ros::sleep))
  (ros::ros-info "targer found! ... ~A" (send *target-box* :worldpos)))


;;*pr2*が移動した分だけposやcoordsに補正をかける
(defun relative-to-world-pos (pos)
  (v+ (send *pr2* :worldpos) pos))

(defun relative-to-world-coords (coords)
  (send coords :translate (send *pr2* :worldpos)))
  
 
(defun pick-up (pos label-list)
  (ros::ros-info "pick-up start")

  ;;ローカル変数の初期化
  (if (>(elt pos 1) 0) (setq _arm :larm) (setq _arm :rarm))
  (if (>(elt pos 1) 0) (setq _y 50) (setq _y -50)) ;;中央に持っていくときに使用
  (if (>(elt pos 1) 0) (setq _theta -90) (setq _theta 90)) ;;中央に持っていくときに使用

  ;;posを見る
  (send *pr2* :head :look-at (relative-to-world-pos pos))
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  ;;subscribe 
  (setq box-sync (instance box-label-synchronizer :init
			   (list (list "/docker/detic_segmentor/output/boxes" jsk_recognition_msgs::BoundingBoxArray)
				 (list "/docker/detic_segmentor/detected_classes" jsk_recognition_msgs::LabelArray))))

  ;;検知したい物体のリストを登録
  (send box-sync :set-target-label-list label-list) 

  ;;物体検知
  (setq *target-box* (make-cube 100 100 100 :pos #f(-10000 -10000 -10000))) ;;*target-box*を初期化
  (detect-socks-pos pos)
  
  ;;ik解けるか確認
  (setq ik (send *pr2* _arm :inverse-kinematics 
	(send (send (relative-to-world-coords (send *target-box* :copy-worldcoords)) :translate #f(0 0 -25)) :rotate (deg2rad 90) :y)
	:rotation-axis nil
	:debug-view nil))

  ;;ik解けなければ移動
  (while (not ik)
    (ros::ros-info "ik failed. start moving")
    (let ((target-x (elt (send *target-box* :worldpos) 0))
	  (target-y (elt (send *target-box* :worldpos) 1))
	  (x 0)
	  (y 0))
      (when (> target-x 250) (setq x (- target-x 250)))
      (setq y (cond ((> target-y 550) (- target-y 550))
		    ((< target-y -550) (+ target-y 550))
		    (t 0)))
      (send *ri* :clear-costmap) ;;障害物のリセット
      (send *ri* :go-pos (/ x 1000) (/ y 1000) 0) ;;go-posのみ単位がメートル ;; :go-pos-unsafeとすると障害物は無視するが、位置精度が悪い
      (send *pr2* :translate (float-vector x y 0))
      (ros::ros-info "moved #f(~A ~A 0)" x y))

    ;;ik更新
    (detect-socks-pos pos)
    (setq ik (send *pr2* _arm :inverse-kinematics 
		   (send (send (relative-to-world-coords (send *target-box* :copy-worldcoords)) :translate #f(0 0 -25)) :rotate (deg2rad 90) :y)
		   :rotation-axis nil
		   :debug-view nil)))
  
  ;;ik完了
  (ros::ros-info "ik succeeded. start glasping at ~A~%" (send *target-box* :copy-worldcoords)) 

  ;;上から靴下を掴む軌道の生成  
  (send *pr2* _arm :inverse-kinematics
	(send (send (relative-to-world-coords (send *target-box* :copy-worldcoords)) :translate #f(0 0 300)) :rotate (deg2rad 90) :y)
	:rotation-axis :x  
	:debug-view nil)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* _arm :inverse-kinematics
	(send (send (relative-to-world-coords (send *target-box* :copy-worldcoords)) :translate #f(0 0 100)) :rotate (deg2rad 90) :y)
	:rotation-axis :x 
	:debug-view nil)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* _arm :inverse-kinematics
	(send (send (relative-to-world-coords (send *target-box* :copy-worldcoords)) :translate #f(0 0 -25)) :rotate (deg2rad 90) :y)
	:rotation-axis nil
	:debug-view nil)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
      
  ;;掴む
  (send *ri* :start-grasp _arm :wait t) ;;graspとspeakは:wait t が必要
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *pr2* _arm :inverse-kinematics
	(send (send (relative-to-world-coords (send *target-box* :copy-worldcoords)) :translate #f(0 0 300)) :rotate (deg2rad 90) :y)
	:rotation-axis nil
	:look-at-target t
	:debug-view nil)
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation))


(defun connect-objects ()
  ;;腕を中央へ
  (send *pr2* :rarm
	:inverse-kinematics(send (send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 400.0 -50 400.0)) :rotate (deg2rad 90) :z )
	:rotation-axis :x
	:look-at-target t
	:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm
	:inverse-kinematics(send (send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 400.0 50 400.0)) :rotate (deg2rad -90) :z )
	:rotation-axis :x
	:look-at-target t
	:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ;;靴下をくっつける
  (send *pr2* :larm
	:inverse-kinematics(send (send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 400.0 20 400.0)) :rotate (deg2rad -90) :z )
	:rotation-axis :x
	:look-at-target t
	:debug-view nil)

  (send *pr2* :rarm
	:inverse-kinematics(send (send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 400.0 -20 400.0)) :rotate (deg2rad 90) :z )
	:rotation-axis :x
	:look-at-target t
	:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ;;左手を離す
  (send *ri* :stop-grasp :larm :wait t) ;;graspとspeakは:wait t が必要
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm
	:inverse-kinematics(send (send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 400.0 100 400.0)) :rotate (deg2rad -90) :z )
	:rotation-axis :x
	:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation))


(defun throw-in ()
  ;;右腕の移動
  (send *pr2* :rarm
	:inverse-kinematics(send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 600.0 -10 1000.0))
	:rotation-axis :z
	:look-at-target t
	:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :rarm
	:inverse-kinematics(send (send *pr2* :copy-worldcoords)
				       :translate (float-vector 800.0 -10 1000.0))
	:rotation-axis :z
	:look-at-target t
	:debug-view nil)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ;;物体を離す
  (send *ri* :stop-grasp :rarm :wait t) ;;graspとspeakは:wait t が必要
  (send *irtviewer* :draw-objects)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
  (send *ri* :wait-interpolation))


;;メイン処理;;
(if (not (boundp '*pr2*)) (pr2-init))
(setq *ri* (instance pr2-interface :init))
(objects (list *pr2*))

(setq isDebug t)

(init-pose)
(when isDebug (setq *socks* (make-coords :pos #f(400 600 70))))
(pick-up #f(200 400 70) (list "matchbox" "toy" "kite"))
(when isDebug (setq *socks* (make-coords :pos #f(500 -800 70))))
(pick-up #f(200 -400 70) (list "matchbox" "toy" "kite"))
(connect-objects)
(throw-in)
(tuck-pose)
