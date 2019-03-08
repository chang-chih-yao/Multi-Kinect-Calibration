# Multi-Kinect-Calibration

- 開發平台：Win 10
- 開發裝置：Microsoft Kinect 2
- 開發工具：Visual Studio 2013

***

由於 Kinect 透過深度攝影機從人抓到3D骨架，是有它的限制的，
如果發生遮蔽，那麼發生遮蔽的那些關節點都會很不準，為了解決這個問題，我們就用兩台kinect擺放位置差不多夾角90度。

透過socket傳輸彼此的關節點座標給另一台。
但由於兩台kinect都不知道彼此相對位置，所以需要先校正(calibration)

Two Kinect Calibration(透過讓兩台kinect看到同一個棋盤去校正)：
- Use depth camera(use body joints) -> Calculate extrinsic Mat
- Use color camera(RGB image)       -> Chessboard calibration
- ICP

Use depth camera：

![image](https://github.com/chang-chih-yao/Multi-Kinect-Calibration/blob/master/depth%20camera.JPG)

此圖是根據5個3D點去做校正，紅色是原本的kinect(server)，綠色是另一台kinect透過extrinsic matrix處理後得到的結果。

Use color camera：

![image](https://github.com/chang-chih-yao/Multi-Kinect-Calibration/blob/master/chessboard_calibration_1.JPG)


![image](https://github.com/chang-chih-yao/Multi-Kinect-Calibration/blob/master/chessboard_calibration_2.JPG)
