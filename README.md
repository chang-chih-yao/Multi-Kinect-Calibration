# Multi-Kinect-Calibration

- 開發工具：VS 2013
- 開發平台：Win 10

由於 Kinect 透過深度攝影機從人抓到3D骨架，是有它的限制的，
如果發生遮蔽，那麼發生遮蔽的那些關節點都會很不準，為了解決這個問題，我們就用兩台kinect擺放位置差不多夾角90度。

透過socket傳輸彼此的關節點座標給另一台。
但由於兩台kinect都不知道彼此相對位置，所以需要先校正(calibration)
