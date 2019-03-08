# Gradient joint

- 2018/3/12
- single kinect can run
- no socket code
- 300行的位置，file read是指要去讀取intrinsic matrix 跟 distortion matrix。至於這兩個file的產生是透過另外兩個cpp檔 : 
1. Capture_some_chessboard_images 
2. Chessboard_calibration
- 接好一台kinect，就可以開始執行程式。
需要讓他照到整個人辨識出骨架
接著他會去逼近兩個骨架的位置，最後畫面卡住就是已經最佳化完畢
