# Chessboard calibration(one kinect)

- 2018/3/12
- no need kinect can run
- no socket code
- code在144行就結束。

- 後面是測試棋盤3D座標經過extrinsic matrix([R|T]矩陣)轉換後看看可不可以投回去原來的corner上面。有兩種方法，一種是棋盤座標經過[R|T]轉換後call opencv的projectPoints()，R vector跟T vector都丟0，他就會根據我們求出來的intrinsic跟distortion去算出相對於camera的2D座標，這種很簡單只要一行式子就幫你把camera space(3D)轉到image space(2D)。  另一種是棋盤座標經過[R|T]轉換後，自己算出3D轉2D的過程，把intrinsic跟distortion算進去。

- 直接執行即可，會產生兩個txt，一個是intrinsic.txt一個是distor.txt
主要是透過Capture_some_chessboard_images這個程式會產生n張照片來calibration
