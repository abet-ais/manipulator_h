manipulator_hで動かしたファイルなどをここに入れる


・manipulator_hをシミュレーションにて動作させる方法
1. roscd manipulator_h_manager/launch 
2. 4行目のvalueをtrueにする
3. roslaunch manipulator_h_gazebo manipulator_h_gazebo.launch
4. gazeboウィンドウ内左下の再生ボタンを押す
5. roslaunch manipulator_h_manager manpulator_h_manager.launch
6. roslaunch manipulator_h_moveit_config demo.launch


・manipulator_hを実機にて動作
1. roscd manipulator_h_manager/launch 
2. 4行目のvalueをfalseにする
3. 緊急停止をすぐに押せる場所におく
4. 緊急停止を解除し、電源を入れる
5. sudo bash
6. source devel/setup.bash
7. roslaunch manipulator_h_manager manipulator_h_manager.launch
8. roslaunch manipulator_h_moveit_config demo.launch


・ソースの実行
1. roscd manipu_h_motion/scripts
2. A.pyスクリプトを作り，
   from send_orbit_to_gazebo_ import Manipulator_h　をインポート
3. chmod +x A.py
4. python A.py

