# ROS2のインストールと概要(r2cv-ros2)
ROS2 Foxyのインストールと概要を紹介します。

## 目次
[1. ROS2 Foxyのインストール](#1)

[2. ワークスペースの作成](#2)

[3. パッケージの作成](#3)

[4. なぜROSか?](#4)

[5. ROS2を体感する。](#5)

[<R2CVに戻る>](https://github.com/nishibra/r2cv-1)

<a id="1"></a>
## 1. ROS2 Foxyのインストール
>参考サイト
https://index.ros.org/doc/ros2/Installation/

ROS2 Foxyデスクトップのインストール
以下のコマンドを実行するとROS2 Foxy Desktopをインストールできます。
```
$ sudo apt update
$ sudo apt install curl gnupg2 lsb-release
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
$ sudo apt update
$ export ROS_DISTRO=foxy
$ sudo apt install ros-$ROS_DISTRO-desktop
$ sudo apt install python3-colcon-common-extensions
$ sudo apt install python3-rosdep
$ sudo apt install python3-argcomplete
$ sudo rosdep init
$ rosdep update
$ source /opt/ros/foxy/setup.bash
$ source ~/.bashrc
```
<a id="2"></a>
## 2. ワークスペースの作成
ROS2のインストールが終了したらまず作業領域を作成します。
```
$ mkdir -p ~/ros2ws/src
$ cd ~/ros2ws/src
```
<a id="3"></a>
## 3. パッケージの作成
続いてパッケージを作成します。ここではパッケージ名をr2cvrなどとします。
```
$ cd ~/ros2ws/src
$ ros2 pkg create --build-type ament_python r2cvr
$ cd ~/ros2ws
```
このディレクトリーでビルトします。
```
$ colcon build
```
 以下の行の入力し、パッケージの設定を反映しプログラムを走らせることができます。
```
$ . ~/ros2ws/install/setup.bash
```
## 4. bashrcの設定
パッケージの作成が終わったら、bashrcに以下を追記しておくと便利です。画面起動でsetupができます。また、aliasを設定しておけば簡易にcbと打てばcolcon buildが実行できます。
```
$ sudo gedit .bashrc
```

> bashrcに追記
```
source /opt/ros/foxy/setup.bash
source ~/ros2ws/install/setup.bash
alias cw='cd ~/ros2ws'
alias cs='cd ~/ros2ws/src'
alias cb='cd ~/ros2ws && colcon build'
```


<a id="4"></a>
## 4. なぜROSか?
ロボットはいろいろな仕事をしながらそれぞれが連携して動く必要があります。そのためプログラム間で情報のやり取りを行います。これがプロセス間通信です。マルチタスク化が進み、分担して開発が進めせれるようになってますますROSの必要性が増してた来ました。さらに高速化、リアルタイム性が求められるようになり、ROS2への移行が進められています。

![023](/pics-ros2/image023.png)

### ROSの構造
送りっぱなしの「Topic」と、送ったあと相手から応答を受け取る「service」で通信が行われます。

![024](/pics-ros2/image024.png)

### 分散開発
これによって分散開発が可能となりました。いろいろなプログラミング言語にも対応しています。ここではPython3使用します。

![025](/pics-ros2/image025.png)


<a id="5"></a>
## 5. ROS2を体感する。
### カメラ画像の取り込み
まずプロジェクトのディレクトリーにあるsetup.pyの設定をしておきます。
```
ubuntu@ubuntu:~/ros2ws/src/r2cvr$
```
このsetup.pyに以下のように実行するプログラムを記述しておき、colocon buildします。
```
entry_points={
        'console_scripts': [
            'cam = ' + package_name + '.cam_pub:main',
        ],
```

前述のcam_sample.pyをROS2化ししたものがcam_pub.pyで、取り込んだカメラ画像を配信します。カメラを接続して以下を実行してください。
```
$ ros2 run r2cvr cam
```
で画像をPublishします。　

### rqtでカメラ画像を表示します。
新しいターミナルでrqtを立ち上げます。
```
$ rqt
```
PluginのVisualizationからImage viewを選択し、/img_topicを表示します。
カメラ画像を見ることができます。もう一台PCがあればROS2を同じネットワーク上で走らせておくと同様に表示できます。

### rosbag2で録画してみよう。
```
$ ros2 bag record -o all.bag -a
```
ですべてのTopicをall.bagに保存します。Ctl-Cで終了します。
再生は以下です。配信プログラムを停止し、以下を実行します。
```
$ ros2 bag play all.bag
```
rqtで保存した画像を確認することができます。
個々のTopicを保存する場合は以下です。この例ではimg_topicをtopic.bagに保存します。
```
$ ros2 bag record -o topic.bag /img_topic
```
### ros2 bag command
> info:    Print information about a bag to the screen
> 
> play:    Play back ROS data from a bag
> 
> record:  Record ROS data to a bag
> 
$ 同一ネットワーク内で複数ROS2を使用する
> $ export ROS_DOMAIN_ID=1

