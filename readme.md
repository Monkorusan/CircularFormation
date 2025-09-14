# 結合位相振動子のダイナミクスで駆動されるロボット群の円形フォーメーション制御

## 理論

円形フォーメーション形成するために、ロボットが追従すべきベクトル場は

$$\frac{dr_i}{dt} = r_i\left(1-\frac{r_i^2}{R_i^2}\right)$$
$$ \frac{d\phi_i}{dt} = \Omega + \varepsilon\sin(\phi_{i+1} - \phi_i+B_i) $$

と定義する。ここで、上式は、Stuart-Landau発振器を簡略化したものであり、下式は、蔵本モデルを簡略化したもの（全結合型を一方向結合型に緩和したもの）である。ただし、 $B_{i}$ は、位相のバイアスを表す。
次に、２輪移動型ロボット $i$ のダイナミクスは

$$\begin{bmatrix}
\dot{r}_i \\
r_i\dot{\phi}_i \\
\dot{\theta}_i
\end{bmatrix}=
\begin{bmatrix}
\cos\theta_i & 0 \\
\sin\theta_i & 0 \\
\frac{1}{r_i}\sin\theta_i & 1
\end{bmatrix}
\begin{bmatrix}
v_i \\
\omega_i
\end{bmatrix}$$

で与えられる。2輪移動型ロボットを図のように与えると、ベクトル場に追従するための制御入力は
$$
v_i = k_v \left( \frac{dr_i}{dt} \cos\theta_i + r_i \frac{d\phi_i}{dt} \sin\theta_i \right)
$$
$$
\omega_i = k_\omega \left( r_i \frac{d\phi_i}{dt} \cos\theta_i - \frac{dr_i}{dt} \sin\theta_i \right)
$$
と導出される。

結果：結合項においてバイアスを導入することで、クラスタを組むTrailing円形フォーメーション形成が可能となった。
欠点：衝突回避は考慮されていない。

次のtodo（導入すべき事項）:

- ワールド座標系とロボットの位置を示す座標系をTF2で表現
- 制約に基づく制御（簡単な二次計画法でもよい）

## 実行方法
### requirements
- ROS2 Humble

### how2run (for the first time)
<pre>
mkdir ros2_ws
source /opt/ros/humble/setup.bash
cd ros2_ws
git clone git@github.com:Monkorusan/CircularFormation_CoupledOscillators.git
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch TODO TODO.launch.py</pre>

