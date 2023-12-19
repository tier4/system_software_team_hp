=============
EKFの可観測性
=============

:著者: 石田 岳志

概要
====

2次元平面上のロボットの動作を拡張カルマンフィルタ(EKF)で推定すると数百ステップ程度で安定性が失われることが知られている。これは主にヨー角に対応する推定共分散が実際の共分散よりも小さくなってしまうことが主因とされている [Julier2001]_ [Bailey2006]_ 。ここでは [Huang2008]_ [Huang2009]_ を参考としてこの現象の要因をEKFの可観測性の観点から論じ、解決方法を記す。

システム方程式
==============

制御工学では一般に次のような方程式を扱う。

.. math::
   \dot{\mathbf{x}}(t) &= \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t)) \\
   \mathbf{y}(t) &= \mathbf{h}(\mathbf{x}(t))
   :label: nonlinear-system

| :math:`\mathbf{x}(t) \in \mathbb{R}^{n}` はシステムの状態であり、たとえばロボットの姿勢や速度などを記述する。システムの状態は一般に時間変化するため、時刻 :math:`t` の関数 :math:`\mathbf{x}(t)` として表される。 :math:`\dot{\mathbf{x}}(t)` は :math:`\mathbf{x}(t)` の時間微分を表す。一般に状態 :math:`\mathbf{x}(t)` は直接観測できないものとして扱われる。たとえばロボットの姿勢や向きなどをセンサで観測しても、それはあくまで観測値であり、状態値ではない。一般に状態値はあくまで観測値などを用いて推定する対象であり、直接観測できないとされる。
| :math:`\mathbf{u}(t) \in \mathbb{R}^{m}` はシステムに対する入力である。たとえばロボットの車輪を何回転させるか、向きを何度変化させるかといった情報をここに格納する。入力も時間変化するため、 :math:`\mathbf{u}(t)` も時刻 :math:`t` の関数である。一般にロボットは付随するサーボモータや駆動モータなどへの入力値を直接把握できる。したがって、ここでは任意の時刻 :math:`t` について :math:`\mathbf{u}(t)` の値を知ることができるものとする。
| :math:`\mathbf{y}(t) \in \mathbb{R}^{l}, l \leq n` はセンサーによる状態の観測値を表す。たとえばロボットに距離センサを搭載すれば周囲の物体までの距離がわかるため、こういった値を :math:`\mathbf{y}(t)` に格納する。状態が変化すると観測値も変化するため、 :math:`\mathbf{y}(t)` も時刻 :math:`t` の関数である。一般的にセンサはある一定間隔で観測値を出力する。しかしここでは理論面での話を扱うため、任意の時刻 :math:`t` について :math:`\mathbf{y}(t)` の値を知ることができるものと仮定する。
| 1本目の式は状態方程式、2本目の式は出力方程式と呼ばれる。我々の目的はある時刻 :math:`t_{a}` における観測値 :math:`\mathbf{y}(t_{a})` から別の時刻 :math:`t_{b}` の状態 :math:`\mathbf{x}(t_{b})` を計算することである。なお、関数 :math:`\mathbf{f}, \mathbf{h}` のいずれも :math:`C^{\infty}` 級とする。

線形システムの場合
------------------

非線形システム :eq:`nonlinear-system` は複雑で解析が難しいため、行列 :math:`A \in \mathbb{R}^{n \times n}, B \in \mathbb{R}^{n \times m}, C \in \mathbb{R}^{l \times n}` を用いてより単純な線形システムを考えることもある。

.. math::
   \dot{\mathbf{x}}(t) &= A\mathbf{x}(t) + B\mathbf{u}(t) \\
   \mathbf{y}(t) &= C\mathbf{x}(t)
   :label: linear-system

ここで :math:`\mathbf{u}(t) = \left[u_{1}(t) \; ... \; u_{m}(t)\right]^{\top}` である。 :ref:`solving_state_equation` より、 :math:`\mathbf{x}(t_{0})` を初期値とした :math:`\mathbf{x}(t_{1})` の値は次の式で表される。

.. math::
    \mathbf{x}(t_{1}) = e^{A(t_{1}-t_{0})} \mathbf{x}(t_{0}) + \int_{t_{0}}^{t_{1}} e^{A(t_{1}-\tau)}B\mathbf{u}(\tau) d\tau, \; t_{0} \leq t_{1}
   :label: linear-transition-solution

システムの可観測性
==================

線形システムの可観測性
----------------------

ある時刻 :math:`t_{1}, t_{1} > t_{0}` の観測値 :math:`\mathbf{y}(t_{1})` を用いてシステムの初期値 :math:`\mathbf{x}(t_{0})` を一意に定められるとき、そのシステムは可観測であるという。
具体例を見たほうがわかりやすいため、まずは線形システムの可観測性について見てみよう。

ここではまず線形システムの可観測性を考える。状態方程式の解 :eq:`linear-transition-solution` より、状態と観測値の関係は次の式で表される。

.. math::
   \mathbf{y}(t_{1}) = C e^{A (t_{1} - t_{0})} \mathbf{x}(t_{0}) + C \int_{t_{0}}^{t_{1}} e^{A(t_{1} - \tau)} B \mathbf{u}(\tau) d\tau \\

| 制御入力に関連する項を左辺に移し、これを :math:`\mathbf{b}(t_{0}, t_{1})` とおこう。

.. math::
   \mathbf{b}(t_{0}, t_{1}) = \mathbf{y}(t_{1}) - C \int_{t_{0}}^{t_{1}} e^{A(t_{1} - \tau)} B \mathbf{u}(\tau) d\tau = C e^{A (t_{1} - t_{0})} \mathbf{x}(t_{0}) \\
   :label: known-unknown-relationship-in-linear-system

| さて、仮定より我々は任意の時刻 :math:`t` において :math:`\mathbf{y}(t)` および :math:`\mathbf{u}(t)` の値を得られるため、 :math:`\mathbf{b}(t_{0}, t_{1})` の値も正確に知ることができる。
| では :math:`\mathbf{x}(t_{0})` はどうだろうか。 :math:`\mathbf{x}(t_{0})` は直接観測できないため、 :math:`\mathbf{b}(t_{0}, t_{1})` から計算する必要がある。しかし :math:`C e^{A (t_{1} - t_{0})}` の性質によっては、ある異なる時刻 :math:`t` の状態について :math:`\mathbf{b}(t_{0}, t_{1})` と同一の :math:`\mathbf{b}(t, t_{1})` が観測される、すなわち

.. math::
   C e^{A (t_{1} - t)} \mathbf{x}(t) = C e^{A (t_{1} - t_{0})} \mathbf{x}(t_{0}) = \mathbf{b}(t, t_{1}) = \mathbf{b}(t_{0}, t_{1}), t \neq t_{0}

となる :math:`\mathbf{x}(t)` が存在してしまうことが考えられる。この場合、特定の時刻 :math:`t_{0}` の状態 :math:`\mathbf{x}(t_{0})` を一意に定めることができない。このとき、この線形システムは可観測でない。

以上より次の2つのことがおわかりいただけただろう。

* ある時刻 :math:`t_{1}` の観測値 :math:`\mathbf{y}(t_{1})` を用いて別の時刻 :math:`t_{0}, t_{0} < t_{1}` における状態 :math:`\mathbf{x}(t_{0})` を一意に定められるとき、そのシステムは可観測であるという
* 線形システムにおいては行列 :math:`C e^{A (t_{1} - t_{0})}` の性質を調べることでシステムの可観測性を判定できる

非線形システムの可観測性
------------------------

非線形なシステムの可観測性を見ていこう。

.. math::
   \dot{\mathbf{x}}(t) &= \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t)) \\
   \mathbf{y}(t) &= \mathbf{h}(\mathbf{x}(t))

改めて、我々の目的は観測値 :math:`\mathbf{y}(t) \in \mathbb{R}^{l}` および既知の入力 :math:`\mathbf{u}(t) \in \mathbb{R}^{m}` から状態 :math:`\mathbf{x}(t) \in \mathbb{R}^{n}` を一意に定めることである。
一般に観測値の次元数 :math:`l` は状態の次元数 :math:`n` と同じかそれより小さいため、 :math:`\mathbf{h}` の逆関数を求めるだけでは状態を一意に定めることができない。ではどうするかというと、 :math:`\mathbf{y}(t)` を :math:`\nu` 回微分して互いに独立な関数を :math:`\nu + 1` 個列挙し、これと状態 :math:`\mathbf{x}(t)` をある関数 :math:`\phi` によって対応付けることで状態を一意に定めるのである。

.. math::
   O &:= \begin{bmatrix}
   \mathbf{y}(t) \\
   \dot{\mathbf{y}}(t) \\
   \vdots \\
   \mathbf{y}^{(\nu)}(t) \\
   \end{bmatrix} \\
   O &= \mathbf{\phi}(\mathbf{x}(t))
   :label: observation-space

:math:`\mathbf{y}^{(\nu)}(t)` よりも高次の導関数を :math:`O` に含めないのは、 :math:`\nu + 1` 次以上の導関数が :math:`\nu` 次までの導関数の線型結合で表せることを仮定しているからである(参照： :ref:`representing_higher_order_derivatives` )。

我々の関心は :math:`\mathbf{\phi}` の逆関数 :math:`\mathbf{\phi}^{-1}` が存在するかどうかである。 とりうる全ての状態について逆関数 :math:`\mathbf{\phi}^{-1}(\mathbf{x}(t))` の値を一意に定められるとき、そのシステムは可観測である。


可観測性の検証、すなわち :math:`\mathbf{\phi}` が可逆であるかどうかの検証には逆関数定理を用いる。

  開集合 :math:`U \subset R^{n}` および微分可能な写像 :math:`\mathbf{\phi} : U \to R^{n}` について、 :math:`\mathbf{\phi}` の :math:`\mathbf{p} \in U` におけるヤコビアン :math:`\frac{\partial \mathbf{\phi}}{\partial \mathbf{x}}` が正則であるとき、 :math:`\mathbf{\phi}` は :math:`\mathbf{p}` の近傍で可逆である。

すなわち、ある状態 :math:`\mathbf{x}_{0} \in \mathbb{R}^{n}` の近傍で :math:`\mathbf{\phi}` が可逆であることは、 :math:`\operatorname{rank}( \left .{ \frac{\partial \mathbf{\phi}}{\partial \mathbf{x}} } \right \vert_{\mathbf{x}_{0}} ) = n` が成り立つことと等価である。

逆関数定理はあくまで点 :math:`\mathbf{p}` の近傍における関数 :math:`\mathbf{\phi}` の可逆性を述べている。一般に、逆関数定理だけでは :math:`\mathbf{\phi}` の定義域全体における可逆性は検証できないことに注意が必要である。

以上より、非線形システム :eq:`nonlinear-system` の局所的な可観測性は次のようにして調べることができる。

  非線形システム :eq:`nonlinear-system` は、状態と観測空間を対応付ける写像 :math:`\mathbf{\phi}` および状態 :math:`\mathbf{x}_{0}` について :math:`\operatorname{rank}( \left .{  \frac{\partial \mathbf{\phi}}{\partial \mathbf{x}} } \right \vert_{\mathbf{x}_{0}})) = n` が成り立つとき、 :math:`\mathbf{x}_{0}` の周辺で局所的に可観測である。

逆関数定理はあくまで局所的な可逆性を述べるのみであるため、一般的には非線形システムの可観測性も局所的にしか明らかにできないことに注意が必要である。

.. _observability_of_discrete_linear_system:

離散時間線形システムの可観測性
------------------------------

離散時間線形システムの観測性を調べる。

.. math::
   \mathbf{x}_{k+1} &= A\mathbf{x}_{k} + B\mathbf{u}_{k}  \\
   \mathbf{y}_{k} &= C\mathbf{x}_{k} \\
   :label: discrete-linear-system

| ベクトルや行列のサイズは :eq:`linear-system` と同様で、 :math:`\mathbf{x}_{k} \in \mathbb{R}^{n}, \mathbf{y}_{k} \in \mathbb{R}^{m}, A \in \mathbb{R}^{n \times n},B \in \mathbb{R}^{n \times m},C \in \mathbb{R}^{l \times n}` とする。
| まずはこれを時間発展に沿って展開してみよう。

.. math::
   \mathbf{y}_{0} &= C\mathbf{x}_{0} \\
   \\
   \mathbf{x}_{1} &= A\mathbf{x}_{0} + B\mathbf{u}_{0} \\
   \mathbf{y}_{1} &= C\mathbf{x}_{1} \\
                 &= CA\mathbf{x}_{0} + CB\mathbf{u}_{0} \\
   \\
   \mathbf{x}_{2} &= A\mathbf{x}_{1} + B\mathbf{u}_{1} \\
                 &= A(A\mathbf{x}_{0} + B\mathbf{u}_{0}) + B\mathbf{u}_{1} \\
                 &= A^{2}\mathbf{x}_{0} + AB\mathbf{u}_{0} + B\mathbf{u}_{1} \\
   \mathbf{y}_{2} &= C\mathbf{x}_{2} \\
                 &= CA^{2}\mathbf{x}_{0} + CAB\mathbf{u}_{0} + CB\mathbf{u}_{1} \\
   \\
   \mathbf{x}_{3} &= A\mathbf{x}_{2} + B\mathbf{u}_{2} \\
                 &= A(A^{2}\mathbf{x}_{0} + AB\mathbf{u}_{0} + B\mathbf{u}_{1}) + B\mathbf{u}_{2} \\
                 &= A^{3}\mathbf{x}_{0} + A^2 B\mathbf{u}_{0} + AB\mathbf{u}_{1} + B\mathbf{u}_{2} \\
   \mathbf{y}_{3} &= C\mathbf{x}_{3} \\
                 &= CA^{3}\mathbf{x}_{0} + CA^2 B\mathbf{u}_{0} + CAB\mathbf{u}_{1} + CB\mathbf{u}_{2} \\
   &\vdots \\
   \mathbf{x}_{k} &= A\mathbf{x}_{k} + B\mathbf{u}_{k} \\
                 &= A^{k}\mathbf{x}_{0} + \sum_{i=0}^{k-1} A^{k-i-1} B\mathbf{u}(i) \\
   \mathbf{y}_{k} &= C\mathbf{x}_{k} \\
                 &= CA^{k}\mathbf{x}_{0} + \sum_{i=0}^{k-1} CA^{k-i-1} B\mathbf{u}(i) \\

状態の次元数は :math:`n` なので、状態 :math:`\mathbf{x}_{k}` を一意に決定するには :math:`n` 本の式が作れればよい。

.. math::
   \mathbf{y}_{k+0} &= C\mathbf{x}_{k} \\
   \mathbf{y}_{k+1} &= CA\mathbf{x}_{k} + CB\mathbf{u}_{k} \\
   \mathbf{y}_{k+2} &= CA^{2}\mathbf{x}_{k} + CAB\mathbf{u}_{k} + CB\mathbf{u}_{k+1} \\
   \mathbf{y}_{k+3} &= CA^{3}\mathbf{x}_{k} + CA^2 B\mathbf{u}_{k} + CAB\mathbf{u}_{k+1} + CB\mathbf{u}_{k+2} \\
   &\vdots \\
   \mathbf{y}_{k+n-1} &= CA^{n-1}\mathbf{x}_{k} + \sum_{i=0}^{n-1} CA^{n-i-2} B\mathbf{u}_{k+i} \\
   :label: discrete-observation-time-series

この時間展開をまとめてよりシンプルな式で表現しよう。

.. math::
   \mathbf{y} = \begin{bmatrix}
   \mathbf{y}_{k+0} \\
   \mathbf{y}_{k+1} \\
   \mathbf{y}_{k+2} \\
   \mathbf{y}_{k+3} \\
   \vdots \\
   \mathbf{y}_{k+n-1} \\
   \end{bmatrix},\;
   M = \begin{bmatrix}
   C      \\
   CA     \\
   CA^{2} \\
   CA^{3} \\
   \vdots \\
   CA^{n-1} \\
   \end{bmatrix},\;
   \mathbf{u} = \begin{bmatrix}
   \mathbf{0} \\
   CB\mathbf{u}_{0} \\
   CAB\mathbf{u}_{0} + CB\mathbf{u}_{1} \\
   CA^2 B\mathbf{u}_{0} + CAB\mathbf{u}_{1} + CB\mathbf{u}_{2} \\
   \vdots \\
   \sum_{i=0}^{n-1} CA^{n-i-2} B\mathbf{u}_{k+i} \\
   \end{bmatrix}
   :label: observability_matrices

これにより時間発展 :eq:`discrete-observation-time-series` は次の式で表現できる。

.. math::
   \mathbf{y} = M\mathbf{x}_{k} + \mathbf{u}  \\
   :label: simplified-discrete-observation-time-series

観測値 :math:`\mathbf{y}` から :math:`\mathbf{x}_{k}` を計算するには次のようにすればよい。

.. math::
   \mathbf{x}_{k} = (M^{\top}M)^{-1}(M^{\top}\mathbf{y} - M^{\top}\mathbf{u})

| 仮定より、 :math:`\mathbf{y}, \mathbf{u}` はいずれも既知である。
| 以上より、離散時間線形システムについてある時刻 :math:`k` の状態 :math:`\mathbf{x}_{k}` を一意に決定するための必要条件は行列 :math:`(M^{\top}M)` が正則であること、すなわち :math:`\operatorname{rank}(M^{\top}M) = \operatorname{rank}(M) = n` であることがわかる。

行列 :math:`M` はシステムの可観測製の判定に使えるため、 **可観測行列** と呼ばれる。

式 :eq:`simplified-discrete-observation-time-series` を見れば、行列 :math:`M` の零空間(あるいは核) :math:`\operatorname{Null}(M)` が観測不可能な空間を表していることがわかる。

.. math::
   \operatorname{Null}(M) = \left\{ \mathbf{x} \in \mathbf{R}^{n} \;|\; M\mathbf{x} = \mathbf{0} \right\}

EKFの局所可観測性の検証
=======================

根底となるモデル
----------------

ここでは2次元平面上で unicycle model に従って動くロボットの状態遷移をEKFによって推定することを考える。

時刻 :math:`k` における状態を次のように表す。

.. math::
    \mathbf{x}_{k} = \left[x_{{R}_{k}}\; y_{{R}_{k}}\; \phi_{{R}_{k}}\; x_{{{L}_{1}}_{k}}\; y_{{{L}_{1}}_{k}}\; ...\; x_{{{L}_{N}}_{k}}\; y_{{{L}_{N}}_{k}} \right]^{\top} \in \mathbf{R}^{2N+3}

:math:`x_{{R}_{k}}, y_{{R}_{k}}, \phi_{{R}_{k}}` はそれぞれロボットのx座標、y座標、ヨー角を表す。 :math:`x_{{{L}_{i}}_{k}}, y_{{{L}_{i}}_{k}}, i=1,...,N` はランドマークの座標を表す。
状態は :math:`\delta t` 秒ごとに更新されるものとする。

制御入力は車両の速度を :math:`v_{k}` 、角速度を :math:`\omega_{k}` とし、 :math:`\mathbf{u}_{k} = \left[v_{k}\;\omega_{k}\right]^{\top}` と定める。

真値の状態遷移にはノイズが乗る。ノイズの共分散行列を :math:`Q \in \mathbb{R}^{(2N+3)\times(2N+3)}` とすると、真値の状態遷移は次のように表される。

.. math::
    \mathbf{x}_{k+1} = \mathbf{f}(\mathbf{x}_{k}, \mathbf{u}_{k}) + \mathbf{w}_{k}, \quad \mathbf{w}_{k} \sim \mathcal{N}(\mathbf{0}, Q_{k}) \\

関数 :math:`\mathbf{f}` は次のようになる。

.. math::
    \mathbf{f} :
    \begin{bmatrix}
    x_{{R}_{k}}       \\
    y_{{R}_{k}}       \\
    \phi_{{R}_{k}}    \\
    x_{{{L}_{1}}_{k}} \\
    y_{{{L}_{1}}_{k}} \\
    \vdots \\
    x_{{{L}_{N}}_{k}} \\
    y_{{{L}_{N}}_{k}} \\
    \end{bmatrix}
    \mapsto
    \begin{bmatrix}
    x_{{R}_{k}} + v_{k} \cos(\phi_{{R}_{k}}) \delta t  \\
    y_{{R}_{k}} + v_{k} \sin(\phi_{{R}_{k}}) \delta t  \\
    \phi_{{R}_{k}} + \omega_{k} \delta t \\
    x_{{{L}_{1}}_{k}}  \\
    y_{{{L}_{1}}_{k}}  \\
    \vdots \\
    x_{{{L}_{N}}_{k}}  \\
    y_{{{L}_{N}}_{k}}  \\
    \end{bmatrix}


次に観測モデルを定める。ここでは一般的に用いられる range-bearing センサに倣い、ランドマークまでの距離と角度を観測値とする。

.. math::
   \mathbf{h} :
    \begin{bmatrix}
    x_{{R}_{k}}       \\
    y_{{R}_{k}}       \\
    \phi_{{R}_{k}}    \\
    x_{{{L}_{1}}_{k}} \\
    y_{{{L}_{1}}_{k}} \\
    \vdots \\
    x_{{{L}_{N}}_{k}} \\
    y_{{{L}_{N}}_{k}} \\
    \end{bmatrix}
    \mapsto
    \begin{bmatrix}
    \sqrt{(x_{{{L}_{1}}_{k}} - x_{{R}_{k}})^{2} + (y_{{R}_{k}} - y_{{{L}_{1}}_{k}})^{2}} \\
    \vdots \\
    \sqrt{(x_{{{L}_{N}}_{k}} - x_{{R}_{k}})^{2} + (y_{{R}_{k}} - y_{{{L}_{N}}_{k}})^{2}} \\
    \operatorname{atan_2}(y_{{R}_{k}} - y_{{{L}_{1}}_{k}}, x_{{{L}_{1}}_{k}} - x_{{R}_{k}}) - \phi_{R} \\
    \vdots \\
    \operatorname{atan_2}(y_{{R}_{k}} - y_{{{L}_{N}}_{k}}, x_{{{L}_{N}}_{k}} - x_{{R}_{k}}) - \phi_{R} \\
    \end{bmatrix}


観測値にもノイズが乗る。ノイズの共分散行列を :math:`R_{k} \in \mathbb{R}^{2N \times 2N}` とすると、観測値は次のように表される。

.. math::
    \mathbf{z}_{k} = \mathbf{h}(\mathbf{x}_{k}) + \mathbf{v}_{k}, \quad \mathbf{v}_{k} \sim \mathcal{N}(\mathbf{0}, R_{k}) \\

EKFによる状態推定
-----------------

Prediction
~~~~~~~~~~

時刻 :math:`j` の情報を用いて推定された時刻 :math:`i` の情報を :math:`i|j` で表記する。EKFでは状態誤差 :math:`\tilde{\mathbf{x}}_{k|k} = \mathbf{x}_{k} - \hat{\mathbf{x}}_{k|k}` が平均 :math:`\mathbf{0}` 、分散 :math:`P_{k|k}` の正規分布に従い、これが行列 :math:`\Phi_{k}` に従って遷移していくことを仮定する。

.. math::
    \tilde{\mathbf{x}}_{k+1|k}
    &= \mathbf{f}(\mathbf{x}_{k}, \mathbf{u}_{k}) - \mathbf{f}(\hat{\mathbf{x}}_{k|k}, \mathbf{u}_{k}) \\
    &\approx \Phi_{k}(\mathbf{x}_{k} - \hat{\mathbf{x}}_{k|k}) \\
    &= \Phi_{k}\tilde{\mathbf{x}}_{k|k},\;
    \Phi_{k} = \left .{ \frac{\partial \mathbf{f}}{\partial \mathbf{x}_{k}}} \right \vert_{\hat{\mathbf{x}}_{k|k}, \mathbf{u}_{k}}

状態誤差 :math:`\tilde{\mathbf{x}}_{k|k}` の分散は次のように計算される。

.. math::
    P_{k|k} = \operatorname{cov}(\mathbf{x}_{k} - \hat{\mathbf{x}}_{k|k}) = \operatorname{cov}(\tilde{\mathbf{x}}_{k|k})

:math:`\mathbb{E}[\hat{\mathbf{x}}_{k|k}^{\top}\mathbf{w}] = 0` と仮定すれば、 :math:`\operatorname{cov}(\tilde{\mathbf{x}}_{k+1|k})` を次のように計算することができる。

.. math::
    P_{k+1|k} = \operatorname{cov}(\tilde{\mathbf{x}}_{k+1|k}) = \Phi_{k}P_{k|k}\Phi_{k}^{\top} + Q_{k}

Update
~~~~~~

観測モデルも同様に線形近似する。

.. math::
   \tilde{\mathbf{z}}_{k} &= \mathbf{z}_{k} - \tilde{\mathbf{z}}_{k|k-1} \\
                          &= \mathbf{h}(\mathbf{x}_{k}) - \mathbf{h}(\tilde{\mathbf{x}}_{k|k-1})  \\
                          &\approx H(\mathbf{x}_{k} - \hat{\mathbf{x}}_{k|k-1}) \\
                          &= H\tilde{\mathbf{x}}_{k|k-1},\;
   H_{k} = \left .{ \frac{\partial \mathbf{h}}{\partial \mathbf{x}_{k}} } \right\vert_{\hat{\mathbf{x}}_{k|k-1}}

カルマンゲインを計算し、状態と共分散を更新する。

.. math::
   S_{k} &= H_{k}P_{k|k-1}H_{k}^{\top} + R_{k}  \\
   K_{k} &= P_{k|k-1}H_{k}^{\top}S_{k}^{-1} \\
   \hat{\mathbf{x}}_{k|k} &= \hat{\mathbf{x}}_{k|k-1} + K_{k}(\mathbf{z}_{k} - \mathbf{h}(\hat{\mathbf{x}}_{k|k-1})) \\
   P_{k|k} &= (I - K_{k}H_{k})P_{k|k-1}  \\

EKFの可観測性
-------------

ここではEKFの可観測性を調査する。2次元平面状を動く車両の状態をEKFで推定すると、観測可能な次元数が理想的なケースよりも増えてしまうことを示す。これは共分散の過剰な収束および状態推定の不安定化を招く。

EKFの状態誤差の遷移は離散時間線形システムとみなすことができるため、その可観測性を調べるには :ref:`observability_of_discrete_linear_system` に従って可観測行列を作成し、そのランクを調べればよい。Jacobianが真の状態で計算される理想的なシステムと、Jacobianが状態の推定値で評価される通常のEKFについてそれぞれの可観測性を判定し、EKFが理想的なケースよりも多くの観測可能な次元数を持つことをみる。

理想的なケース
~~~~~~~~~~~~~~

まずは状態遷移モデルおよび観測モデルが真の状態で微分される理想的なシステムの可観測性を見る。これによりノイズにとわられない、システムが持つ本来の可観測性を調べることができる。

状態は真値をとり、かつノイズもないことを仮定する。したがって真の状態を表す記号を :math:`\mathbf{x}^{*}_{k|k}` とすると、 :math:`\mathbf{x}^{*}_{k|k-1} = \mathbf{x}^{*}_{k|k} = \mathbf{x}^{*}_{k}` である。

ノイズがないことを仮定するため、状態遷移は次のように表される。

.. math::
    \mathbf{x}^{*}_{k+1} &= \mathbf{f}(\mathbf{x}^{*}_{k}, \mathbf{u}_{k})  \\
    \mathbf{z}^{*}_{k} &= \mathbf{h}(\mathbf{x}^{*}_{k})

真の状態で評価したJacobianを記号 :math:`\breve{\boldsymbol{\cdot}}` で表記する。

.. math::
    \breve{\Phi}_{k} = \left .{ \frac{\partial \mathbf{f}}{\partial \mathbf{x}_{k}}} \right \vert_{\mathbf{x}^{*}_{k|k}, \mathbf{u}_{k}}
    \breve{H}_{k} = \left .{ \frac{\partial \mathbf{h}}{\partial \mathbf{x}_{k}}} \right \vert_{\mathbf{x}^{*}_{k|k}, \mathbf{u}_{k}}

.. math::
    \breve{\Phi}_{k}
    &=
    \frac{\partial }{\partial \mathbf{x}_{k}}
    \begin{bmatrix}
    x^{*}_{{R}_{k}} + v_{k} \cos(\phi^{*}_{{R}_{k}}) \delta t  \\
    y^{*}_{{R}_{k}} + v_{k} \sin(\phi^{*}_{{R}_{k}}) \delta t  \\
    \phi^{*}_{{R}_{k}} + \omega^{*}_{k} \delta t \\
    x^{*}_{{{L}_{1}}_{k}}  \\
    y^{*}_{{{L}_{1}}_{k}}  \\
    \vdots \\
    x^{*}_{{{L}_{N}}_{k}}  \\
    y^{*}_{{{L}_{N}}_{k}}  \\
    \end{bmatrix} \\
    &=
    \begin{bmatrix}
    1 &   & -v_{k} \sin(\phi^{*}_{{R}_{k}}) \delta t &   &   &        &   &   \\
      & 1 &  v_{k} \cos(\phi^{*}_{{R}_{k}}) \delta t &   &   &        &   &   \\
      &   &                                        1 &   &   &        &   &   \\
      &   &                                          & 1 &   &        &   &   \\
      &   &                                          &   & 1 &        &   &   \\
      &   &                                          &   &   & \ddots &   &   \\
      &   &                                          &   &   &        & 1 &   \\
      &   &                                          &   &   &        &   & 1 \\
    \end{bmatrix}

関係 :math:`v_{k} \sin(\phi^{*}_{{R}_{k}}) = y^{*}_{R_{k+1}} - y^{*}_{R_{k}},\;v_{k} \cos(\phi^{*}_{{R}_{k}}) = x^{*}_{R_{k+1}} - x^{*}_{R_{k}}` を用いると次のようになる。

.. math::
    \breve{\Phi}_{k}
    &=
    \begin{bmatrix}
    1 &   & -(y^{*}_{R_{k+1}} - y^{*}_{R_{k}}) &   &   &        &   &   \\
      & 1 &  x^{*}_{R_{k+1}} - x^{*}_{R_{k}}   &   &   &        &   &   \\
      &   &                          1 &   &   &        &   &   \\
      &   &                            & 1 &   &        &   &   \\
      &   &                            &   & 1 &        &   &   \\
      &   &                            &   &   & \ddots &   &   \\
      &   &                            &   &   &        & 1 &   \\
      &   &                            &   &   &        &   & 1 \\
    \end{bmatrix} \\
    &=
    \begin{bmatrix}
    \begin{array}{c|c}
    \breve{\Phi}_{R_{k}} & 0      \\
          \hline
               0 & I      \\
    \end{array}
    \end{bmatrix}

ここでロボット状態誤差の遷移に関する部分を :math:`\breve{\Phi}_{R_{k}}` としている。

.. math::
    \breve{\Phi}_{R_{k}}
    &=
    \begin{bmatrix}
      1 &   & -(y^{*}_{R_{k+1}} - y^{*}_{R_{k}}) & \\
        & 1 &  x^{*}_{R_{k+1}} - x^{*}_{R_{k}}   & \\
        &   &                          1 &
    \end{bmatrix}


観測モデルのJacobianは次のようになる。

.. math::
   \breve{H_{k}}
   &=
   \begin{bmatrix}
   \begin{array}{ccc|cccc}
   -\frac{x^{*}_{{{L}_{1}}_{k}} - x^{*}_{{R}_{k}}}{\rho_{1}} &
   -\frac{y^{*}_{{{L}_{1}}_{k}} - y^{*}_{{R}_{k}}}{\rho_{1}} &
   0 &
   \frac{x^{*}_{{{L}_{1}}_{k}} - x^{*}_{{R}_{k}}}{\rho_{1}} &
   \frac{y^{*}_{{{L}_{1}}_{k}} - y^{*}_{{R}_{k}}}{\rho_{1}} & 0 & 0 \\
   \vdots & \vdots & \vdots & 0 & \ddots & \ddots & 0 \\
   -\frac{x^{*}_{{{L}_{N}}_{k}} - x^{*}_{{R}_{k}}}{\rho_{N}} &
   -\frac{y^{*}_{{{L}_{N}}_{k}} - y^{*}_{{R}_{k}}}{\rho_{N}} &
   0 & 0 & 0 &
   \frac{x^{*}_{{{L}_{N}}_{k}} - x^{*}_{{R}_{k}}}{\rho_{N}} &
   \frac{y^{*}_{{{L}_{N}}_{k}} - y^{*}_{{R}_{k}}}{\rho_{N}} \\
    \frac{y^{*}_{{{L}_{1}}_{k}} - y^{*}_{{R}_{k}}}{\rho_{1}^2} &
   -\frac{x^{*}_{{{L}_{1}}_{k}} - x^{*}_{{R}_{k}}}{\rho_{1}^2} &
   -1 &
   -\frac{y^{*}_{{{L}_{1}}_{k}} - y^{*}_{{R}_{k}}}{\rho_{1}^2} &
    \frac{x^{*}_{{{L}_{1}}_{k}} - x^{*}_{{R}_{k}}}{\rho_{1}^2} & 0 & 0 \\
   \vdots & \vdots & \vdots & 0 & \ddots & \ddots & 0 \\
    \frac{y^{*}_{{{L}_{N}}_{k}} - y^{*}_{{R}_{k}}}{\rho_{N}^2} &
   -\frac{x^{*}_{{{L}_{N}}_{k}} - x^{*}_{{R}_{k}}}{\rho_{N}^2} &
   -1 & 0 & 0 &
   -\frac{y^{*}_{{{L}_{N}}_{k}} - y^{*}_{{R}_{k}}}{\rho_{N}^2} &
    \frac{x^{*}_{{{L}_{N}}_{k}} - x^{*}_{{R}_{k}}}{\rho_{N}^2} \\
   \end{array}
   \end{bmatrix} \\
   &=
   \begin{bmatrix}
   \begin{array}{c|c}
   \breve{H}_{R_{k}} & \breve{H}_{L_{k}}
   \end{array}
   \end{bmatrix},\\
   &\text{where}\;\rho_{j} = \sqrt{(x^{*}_{{{L}_{j}}_{k}} - x^{*}_{{R}_{k}})^{2} + (y^{*}_{{R}_{k}} - y^{*}_{{{L}_{j}}_{k}})^{2}}

理想的なシステムの可観測性を見てみよう。式 :eq:`observability_matrices` にしたがって可観測行列を計算する。時刻 :math:`k` を起点とした可観測行列は次のようになる。

.. math::
   \breve{M}_{k} = \begin{bmatrix}
    \breve{H}_{k}  \\
    \breve{H}_{k+1} \breve{\Phi}_{k}  \\
    \breve{H}_{k+2} \breve{\Phi}_{k+1} \breve{\Phi}_{k}  \\
    \vdots  \\
    \breve{H}_{k+n} \breve{\Phi}_{k+n-1} ... \breve{\Phi}_{k+1} \breve{\Phi}_{k}
   \end{bmatrix}
   &=
   \begin{bmatrix}
    \breve{H}_{R_{k}} & \breve{H}_{L_{k}}  \\
    \breve{H}_{R_{k+1}}\breve{\Phi}_{R_{k}} & \breve{H}_{L_{k+1}} \\
    \breve{H}_{R_{k+2}}\breve{\Phi}_{R_{k+1}}\breve{\Phi}_{R_{k}} & \breve{H}_{L_{k+2}} \\
    \vdots \\
    \breve{H}_{R_{k+n}}\breve{\Phi}_{R_{k+n-1}} ... \breve{\Phi}_{R_{k+1}} \breve{\Phi}_{R_{k}} & \breve{H}_{L_{k+n}}
   \end{bmatrix}

この行列のランクを調べればシステムの可観測性を判定できる。

まずは :math:`\breve{\Phi}_{R_{k}}` の便利な性質を活用しよう。

.. math::
    \breve{\Phi}_{R_{k+1}} \breve{\Phi}_{R_{k}}
    &=
    \begin{bmatrix}
      1 &   & -(y^{*}_{R_{k+2}} - y^{*}_{R_{k+1}})\\
        & 1 &  x^{*}_{R_{k+2}} - x^{*}_{R_{k+1}}  \\
        &   &                            1
    \end{bmatrix}
    \begin{bmatrix}
      1 &   & -(y^{*}_{R_{k+1}} - y^{*}_{R_{k}}) \\
        & 1 &  x^{*}_{R_{k+1}} - x^{*}_{R_{k}}   \\
        &   &                          1
    \end{bmatrix} \\
    &=
    \begin{bmatrix}
      1 &   & -(y^{*}_{R_{k+2}} - y^{*}_{R_{k}}) \\
        & 1 &  x^{*}_{R_{k+2}} - x^{*}_{R_{k}}   \\
        &   &                          1
    \end{bmatrix}

より、ある :math:`\lambda=1,...,n` について

.. math::
    \breve{\Phi}_{R_{k+\lambda-1}} ... \breve{\Phi}_{R_{k+1}} \breve{\Phi}_{R_{k}}
    &=
    \begin{bmatrix}
      1 &   & -(y^{*}_{R_{k+\lambda}} - y^{*}_{R_{k}}) \\
        & 1 &  x^{*}_{R_{k+\lambda}} - x^{*}_{R_{k}}   \\
        &   &                            1     \\
    \end{bmatrix}

である。

つぎに :math:`\breve{H}_{k}` について見てみよう。
まず関数 :math:`\mathbf{h}(\mathbf{x}_{k})` を :math:`\mathbf{h}_{a}, \mathbf{h}_{b}` の2つに分解し、 :math:`\mathbf{h}(\mathbf{x}_{k}) = \mathbf{h}_{a}(\mathbf{h}_{b}(\mathbf{x}_{k}))` とする。ここで :math:`\mathbf{h}_{b}` を次のように定義する。

.. math::
    \mathbf{h}_{b}(\mathbf{x}_{k})
    &=
    \begin{bmatrix}
    C(\phi_{R_{k}})^{\top} & &  \\
    & \ddots & \\
    & & C(\phi_{R_{k}})^{\top} \\
    \end{bmatrix}
    \begin{bmatrix}
    x^{*}_{{{L}_{1}}_{k}} - x^{*}_{{R}_{k}}  \\
    y^{*}_{{{L}_{1}}_{k}} - y^{*}_{{R}_{k}}  \\
    \vdots  \\
    x^{*}_{{{L}_{N}}_{k}} - x^{*}_{{R}_{k}}  \\
    y^{*}_{{{L}_{N}}_{k}} - y^{*}_{{R}_{k}}  \\
    \end{bmatrix}\\
    &\text{where}\;
    C(\phi) = \begin{bmatrix}
        \cos \phi & -\sin \phi \\
        \sin \phi & \cos \phi
    \end{bmatrix}

:math:`\mathbf{h}_{b}` はロボットから見たランドマークの相対位置を表している。

合成関数の微分法により、 :math:`\breve{H}_{k}` は次のように計算できる。

.. math::
    \breve{H}_{k} =
    \begin{bmatrix}
    \begin{array}{c|c}
    \breve{H}_{R_{k}} & \breve{H}_{L_{k}}
    \end{array}
    \end{bmatrix}
    = \frac{\partial \mathbf{h}_{a}}{\partial \mathbf{h}_{b}} \frac{\partial \mathbf{h}_{b}}{\partial \mathbf{x}_{k}}

:math:`\mathbf{h}_{b}` の微分は以下のように計算される。


..
  .. math::
      \frac{\partial \mathbf{h}_{b}(\mathbf{x}_{k})}{\partial \phi_{R_{k}}}
      &=
      \begin{bmatrix}
      C(\phi_{R_{k}})^{\top} & &  \\
      & \ddots & \\
      & & C(\phi_{R_{k}})^{\top} \\
      \end{bmatrix}
      \begin{bmatrix}
        y^{*}_{{{L}_{1}}_{k}} - y^{*}_{{R}_{k}}   \\
      -(x^{*}_{{{L}_{1}}_{k}} - x^{*}_{{R}_{k}})  \\
      \vdots  \\
        y^{*}_{{{L}_{N}}_{k}} - y^{*}_{{R}_{k}}   \\
      -(x^{*}_{{{L}_{N}}_{k}} - x^{*}_{{R}_{k}})  \\
      \end{bmatrix}

.. math::
    \frac{\partial \mathbf{h}_{b}}{\partial \mathbf{x}_{k}}
    &=
    \begin{bmatrix}
    C^{\top}(\phi_{R_{k}}) &        &                        \\
                           & \ddots &                        \\
                           &        & C^{\top}(\phi_{R_{k}}) \\
    \end{bmatrix}
    \begin{bmatrix}
    \begin{array}{ccc|ccccc}
    -1     &        &   y^{*}_{{{L}_{1}}_{k}} - y^{*}_{{R}_{k}}  & 1 &   &        &   &   \\
           & -1     & -(x^{*}_{{{L}_{1}}_{k}} - x^{*}_{{R}_{k}}) &   & 1 &        &   &   \\
    \vdots & \vdots & \vdots                             &   &   & \ddots &   &   \\
    -1     &        &   y^{*}_{{{L}_{N}}_{k}} - y^{*}_{{R}_{k}}  &   &   &        & 1 &   \\
           & -1     & -(x^{*}_{{{L}_{N}}_{k}} - x^{*}_{{R}_{k}}) &   &   &        &   & 1 \\
    \end{array}
    \end{bmatrix} \\
    &=
    \begin{bmatrix}
    C^{\top}(\phi_{R_{k}}) &        &                        \\
                           & \ddots &                        \\
                           &        & C^{\top}(\phi_{R_{k}}) \\
    \end{bmatrix}
    \begin{bmatrix}
    \begin{array}{c|c}
        \breve{H}_{b_{R_{k}}} & I
    \end{array}
    \end{bmatrix}
    :label: h_b_derivative

したがって :math:`\breve{H}_{R_{k}}` は次のようになる。

.. math::
   \breve{H}_{R_{k+\lambda}}
    &= \frac{\partial \mathbf{h}_{a}}{\partial \mathbf{h}_{b}}
    \begin{bmatrix}
    C^{\top}(\phi_{R_{k+\lambda}}) &        &                        \\
                           & \ddots &                        \\
                           &        & C^{\top}(\phi_{R_{k+\lambda}}) \\
    \end{bmatrix}
    \breve{H}_{b_{R_{k+\lambda}}}  \\
    &=
    \frac{\partial \mathbf{h}_{a}}{\partial \mathbf{h}_{b}}
    \begin{bmatrix}
    C^{\top}(\phi_{R_{k+\lambda}}) &        &                        \\
                           & \ddots &                        \\
                           &        & C^{\top}(\phi_{R_{k+\lambda}}) \\
    \end{bmatrix}
    \begin{bmatrix}
    -1     &        &   y^{*}_{{{L}_{1}}_{k+\lambda}} - y^{*}_{{R}_{k+\lambda}}  \\
           & -1     & -(x^{*}_{{{L}_{1}}_{k+\lambda}} - x^{*}_{{R}_{k+\lambda}}) \\
    \vdots & \vdots & \vdots                             \\
    -1     &        &   y^{*}_{{{L}_{N}}_{k+\lambda}} - y^{*}_{{R}_{k+\lambda}}  \\
           & -1     & -(x^{*}_{{{L}_{N}}_{k+\lambda}} - x^{*}_{{R}_{k+\lambda}}) \\
    \end{bmatrix} \\

可観測行列の :math:`\lambda+1` ブロック行目のうちロボットの状態 :math:`x^{*}_{{R}_{k}}\; y^{*}_{{R}_{k}}\; \phi_{{R}_{k}}` に関連する部分は次のように計算できる。

.. math::
    \breve{H}_{R_{k+\lambda}}\breve{\Phi}_{R_{k+\lambda-1}} ... \breve{\Phi}_{R_{k}}
    &=
    D_{k+\lambda}
    \begin{bmatrix}
    -1     &        &   y^{*}_{{{L}_{1}}_{k+\lambda}} - y^{*}_{{R}_{k+\lambda}}  \\
           & -1     & -(x^{*}_{{{L}_{1}}_{k+\lambda}} - x^{*}_{{R}_{k+\lambda}}) \\
    \vdots & \vdots & \vdots                             \\
    -1     &        &   y^{*}_{{{L}_{N}}_{k+\lambda}} - y^{*}_{{R}_{k+\lambda}}  \\
           & -1     & -(x^{*}_{{{L}_{N}}_{k+\lambda}} - x^{*}_{{R}_{k+\lambda}}) \\
    \end{bmatrix}
    \begin{bmatrix}
      1 &   & -(y^{*}_{R_{k+\lambda}} - y^{*}_{R_{k}}) & \\
        & 1 &  x^{*}_{R_{k+\lambda}} - x^{*}_{R_{k}}   & \\
        &   &                            1 &
    \end{bmatrix} \\
    &=
    D_{k+\lambda}
    \begin{bmatrix}
    -1     &        &   y^{*}_{{{L}_{1}}_{k+\lambda}} - y^{*}_{{R}_{k}}  \\
           & -1     & -(x^{*}_{{{L}_{1}}_{k+\lambda}} - x^{*}_{{R}_{k}}) \\
    \vdots & \vdots & \vdots                             \\
    -1     &        &   y^{*}_{{{L}_{N}}_{k+\lambda}} - y^{*}_{{R}_{k}}  \\
           & -1     & -(x^{*}_{{{L}_{N}}_{k+\lambda}} - x^{*}_{{R}_{k}}) \\
    \end{bmatrix},\\
   \text{where}\;
   D_{k} &=
    \frac{\partial \mathbf{h}_{a}}{\partial \mathbf{h}_{b}}
    \begin{bmatrix}
    C^{\top}(\phi_{R_{k+\lambda}}) &        &                        \\
                           & \ddots &                        \\
                           &        & C^{\top}(\phi_{R_{k+\lambda}}) \\
    \end{bmatrix}

このモデルではノイズがなく、ランドマーク位置も不変であることを仮定しているため、  :math:`x^{*}_{{L_{i}}_{k}}=x^{*}_{{{L}_{i}}_{k+\lambda}},y^{*}_{{L_{i}}_{k}}=y^{*}_{{{L}_{i}}_{k+\lambda}}, i=1,...,N, \lambda=0,...,n` とおくことができる。結果として可観測行列のロボット状態に関連する部分は次のようになる。

.. math::
    &\breve{H}_{R_{k+\lambda}}\breve{\Phi}_{R_{k+\lambda-1}} ... \breve{\Phi}_{R_{k+1}}
    =
    D_{k+\lambda}
    \begin{bmatrix}
    -1     &        &   y^{*}_{{{L}_{1}}_{k}} - y^{*}_{{R}_{k}}  \\
           & -1     & -(x^{*}_{{{L}_{1}}_{k}} - x^{*}_{{R}_{k}}) \\
    \vdots & \vdots & \vdots                           \\
    -1     &        &   y^{*}_{{{L}_{N}}_{k}} - y^{*}_{{R}_{k}}  \\
           & -1     & -(x^{*}_{{{L}_{N}}_{k}} - x^{*}_{{R}_{k}}) \\
    \end{bmatrix}

可観測行列のランドマークに関連する部分は式 :eq:`h_b_derivative` より次のようになる。

.. math::
    &\breve{H}_{L_{k+\lambda}}
    =
    D_{k+\lambda}
    \begin{bmatrix}
    1 &   &        &   &   \\
      & 1 &        &   &   \\
      &   & \ddots &   &   \\
      &   &        & 1 &   \\
      &   &        &   & 1 \\
    \end{bmatrix}

以上より可観測行列の各行は次の式で計算できる。

.. math::
    &\begin{bmatrix}
    \begin{array}{c|c}
    \breve{H}_{R_{k+\lambda}}\breve{\Phi}_{R_{k+\lambda-1}} ... \breve{\Phi}_{R_{k}} & \breve{H}_{L_{k+\lambda}}
    \end{array}
    \end{bmatrix}
    =
    D_{k+\lambda}\breve{E},\\
    &\breve{E} =
    \begin{bmatrix}
    \begin{array}{ccc|ccccc}
    -1     &        &   y^{*}_{{{L}_{1}}_{k}} - y^{*}_{{R}_{k}}  & 1 &   &        &   &   \\
           & -1     & -(x^{*}_{{{L}_{1}}_{k}} - x^{*}_{{R}_{k}}) &   & 1 &        &   &   \\
    \vdots & \vdots & \vdots                             &   &   & \ddots &   &   \\
    -1     &        &   y^{*}_{{{L}_{N}}_{k}} - y^{*}_{{R}_{k}}  &   &   &        & 1 &   \\
           & -1     & -(x^{*}_{{{L}_{N}}_{k}} - x^{*}_{{R}_{k}}) &   &   &        &   & 1 \\
    \end{array}
    \end{bmatrix}

可観測行列は次のように書くことができる。

.. math::
   \breve{M}_{k} = \begin{bmatrix}
    \breve{H}_{k}  \\
    \breve{H}_{k+1} \breve{\Phi}_{k}  \\
    \breve{H}_{k+2} \breve{\Phi}_{k+1} \breve{\Phi}_{k}  \\
    \vdots  \\
    \breve{H}_{k+n} \breve{\Phi}_{k+n-1} ... \breve{\Phi}_{k+1} \breve{\Phi}_{k}
   \end{bmatrix}
   =
   \begin{bmatrix}
   D_{k}  &        &                \\
          & \ddots &                \\
          &        & D_{k+\lambda}  \\
   \end{bmatrix}
   \begin{bmatrix}
   \breve{E} \\
   \vdots \\
   \breve{E}
   \end{bmatrix}
   :label: ideal-observability-matrix

可観測行列 :math:`\breve{M}_{k}` のランクは行列 :math:`E` のランクに等しく、その値は :math:`2N` である。

可観測行列の零空間はシステムが観測不可能な空間と等しい。

.. math::
   \operatorname{Null}(\breve{M}_{k}) =
   \operatorname{\underset{col.}{span}}
   \begin{bmatrix}
    1 & 0 & -y_{R_{k}} \\
    0 & 1 & x_{R_{k}} \\
    0 & 0 & 1  \\
    1 & 0 & -y_{{L_{1}}_{k}} \\
    0 & 1 & x_{{L_{1}}_{k}} \\
    \vdots & \vdots \\
    1 & 0 & -y_{{L_{N}}_{k}} \\
    0 & 1 & x_{{L_{N}}_{k}} \\
   \end{bmatrix}

左2つの基底は並進に関する不確定性を意味しており、3つめの基底は回転に関する不確定性を表現している。

実際のEKFの可観測性
-------------------

実際のEKFの可観測行列を計算してみよう。まずは先ほどと同様に :math:`\Phi_{R_{k+n-1}} ... \Phi_{R_{k+1}} \Phi_{R_{k}}` を計算してみる。

.. math::
    \Phi_{R_{k+1}} \Phi_{R_{k}}
    &=
    \begin{bmatrix}
      1 &   & -(y_{R_{k+2|k+1}} - y_{R_{k+1|k+1}}) \\
        & 1 &  x_{R_{k+2|k+1}} - x_{R_{k+1|k+1}}   \\
        &   &                                   1
    \end{bmatrix}
    \begin{bmatrix}
      1 &   & -(y_{R_{k+1|k}} - y_{R_{k|k}}) \\
        & 1 &  x_{R_{k+1|k}} - x_{R_{k|k}}   \\
        &   &                          1
    \end{bmatrix} \\
    &=
    \begin{bmatrix}
      1 &   & -(y_{R_{k+2|k+1}} - y_{R_{k|k}} - \Delta y_{R_{k+1}}) \\
        & 1 &  x_{R_{k+2|k+1}} - x_{R_{k|k}} - \Delta x_{R_{k+1}}   \\
        &   &                            1
    \end{bmatrix}, \\
    \text{where} \quad
    \Delta x_{R_{k+1}} &= x_{R_{k+1|k+1}} - x_{R_{k+1|k}}  \\
    \Delta y_{R_{k+1}} &= y_{R_{k+1|k+1}} - y_{R_{k+1|k}}  \\

今度はUpdateステップの位置修正ぶんの項 :math:`\Delta y_{R_{k+1}}, \Delta x_{R_{k+1}}` が残ることに注意しよう。

可観測行列を構成する要素を計算する。

.. math::
    \Phi_{R_{k+\lambda-1}} ... \Phi_{R_{k+1}} \Phi_{R_{k}}
    &=
    \begin{bmatrix}
      1 &   & -(y_{R_{k+\lambda|k+\lambda-1}} - y_{R_{k|k}} - \sum_{j=k+1}^{k+\lambda-1} \Delta y_{R_j}) \\
        & 1 &  x_{R_{k+\lambda|k+\lambda-1}} - x_{R_{k|k}} - \sum_{j=k+1}^{k+\lambda-1} \Delta x_{R_j}   \\
        &   &                            1
    \end{bmatrix}, \\

.. math::
   H_{R_{k+\lambda}}
    &=
    \frac{\partial \mathbf{h}_{a}}{\partial \mathbf{h}_{b}}
    \begin{bmatrix}
    C^{\top}(\phi_{R_{k+\lambda|k+\lambda-1}}) &        &                        \\
                           & \ddots &                        \\
                           &        & C^{\top}(\phi_{R_{k+\lambda|k+\lambda-1}}) \\
    \end{bmatrix}
    \begin{bmatrix}
    -1     &        &   y_{{{L}_{1}}_{k+\lambda}} - y_{{R}_{k+\lambda|k+\lambda-1}}  \\
           & -1     & -(x_{{{L}_{1}}_{k+\lambda}} - x_{{R}_{k+\lambda|k+\lambda-1}}) \\
    \vdots & \vdots & \vdots                             \\
    -1     &        &   y_{{{L}_{N}}_{k+\lambda}} - y_{{R}_{k+\lambda|k+\lambda-1}}  \\
           & -1     & -(x_{{{L}_{N}}_{k+\lambda}} - x_{{R}_{k+\lambda|k+\lambda-1}}) \\
    \end{bmatrix} \\

可観測行列の :math:`\lambda+1` ブロック行目は次のようになる。

.. math::
    &\begin{bmatrix}
    \begin{array}{c|c}
    H_{R_{k+\lambda}}\Phi_{R_{k+\lambda-1}} ... \Phi_{R_{k}} & H_{L_{k+\lambda}}
    \end{array}
    \end{bmatrix}
    =\\
    &D_{k+\lambda|k+\lambda-1}
    \begin{bmatrix}
    \begin{array}{ccc|ccccc}
    -1     &        &   y_{{{L}_{1}}_{k+\lambda}} - y_{{R}_{k|k}} - \sum_{j=k+1}^{k+\lambda-1} \Delta y_{R_j}  & 1 &   &        &   &   \\
           & -1     & -(x_{{{L}_{1}}_{k+\lambda}} - x_{{R}_{k|k}} - \sum_{j=k+1}^{k+\lambda-1} \Delta x_{R_j}) &   & 1 &        &   &   \\
    \vdots & \vdots & \vdots                             &   &   & \ddots &   &   \\
    -1     &        &   y_{{{L}_{N}}_{k+\lambda}} - y_{{R}_{k|k}} - \sum_{j=k+1}^{k+\lambda-1} \Delta y_{R_j}  &   &   &        & 1 &   \\
           & -1     & -(x_{{{L}_{N}}_{k+\lambda}} - x_{{R}_{k|k}} - \sum_{j=k+1}^{k+\lambda-1} \Delta x_{R_j}) &   &   &        &   & 1 \\
    \end{array}
    \end{bmatrix}

可観測行列は

.. math::
    M_k =
    \begin{bmatrix}
    \begin{array}{c|c}
        H_{R_{k}} & H_{L_{k}}  \\
        H_{R_{k+1}}\Phi_{R_{k}} & H_{L_{k+1}}  \\
        H_{R_{k+2}}\Phi_{R_{k+1}}\Phi_{R_{k}} & H_{L_{k+2}}  \\
        \vdots \\
        H_{R_{k+n}}\Phi_{R_{k+n-1}}...\Phi_{R_{k+1}}\Phi_{R_{k}} & H_{L_{k+n}}  \\
    \end{array}
    \end{bmatrix}

で計算できるが、これは :math:`\sum_{j=k+1}^{k+\lambda-1} \Delta y_{R_j}, \sum_{j=k+1}^{k+\lambda-1} \Delta x_{R_j}` の項の影響を受けるため、式 :eq:`ideal-observability-matrix` で示されている理想的な可観測行列とは明らかに異なる零空間を有する。

.. math::
    \operatorname{Null}(M_k) = \operatorname{\underset{col.}{span}}
    \begin{bmatrix}
     1 & 0 \\
     0 & 1 \\
     0 & 0 \\
     1 & 0 \\
     0 & 1 \\
     \vdots & \vdots \\
     1 & 0 \\
     0 & 1 \\
    \end{bmatrix}

:math:`\operatorname{Null}(\breve{M}_k)` と比較してわかることは、零空間から回転に関する基底が消えたことである。これは通常のEKFでは理想的なモデルに比べて可観測な空間が増え、その影響を受けて **回転に関する不確定性が仮想的に減る** ことを意味する。すなわち、理想的なモデルさえヨー角が定まらないような状態遷移のパターンが存在するのに、実際のEKFではいかなる場合であっても観測値からヨー角を一意に定めることができてしまう。理想的なモデルには回転に関する不確定性が存在するが、通常のEKFではそれが消えてしまっている。これは結果としてヨー角に対応する共分散が減少することを意味する。
ヨー角に対応する共分散の過剰な減少はEKFの不安定化を招くことがすでに指摘されている [Julier2001]_ [Bailey2006]_ 。したがって、EKFの回転に関する不確定性を復活させることはEKFの安定化に寄与する。 

改善手法 (First Estimates Jacobian)
-----------------------------------

回転に関する不確定性が消えてしまう問題はEKFのUpdateステップにおけるXY座標のずれが蓄積してしまうことによって生じる。したがって、可観測な空間を理想的なEKFと一致させるためにはこのずれをなくしてしまえばよい。

:math:`\Phi_{R_{k}}` と :math:`H_{R_{k}}` を次のようにおくと、可観測行列のうちヨー角に対応する不確定性が復活する。

.. math::
   \Phi^{\prime}_{R_{k}} &=
   \begin{bmatrix}
   1 &   & -(y_{R_{k+1|k}} - y_{R_{k|k-1}}) \\
     & 1 &  x_{R_{k+1|k}} - x_{R_{k|k-1}}   \\
     &   &                                1 \\
   \end{bmatrix} \\
   H^{\prime}_{R_{k+\lambda}} &=
    D_{k+\lambda|k+\lambda-1}
    \begin{bmatrix}
    \begin{array}{ccc|ccccc}
    -1     &        &   y_{{{L}_{1}}_{k|k}} - y_{{R}_{k+\lambda|k+\lambda-1}}  & 1 &   &        &   &   \\
           & -1     & -(x_{{{L}_{1}}_{k|k}} - x_{{R}_{k+\lambda|k+\lambda-1}}) &   & 1 &        &   &   \\
    \vdots & \vdots & \vdots                             &   &   & \ddots &   &   \\
    -1     &        &   y_{{{L}_{N}}_{k|k}} - y_{{R}_{k+\lambda|k+\lambda-1}}  &   &   &        & 1 &   \\
           & -1     & -(x_{{{L}_{N}}_{k|k}} - x_{{R}_{k+\lambda|k+\lambda-1}}) &   &   &        &   & 1 \\
    \end{array}
    \end{bmatrix} \\

主な変更点は Prediction ステップで得られた状態で :math:`\Phi^{\prime}_{R_{k}}` を計算していることと、最初の時刻 :math:`k` で観測されたランドマークの値を用いて :math:`H^{\prime}_{R_{k+\lambda}}` を計算していることである。

実際に可観測行列を計算してみよう。

:math:`\Phi^{\prime}_{R_{k}}` の性質より、

.. math::
   \Phi^{\prime}_{R_{k+1}}\Phi^{\prime}_{R_{k}}
   &=
   \begin{bmatrix}
   1 &   & -(y_{R_{k+2|k+1}} - y_{R_{k+1|k}}) \\
     & 1 &  x_{R_{k+2|k+1}} - x_{R_{k+1|k}}   \\
     &   &                                1 \\
   \end{bmatrix}
   \begin{bmatrix}
   1 &   & -(y_{R_{k+1|k}} - y_{R_{k|k-1}}) \\
     & 1 &  x_{R_{k+1|k}} - x_{R_{k|k-1}}   \\
     &   &                                1 \\
   \end{bmatrix} \\
   &=
   \begin{bmatrix}
   1 &   & -(y_{R_{k+2|k+1}} - y_{R_{k|k-1}}) \\
     & 1 &  x_{R_{k+2|k+1}} - x_{R_{k|k-1}}   \\
     &   &                                1 \\
   \end{bmatrix}, \\
   \Phi^{\prime}_{R_{k+\lambda-1}}...\Phi^{\prime}_{R_{k+1}}\Phi^{\prime}_{R_{k}}
   &=
   \begin{bmatrix}
   1 &   & -(y_{R_{k+\lambda|k+\lambda-1}} - y_{R_{k|k-1}}) \\
     & 1 &  x_{R_{k+\lambda|k+\lambda-1}} - x_{R_{k|k-1}}   \\
     &   &                                1 \\
   \end{bmatrix} \\

となるため、可観測行列の :math:`\lambda+1` 行目は

.. math::
   &\begin{bmatrix}
   \begin{array}{c|c}
   H^{\prime}_{R_{k+\lambda}}\Phi^{\prime}_{R_{k+\lambda-1}}...\Phi^{\prime}_{R_{k+1}}\Phi^{\prime}_{R_{k}} & H^{\prime}_{L_{k+\lambda}}
   \end{array}
   \end{bmatrix} = D_{k+\lambda|k+\lambda-1}E^{\prime}, \\
   &E^{\prime} = \begin{bmatrix}
    \begin{array}{ccc|ccccc}
    -1     &        &   y_{{{L}_{1}}_{k|k}} - y_{{R}_{k|k-1}}  & 1 &   &        &   &   \\
           & -1     & -(x_{{{L}_{1}}_{k|k}} - x_{{R}_{k|k-1}}) &   & 1 &        &   &   \\
    \vdots & \vdots & \vdots                                                   &   &   & \ddots &   &   \\
    -1     &        &   y_{{{L}_{N}}_{k|k}} - y_{{R}_{k|k-1}}  &   &   &        & 1 &   \\
           & -1     & -(x_{{{L}_{N}}_{k|k}} - x_{{R}_{k|k-1}}) &   &   &        &   & 1 \\
    \end{array}
    \end{bmatrix} \\

と計算できる。したがって可観測行列を構成すると、

.. math::
   M^{\prime}_{k}
   &=
    \begin{bmatrix}
    \begin{array}{c|c}
        H^{\prime}_{R_{k}} & H^{\prime}_{L_{k}}  \\
        H^{\prime}_{R_{k+1}}\Phi^{\prime}_{R_{k}} & H^{\prime}_{L_{k+1}}  \\
        H^{\prime}_{R_{k+2}}\Phi^{\prime}_{R_{k+1}}\Phi^{\prime}_{R_{k}} & H^{\prime}_{L_{k+2}}  \\
        \vdots \\
        H^{\prime}_{R_{k+n}}\Phi^{\prime}_{R_{k+n-1}}...\Phi^{\prime}_{R_{k+1}}\Phi^{\prime}_{R_{k}} & H^{\prime}_{L_{k+n}}  \\
    \end{array}
    \end{bmatrix} \\
   &=
   \begin{bmatrix}
   D_{k|k-1}  &        &                \\
              & \ddots &                \\
              &        & D_{k+\lambda|k+\lambda-1}  \\
   \end{bmatrix}
   \begin{bmatrix}
   \breve{E}^{\prime} \\
   \vdots \\
   \breve{E}^{\prime}
   \end{bmatrix}

となり、理想的なシステムの可観測行列 :eq:`ideal-observability-matrix` と同じランクおよび零空間を有することがわかる。

.. math::
   \operatorname{Null}(M^{\prime}_{k}) =
   \operatorname{\underset{col.}{span}}
   \begin{bmatrix}
    1 & 0 & -y_{R_{k|k-1}} \\
    0 & 1 & x_{R_{k|k-1}} \\
    0 & 0 & 1  \\
    1 & 0 & -y_{{L_{1}}_{k|k}} \\
    0 & 1 & x_{{L_{1}}_{k|k}} \\
    \vdots & \vdots \\
    1 & 0 & -y_{{L_{N}}_{k|k}} \\
    0 & 1 & x_{{L_{N}}_{k|k}} \\
   \end{bmatrix}

この操作によりヨー角に関する不確定性が復活し、共分散の過剰な減少を防ぎ、EKFの安定性が向上する。


Appendix
========

.. _solving_state_equation:

状態方程式の解法
----------------

状態方程式

.. math::
   \frac{d}{dt}\mathbf{x}(t) = A \mathbf{x}(t) + B \mathbf{u}(t)

について、右辺第一項を移項し両辺に :math:`e^{-At}` をかける。

.. math::
   \frac{d}{dt}\mathbf{x}(t) - A \mathbf{x}(t) &= B \mathbf{u}(t)  \\
   e^{-At} \frac{d}{dt}\mathbf{x}(t) - e^{-At} A \mathbf{x}(t) &= e^{-At} B \mathbf{u}(t)
   :label: state-equation-times-exp-minus-at

| 正方行列 :math:`A` および行列の指数関数 :math:`e^{At}` について :math:`\frac{d}{dt} e^{At} = A e^{At} = e^{At} A` が成り立つ。
| 積の微分法を用いると

.. math::
   \frac{d}{dt}(e^{-At} \mathbf{x}(t)) &= - e^{-At} A \mathbf{x}(t) + e^{At} \frac{d}{dt} \mathbf{x}(t)  \\

となる。これは :eq:`state-equation-times-exp-minus-at` の左辺に一致する。

.. math::
   \frac{d}{dt}(e^{-At} \mathbf{x}(t)) = e^{-At} B \mathbf{u}(t)

.. math::
   d(e^{-At} \mathbf{x}(t)) = e^{-At} B \mathbf{u}(t) dt

両辺を :math:`t_{0}` から :math:`t_{1}` まで積分する。

.. math::
   \int_{t_{0}}^{t_{1}} d(e^{-At} \mathbf{x}(t)) &= e^{-A t_{1}} \mathbf{x}(t_{1}) - e^{-A t_{0}} \mathbf{x}(t_{0}) = \int_{t_{0}}^{t_{1}} e^{-At} B \mathbf{u}(t) dt \\
   e^{-A t_{1}} \mathbf{x}(t_{1}) &= e^{-A t_{0}} \mathbf{x}(t_{0}) + \int_{t_{0}}^{t_{1}} e^{-At} B \mathbf{u}(t) dt

両辺に :math:`e^{A t_{1}}` をかければ解が得られる。

.. math::
   \mathbf{x}(t_{1})
   &= e^{A t_{1}} e^{-A t_{0}} \mathbf{x}(t_{0}) + e^{A t_{1}} \int_{t_{0}}^{t_{1}} e^{-At} B \mathbf{u}(t) dt \\
   &= e^{A (t_{1} - t_{0})} \mathbf{x}(t_{0}) + \int_{t_{0}}^{t_{1}} e^{A(t_{1} - t)} B \mathbf{u}(t) dt \\

変数の衝突があると混乱を招くため、積分変数を :math:`t` ではなく :math:`\tau` としておこう。

.. math::
   \mathbf{x}(t_{1}) = e^{A (t_{1} - t_{0})} \mathbf{x}(t_{0}) + \int_{t_{0}}^{t_{1}} e^{A(t_{1} - \tau)} B \mathbf{u}(\tau) d\tau

.. _representing_higher_order_derivatives:

高次導関数の低次導関数による表現
--------------------------------

観測モデルを :math:`y(t) = t \sin(t)` としたとき、4次より高次の導関数は3次までの導関数の線型結合で表すことができる。したがって :math:`k = 4` である。

.. math::
    y(t) &= t \sin(t) \\
    \frac{dy(t)}{dt} &= \sin(t) + t \cos(t) \\
    \frac{d^{2}y(t)}{dt^{2}} &= 2 \cos(t) - t \sin(t)  \\
    \frac{d^{3}y(t)}{dt^{3}} &= -3 \sin(t) - t \cos(t) \\
    \frac{d^{4}y(t)}{dt^{4}} &= t \sin(t) - 4 \cos(t) = -y(t) - 2\frac{d^{2}y(t)}{dt^2} \\
    \frac{d^{5}y(t)}{dt^{5}} &= 5 \sin(t) + t \cos(t) = -\frac{dy(t)}{dt} - 2 \frac{d^{3}y(t)}{dt^{3}} \\
    \frac{d^{6}y(t)}{dt^{6}} &= \; ...


.. [Julier2001] Julier, Simon J., and Jeffrey K. Uhlmann. "A counter example to the theory of simultaneous localization and map building." Proceedings 2001 ICRA. IEEE International Conference on Robotics and Automation (Cat. No. 01CH37164). Vol. 4. IEEE, 2001.
.. [Bailey2006] Bailey, Tim, et al. "Consistency of the EKF-SLAM algorithm." 2006 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2006.
.. [Huang2009] Huang, Guoquan P., Anastasios I. Mourikis, and Stergios I. Roumeliotis. "A first-estimates Jacobian EKF for improving SLAM consistency." Experimental Robotics: The Eleventh International Symposium. Springer Berlin Heidelberg, 2009.
.. [Huang2008] Huang, Guoquan P., Anastasios I. Mourikis, and Stergios I. Roumeliotis. "Analysis and improvement of the consistency of extended Kalman filter based SLAM." 2008 IEEE International Conference on Robotics and Automation. IEEE, 2008.
