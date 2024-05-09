======================
角速度バイアス推定方法
======================

:著者: 石田 岳志

概要
====

IMUは低価格かつ小型のオドメトリ推定プラットフォームを構築するために有用である。また、LiDARやカメラに対する補完的特性により、それらと組み合わせて利用することで高精度な位置推定を実現することができる。一方で観測値にバイアスが乗るため、正確なオドメトリ推定を実現するには、観測値に対してバイアス補正を行う必要がある。
ここではIMUの観測値のうち角速度に対するバイアス補正手法を紹介する。


ここではLiDARやカメラなどで地図上での位置を把握できること、あるいはある時間間隔におけるセンサの移動量を観測できることを仮定する。本手法ではどちらを使ってもよいので、ひとまずカメラを用いることとしよう。

IMU座標系(ボディ座標系)とカメラ座標系の間の回転を :math:`R_{CB} \in \mathbb{R}^{3}` とする。一般的に、位置推定を行っている間はIMUとカメラの位置関係は変わらないため、 :math:`R_{CB}` は不変な値として扱われる。
時刻 :math:`i` に得られたカメラ座標系からワールド座標系への回転を :math:`R^{i}_{WC} \in \mathbb{R}^{3}` とする。
時刻 :math:`i` におけるIMU座標系からワールド座標系への回転 :math:`R^{i}_{WB} \in \mathbb{R}^{3}` はこれらの積で表される。

.. math::
   R^{i}_{WB} = R^{i}_{WC}\cdot R_{CB}

理想的なケース
==============

| 一般的にIMUのほうがカメラ(もしくはLiDAR)よりもセンサの周波数が数倍〜数十倍も高い。これはつまり、カメラが1周期の推定を行っている間に、IMUは数十〜数百の観測値を得られることを意味する。
| たとえば時刻 :math:`i` および :math:`j` それぞれにおいてカメラの観測値を用いてボディ回転

.. math::
   R^{i}_{WB} &= R^{i}_{WC}\cdot R_{CB}  \\
   R^{j}_{WB} &= R^{j}_{WC}\cdot R_{CB}


を得られるとしよう。IMUはカメラよりも動作周波数が高いため、時刻 :math:`i` と :math:`j` の間に、IMUの回転速度を表現する角速度ベクトル :math:`\mathbf{\omega}_{k} \in \mathbb{R}^{3}, k=i,..,j-1` を数十個〜数百個得ることができる。

ここで角速度ベクトルに対する指数写像 :math:`\mathrm{Exp}` およびその逆写像 :math:`\mathrm{Log}` を定義する。

.. math::
   &\mathrm{Exp}: \mathbb{R}^{3} \rightarrow \mathrm{SO}(3), \;\;
   &&\mathrm{Exp}(\mathbf{\omega}) := \exp([\mathbf{\omega}]_{\times}) = \sum_{k=0}^{\infty}\frac{1}{k!}([\mathbf{\omega}]_{\times})^{k} \\
   &\mathrm{Log}: \mathrm{SO}(3) \rightarrow \mathbb{R}^{3}, \;\;
   &&\mathrm{Log}(R) := \mathrm{Exp}^{-1}(R) \\

:math:`[\cdot]_{\times}` は角速度ベクトルをリー代数 :math:`\mathfrak{so}(3)` の元に対応させる演算を表す。

.. math::
    [\mathbf{\omega}]_{\times} = \begin{bmatrix}
   0 & -\omega_{3} & \omega_{2}  \\
   \omega_{3} & 0 & -\omega_{1}  \\
   -\omega_{2} & \omega_{1} & 0
   \end{bmatrix}


| IMU観測値の取得間隔を :math:`\Delta t` とする。
| もしIMUから得られる角速度ベクトル :math:`\mathbf{\omega}^{m}_{k}` がIMUの真の角速度 :math:`\mathbf{\omega}_{k}` に等しいならば、 :math:`R^{i}_{WB}` と :math:`R^{j}_{WB}` の間の関係を次のように記述できるはずである。

.. math::
   R^{j}_{WB} = R^{i}_{WB} \cdot \prod^{j-1}_{k=i}\mathrm{Exp}(\mathbf{\omega}^{m}_{k} \Delta t)

実際のケース
============

| 実際には IMU の観測値にはノイズとバイアスが乗るため、観測値をそのまま積分しても回転変化を正確に記述できない。
| すなわち、IMUから得られる角速度の観測値 :math:`\mathbf{\omega}^{m}_{k}` は、真の角速度ベクトル :math:`\mathbf{\omega}_{k}` 、IMUの角速度バイアス :math:`\mathbf{b}^{\omega}_{k} \in \mathbb{R}^{3}` 、ノイズ :math:`\mathbf{\eta}^{\omega}_{k} \in \mathbb{R}^{3}` を用いて次のように表される。

.. math::
    \mathbf{\omega}^{m}_{k} = \mathbf{\omega}_{k} + \mathbf{b}^{\omega}_{k} + \mathbf{\eta}^{\omega}_{k}

| ノイズ :math:`\mathbf{\eta}^{\omega}_{k}` は一般的に平均ゼロの正規分布に従う。

.. math::
   \mathbf{\eta}^{\omega}_{k} \sim \mathcal{N}(\mathbf{0}, Q^{\omega}), \; Q^{\omega} \in \mathbb{R}^{3 \times 3}

| IMUは半導体なので、真の値に対してバイアスが乗ってしまう。また、バイアスの要因は周囲の温度変化などであるため、バイアスの値そのものも時間変化する。しかしながら、バイアス推定に必要なIMUおよびカメラの観測値は数秒〜十数秒ぶんあれば十分であるため、この時間間隔においてはバイアスが一定値であることを仮定する。時刻を表す添字を除去して :math:`\mathbf{b}^{\omega}_{k} = \mathbf{b}^{\omega}` と表記することにしよう。以上を踏まえて真の観測値 :math:`\mathbf{\omega}_{k}` を記述する。

.. math::
    \mathbf{\omega}_{k} = \mathbf{\omega}^{m}_{k} - \mathbf{b}^{\omega} - \mathbf{\eta}^{\omega}_{k}


我々の目的は真の角速度バイアス :math:`\mathbf{b}^{\omega}` を推定することである。真の角速度バイアスが推定できれば、角速度の観測値 :math:`\mathbf{\omega}^{m}_{k}` を補正して回転量を正確に計算できる。



手法
====

角速度バイアスについて誤差関数 :math:`E_{ij}` を定め、この誤差関数を最小化させるようなバイアスの値をGauss-Newton法で探索する。

.. math::
    E_{ij}(\hat{\mathbf{b}}^{\omega}) &= || \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega}) ||^{2}, \\
    \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega}) &= \mathrm{Log}({R^{j}_{WB}}^{\top} \cdot R^{i}_{WB} \cdot \prod^{j-1}_{k=i}\mathrm{Exp}((\mathbf{\omega}^{m}_{k} - \hat{\mathbf{b}}^{\omega}) \Delta t))

なお、真の角速度バイアス :math:`\mathbf{b}^{\omega}` と区別するため、ここでは角速度バイアスの推定値を :math:`\hat{\mathbf{b}}^{\omega}` と表記している。

誤差関数 :math:`E_{ij}(\hat{\mathbf{b}}^{\omega})` を最小化するようなバイアスの値をGauss-Newton法で探索するには残差 :math:`\mathbf{r}_{ij}` を線形近似する必要がある。すなわち、Jacobian :math:`J_{ij}` を用いて

.. math::
    \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega} + \Delta \hat{\mathbf{b}}^{\omega}) \approx \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega}) + J_{ij} \Delta \hat{\mathbf{b}}^{\omega}

と表現したい。Jacobianを計算することができれば、Gauss-Newton法の更新式に従い、誤差関数を最小化するような :math:`\hat{\mathbf{b}}^{\omega}` を見つけることができる。

.. math::
    \hat{\mathbf{b}}^{\omega} \leftarrow \hat{\mathbf{b}}^{\omega} - (J_{ij}^{\top}J_{ij})^{-1}J_{ij}^{\top}\mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega})

残差の線形近似
~~~~~~~~~~~~~~

我々の関心は、 :math:`\hat{\mathbf{b}}^{\omega}` を :math:`\Delta \hat{\mathbf{b}}^{\omega}` だけ大きくしたときに、残差 :math:`\mathbf{r}_{ij}` がどのように変化するかである。

.. math::
   \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega} + \Delta \hat{\mathbf{b}}^{\omega})
   = \mathrm{Log}({R^{j}_{WB}}^{\top} \cdot R^{i}_{WB} \cdot \prod^{j-1}_{k=i} \mathrm{Exp}([\mathbf{\omega}^{m}_{k} - (\hat{\mathbf{b}}^{\omega} + \Delta \hat{\mathbf{b}}^{\omega})] \Delta t))
   :label: error-function

表記の煩雑さを低減するため、 :math:`\hat{\mathbf{\omega}}^{m}_{k} = \mathbf{\omega}^{m}_{k} - \hat{\mathbf{b}}^{\omega}` とおく。

.. math::
   \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega} + \Delta \hat{\mathbf{b}}^{\omega})
   &= \mathrm{Log}({R^{j}_{WB}}^{\top} \cdot R^{i}_{WB} \cdot \prod^{j-1}_{k=i}\mathrm{Exp}([\hat{\mathbf{\omega}}^{m}_{k} - \Delta \hat{\mathbf{b}}^{\omega}] \Delta t)) \\
   &= \mathrm{Log}({R^{j}_{WB}}^{\top} \cdot R^{i}_{WB} \cdot \prod^{j-1}_{k=i}\mathrm{Exp}(\hat{\mathbf{\omega}}^{m}_{k}\Delta t - \Delta \hat{\mathbf{b}}^{\omega} \Delta t))

さて、行列の指数関数には一般に指数法則が成立しない。n次元ベクトル :math:`\mathbf{a}, \mathbf{b} \in \mathbb{R}^{n}` について、一般に

.. math::
   \mathrm{Exp}(\mathbf{a} + \mathbf{b}) \neq \mathrm{Exp}(\mathbf{a})\mathrm{Exp}(\mathbf{b})

である。

:math:`\mathbf{a}` と :math:`\mathbf{b}` が互いに線形従属である場合にのみ指数法則

.. math::
   \mathrm{Exp}(\mathbf{a} + \mathbf{b}) = \mathrm{Exp}(\mathbf{a}) \cdot \mathrm{Exp}(\mathbf{b})

が成立する。したがって、式 :eq:`error-function` の :math:`\mathrm{Exp}` の積に対して指数法則を適用することができない。

| 一方で、一般に :math:`||\mathbf{b}||` が小さいとき、リー代数の和の指数写像は次のように近似できる

.. math::
   \mathrm{Exp}(\mathbf{a} + \mathbf{b}) \approx \mathrm{Exp}(\mathbf{a}) \cdot \mathrm{Exp}(J_{r}(\mathbf{a}) \cdot \mathbf{b})

が成立する。ここで :math:`J_{r}` は right Jacobian と呼ばれるものであり、解析的に計算できる。

これを利用すると、残差 :math:`\mathbf{r}_{ij}` は

.. math::
   \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega} + \Delta \hat{\mathbf{b}}^{\omega})
   \approx \mathrm{Log}({R^{j}_{WB}}^{\top} \cdot R^{i}_{WB} \cdot \prod^{j-1}_{k=i}\mathrm{Exp}(\hat{\mathbf{\omega}}^{m}_{k} \Delta t)\cdot \mathrm{Exp}(-J_{r}(\hat{\mathbf{\omega}}^{m}_{k} \Delta t) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t))

と書ける。読みやすさのために :math:`\hat{R}^{m}_{k} = \mathrm{Exp}(\hat{\mathbf{\omega}}^{m}_{k} \Delta t), \; \hat{\mathbf{\theta}}^{m}_{k} = \hat{\mathbf{\omega}}^{m}_{k} \Delta t` とおこう。

.. math::
   \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega} + \Delta \hat{\mathbf{b}}^{\omega})
   \approx \mathrm{Log}({R^{j}_{WB}}^{\top} \cdot R^{i}_{WB} \cdot \prod^{j-1}_{k=i} \left[ \hat{R}^{m}_{k} \cdot \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \right])


さて、 :math:`\mathrm{Exp}` には次の面白い性質がある。

    :math:`\forall \mathbf{\omega} \in \mathbb{R}^{3}, \forall R \in \mathrm{SO}(3)` について、

    .. math::
       \mathrm{Exp}(R\cdot\mathbf{\omega}) &= R\cdot\mathrm{Exp}(\mathbf{\omega})\cdot R^{\top} \\
       R^{\top}\cdot\mathrm{Exp}(R\cdot \mathbf{\omega}) &= \mathrm{Exp}(\mathbf{\omega}) \cdot R^{\top} \\
       \mathrm{Exp}(R^{\top}\cdot\mathbf{\omega}) &= R^{\top}\cdot\mathrm{Exp}(\mathbf{\omega})\cdot R \\
       R\cdot\mathrm{Exp}(R^{\top}\cdot\mathbf{\omega}) &= \mathrm{Exp}(\mathbf{\omega}) \cdot R

    が成り立つ。

残差 :math:`\mathbf{r}_{ij}` に対してこれを適用するため、総乗記号の中身を展開する。

例として、 :math:`i=1, j=5` の場合を示そう。

.. math::
   &\prod^{4}_{k=1}\hat{R}^{m}_{k} \cdot \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \\\\
   =
   & \hat{R}^{m}_{1} \cdot \\
   & \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{1}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \hat{R}^{m}_{2} \cdot \\
   & \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{2}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \hat{R}^{m}_{3} \cdot \\
   & \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{3}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \hat{R}^{m}_{4} \cdot \\
   & \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{4}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \\\\
   =
   & \hat{R}^{m}_{1} \cdot \hat{R}^{m}_{2} \cdot \\
   & \mathrm{Exp}(-{\hat{R}^{m}_{2}}^{\top} \cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{1}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \hat{R}^{m}_{3} \cdot \\
   & \mathrm{Exp}(-{\hat{R}^{m}_{3}}^{\top} \cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{2}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \hat{R}^{m}_{4} \cdot \\
   & \mathrm{Exp}(-{\hat{R}^{m}_{4}}^{\top} \cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{3}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \\
   & \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{4}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \\\\
   =
   & \hat{R}^{m}_{1} \cdot \hat{R}^{m}_{2} \cdot \hat{R}^{m}_{3} \cdot \\
   & \mathrm{Exp}(-{\hat{R}^{m}_{3}}^{\top} \cdot {\hat{R}^{m}_{2}}^{\top} \cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{1}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \hat{R}^{m}_{4} \cdot \\
   & \mathrm{Exp}(-{\hat{R}^{m}_{4}}^{\top} \cdot {\hat{R}^{m}_{3}}^{\top} \cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{2}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \\
   & \mathrm{Exp}(-{\hat{R}^{m}_{4}}^{\top} \cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{3}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \\
   & \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{4}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \\\\
   =
   & \hat{R}^{m}_{1} \cdot \hat{R}^{m}_{2} \cdot \hat{R}^{m}_{3} \cdot \hat{R}^{m}_{4} \cdot \\
   & \mathrm{Exp}(-{\hat{R}^{m}_{4}}^{\top} \cdot {\hat{R}^{m}_{3}}^{\top} \cdot {\hat{R}^{m}_{2}}^{\top} \cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{1}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \\
   & \mathrm{Exp}(-{\hat{R}^{m}_{4}}^{\top} \cdot {\hat{R}^{m}_{3}}^{\top} \cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{2}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \\
   & \mathrm{Exp}(-{\hat{R}^{m}_{4}}^{\top} \cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{3}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \cdot \\
   & \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{4}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t)

このようにして、 :math:`\Delta \hat{\mathbf{b}}^{\omega}` に依存する部分とそうでない部分を分離することができる。

一般的に書けば次のようになる。

.. math::
   \prod^{j-1}_{k=i}\hat{R}^{m}_{k} \cdot \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t)
   &=
   \hat{R}^{m}_{i,j-1}
   \prod_{k=i}^{j-1}
   \mathrm{Exp}(-{\hat{R}^{m}_{k+1,j-1}}^{\top}\cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t), \\
   &\text{where} \;\; \hat{R}^{m}_{k,j-1} = \prod^{j-1}_{l=k} \hat{R}^{m}_{l}

以上の結果を利用すれば、残差 :math:`\mathbf{r}_{ij}` は次のようになる。

.. math::
    \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega} + \Delta \hat{\mathbf{b}}^{\omega})
   &\approx \mathrm{Log}({R^{j}_{WB}}^{\top} \cdot R^{i}_{WB} \cdot \prod^{j-1}_{k=i} \left[ \hat{R}^{m}_{k} \cdot \mathrm{Exp}(-J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t) \right]) \\
   &=
   \mathrm{Log}({R^{j}_{WB}}^{\top} \cdot R^{i}_{WB} \cdot \hat{R}^{m}_{i,j-1} \cdot
   \prod_{k=i}^{j-1}
   \mathrm{Exp}(-{\hat{R}^{m}_{k+1,j-1}}^{\top}\cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t)) \\

さて、任意の :math:`\mathbf{x}\in\mathrm{R}^{3}` に対して :math:`||J_{r}(\hat{\mathbf{\theta}}^{m}_{k})\mathbf{x}|| \leq ||\mathbf{x}||` となる(参考: `行列ノルム`_)ので、次が成り立つ。

.. math::
   ||{\hat{R}^{m}_{k+1,j-1}}^{\top}\cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t|| = ||J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t|| \leq ||\Delta \hat{\mathbf{b}}^{\omega} \Delta t||

また、 :math:`\mathbf{a}, \mathbf{b} \in \mathrm{R}^{3}` について、これらのノルムが十分に小さいとき、次が成り立つ

.. math::
    \mathrm{Log}(\mathrm{Exp}(\mathbf{a} + \mathbf{b})) = \mathbf{a} + \mathbf{b} + O(||\mathbf{a}||^{2}, ||\mathbf{b}||^{2})

したがって、 :math:`||\Delta \hat{\mathbf{b}}^{\omega} \Delta t||` が十分に小さいならば、残差 :math:`\mathbf{r}_{ij}` をさらに次のように近似できる。

.. math::
    \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega} + \Delta \hat{\mathbf{b}}^{\omega})
   &\approx
   \mathrm{Log}({R^{j}_{WB}}^{\top} \cdot R^{i}_{WB} \cdot \hat{R}^{m}_{i,j-1} \cdot
   \mathrm{Exp}(-\sum_{k=i}^{j-1} {\hat{R}^{m}_{k+1,j-1}}^{\top}\cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t)) \\

ベクトル :math:`\mathbf{a} \in \mathbb{R}^{3}` と微小量 :math:`\Delta \mathbf{a} \in \mathbb{R}^{3}` の間には次の関係が成り立つ。

.. math::
   \mathrm{Log}(\mathrm{Exp}(\mathbf{a})\mathrm{Exp}(\Delta \mathbf{a})) = \mathbf{a} + J_{r}^{-1}(\mathbf{a})\Delta \mathbf{a}

:math:`\mathbf{\xi}_{ij} = \mathrm{Log}({R^{j}_{WB}}^{\top} \cdot R^{i}_{WB} \cdot \hat{R}^{m}_{i,j-1})` としてこの関係性を利用すると、Gauss-Newton 法の Jacobian が導出できる。

.. math::
    \mathbf{r}_{ij}(\hat{\mathbf{b}}^{\omega} + \Delta \hat{\mathbf{b}}^{\omega})
   &\approx
   \mathrm{Log}(\mathrm{Exp}(\mathbf{\xi}_{ij}) \cdot
   \mathrm{Exp}(-\sum_{k=i}^{j-1} {\hat{R}^{m}_{k+1,j-1}}^{\top}\cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t)) \\
   &\approx
   \mathbf{\xi}_{ij} -
   J_{r}^{-1}(\mathbf{\xi}_{ij})\left[
   \sum_{k=i}^{j-1} {\hat{R}^{m}_{k+1,j-1}}^{\top}\cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{k}) \cdot \Delta \hat{\mathbf{b}}^{\omega} \Delta t
   \right ] \\
   &=
   \mathbf{\xi}_{ij} -
   \Delta t \cdot J_{r}^{-1}(\mathbf{\xi}_{ij}) \left[
   \sum_{k=i}^{j-1} {\hat{R}^{m}_{k+1,j-1}}^{\top}\cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{k})
   \right ] \cdot \Delta \hat{\mathbf{b}}^{\omega}  \\
   &=
   \mathbf{\xi}_{ij} -
   J_{ij} \cdot \Delta \hat{\mathbf{b}}^{\omega}, \\
   \text{where} \;\; J_{ij} &=  \Delta t \cdot J_{r}^{-1}(\mathbf{\xi}_{ij}) \left[
   \sum_{k=i}^{j-1} {\hat{R}^{m}_{k+1,j-1}}^{\top}\cdot J_{r}(\hat{\mathbf{\theta}}^{m}_{k})
   \right]

:math:`\mathrm{SO}(3)` の Right Jacobian
========================================

:math:`\mathrm{SO}(3)` の right Jacobian :math:`J_{r}(\mathbf{\theta})` は次のように計算される。

.. math::
   J_{r}(\mathbf{\theta})
   = I
   - \frac{1 - \cos(||\mathbf{\theta}||)}{||\mathbf{\theta}||^{2}}[\mathbf{\theta}]_{\times}
   + \frac{||\mathbf{\theta}|| - \sin(||\mathbf{\theta}||)}{||\mathbf{\theta}||^{3}}[\mathbf{\theta}]_{\times}^{2}


.. _行列ノルム:

行列ノルム
~~~~~~~~~~


:math:`||J_{r}(\mathbf{\theta})||` は :math:`J_{r}(\mathbf{\theta})^{\top}J_{r}(\mathbf{\theta})` の最大固有値の平方根で与えられる。

:math:`k = ||\mathbf{\theta}||` とおいて :math:`J_{r}(\mathbf{\theta})^{\top}J_{r}(\mathbf{\theta})` の固有値を計算する。:math:`[\mathbf{\theta}]_{\times}^{\top} = -[\mathbf{\theta}]_{\times}` より、

.. math::
    J_{r}(\mathbf{\theta})^{\top}J_{r}(\mathbf{\theta})
    &= \left(I + \frac{1 + \cos(k)}{k}[\mathbf{\theta}]_{\times} + \frac{k - \sin(k)}{k}[\mathbf{\theta}]_{\times}^{2}\right)
       \left(I - \frac{1 + \cos(k)}{k}[\mathbf{\theta}]_{\times} + \frac{k - \sin(k)}{k}[\mathbf{\theta}]_{\times}^{2}\right) \\
    &= I + \left[2\frac{k - \sin(k)}{k^{3}} - \left(\frac{1 - \cos(k)}{k^{2}}\right)^{2}\right][\mathbf{\theta}]_{\times}^{2}
    + \left[\frac{k - \sin(k)}{k^{3}}\right]^{2}[\mathbf{\theta}]_{\times}^{4} \\

関係性 :math:`[\mathbf{\theta}]_{\times}^{4} = -k^{2}[\mathbf{\theta}]_{\times}^{2}` を用いると、

.. math::
    J_{r}(\mathbf{\theta})^{\top}J_{r}(\mathbf{\theta}) = I + \frac{1}{k^{4}}\left(k^2 + 2\cos(k)- 2 \right) [\mathbf{\theta}]_{\times}^{2} \\

が得られる。この固有値 :math:`\lambda` は

.. math::
    \det(J_{r}(\mathbf{\theta})^{\top}J_{r}(\mathbf{\theta}) - \lambda I)
    &= -\frac{1}{k^{4}}(\lambda - 1)\left[k^{2}\lambda + 2\cos(t) - 2\right]^{2} \\
    &= 0

の解として与えられ、結果として

.. math::
   \lambda = 1,\;\frac{2}{k^{2}}(1-\cos(k))

が得られる。なお、 :math:`\frac{2}{k^{2}}(1-\cos(k))` は :math:`k = 0` において最大値 :math:`1` をとる。

以上より、 :math:`J_{r}(\mathbf{\theta})^{\top}J_{r}(\mathbf{\theta})` の最大固有値が :math:`1` であることから :math:`||J_{r}(\mathbf{\theta})|| = 1` であり、この結果を利用して :math:`||J_{r}(\mathbf{\theta})\mathbf{x}||` の上界を与えることができる。

.. math::
    \forall\mathbf{x} \in \mathrm{R}^{3},\; ||J_{r}(\mathbf{\theta})\mathbf{x}|| \leq ||J_{r}(\mathbf{\theta})|| \cdot ||\mathbf{x}|| = ||\mathbf{x}||

