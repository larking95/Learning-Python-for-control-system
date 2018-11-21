# python-controlを用いたVRFTによるモデルフリー制御器設計
# python-control のドキュメンテーション(in English)
#   https://python-control.readthedocs.io/en/latest/

from control import matlab as crl
from matplotlib import pyplot as plt
import numpy as np


def main():
    # 制御対象を定義
    J = 0.6     # 慣性
    D = 0.2     # 粘性
    Kt = 0.15   # トルク定数
    P = crl.tf([Kt], [J, D, 0])     # モータ角度の電流制御

    # 参照モデルの定義
    M = crl.tf([50], [1, 50])

    # 制御対象と参照モデルのゲイン線図
    plt.figure(1)
    crl.bode([P, M])
    plt.legend(["Plant", "Ref. model"])

    # prbsを生成する
    plt.figure(2)
    plt.plot(prbs(4))

    plt.show()


def prbs(n_step):
    generation_polynominals = \
     {3: [1, 3],
      4: [1, 4],
      5: [2, 5],
      6: [1, 6],
      7: [1, 7],
      8: [1, 2, 7, 8],
      9: [4, 9],
      10: [3, 10],
      11: [9, 11],
      12: [6, 8, 11, 12]}

    if n_step in generation_polynominals.keys():
        gen_poly = np.array(generation_polynominals[n_step])
        gen_poly = gen_poly - 1

    # データ長
    n_data = 2**n_step - 1
    # 出力
    u = np.zeros(n_data)
    # レジスタの状態数
    # x = np.zeros(n_step)
    # x[0] = 1
    x = np.ones(n_step)
    # 計算
    for k in range(0, n_data):
        u[k] = x[-1]
        if len(gen_poly)/2 == 1:
            x_new = bool(x[gen_poly[0]]) ^ bool(x[gen_poly[1]])
            x = np.roll(x, 1)
            x[0] = x_new
        else:
            print("未実装")

    print("done")
    if u[0] != 0:
        u = 1 - u
    return u


if __name__ == "__main__":
    main()
