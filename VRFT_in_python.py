# python-controlを用いたVRFTによるモデルフリー制御器設計
# python-control のドキュメンテーション(in English)
#   https://python-control.readthedocs.io/en/latest/

from control import matlab as crl
from matplotlib import pyplot as plt
import numpy as np


def main():
    # 制御対象を定義(今回はモータを想定)
    J = 0.6     # 慣性
    D = 0.2     # 粘性
    Kt = 0.15   # トルク定数
    P = crl.tf([Kt], [J, D, 0])     # モータ角度の電流制御

    # 参照モデルの定義
    M = crl.tf([50], [1, 50])

    # シミュレーション条件を定義
    Ts = 0.001  # サンプリング時間

    # 制御器構造の定義
    beta = crl.ss("0", "1", "0; 1", "1; 0")   # PI制御器
    print(crl.tf(beta))
    beta = crl.c2d(beta, Ts)

    # 制御対象と参照モデルのゲイン線図
    plt.figure(1)
    crl.bode([P, M])
    plt.legend(["Plant", "Ref. model"])

    # prbsを生成する
    n = 12      # m系列信号の次数
    Tp = 5      # 何周期印加するか？
    Ap = 0.5    # 振幅 [A]
    u0 = 2*prbs(n)
    u0 = Ap*(u0 - np.average(u0))
    u0 = np.tile(u0, Tp)
    t = np.linspace(0, Ts*u0.shape[0], u0.shape[0])
    plt.figure(2)
    plt.plot(t, u0)
    plt.xlabel("Time [s]")
    plt.ylabel("Current [A]")

    # 実験データの取得
    print("=== Start Experiment ===")
    Pd = crl.c2d(P, Ts, "tustin")   # 双一次変換で離散時間モデルを作成
    t = np.arange(start=0, stop=Tp*Ts*(2**n - 1), step=Ts)
    (y0, t0, xout) = crl.lsim(Pd, u0[:], t[:])   # 応答を取得
    plt.figure(3)
    plt.plot(y0)

    # VRFTを実行する
    print("=== Start VRFT ===")
    W = crl.tf([50], [1, 50])       # 周波数重み
    L = crl.minreal((1 - M)*M*W)    # プレフィルタ
    Ld = crl.c2d(L, Ts, "tustin")
    (ul, tl, xlout) = crl.lsim(Ld, u0, t)
    (phi_l, tl, xlout) = crl.lsim(beta, y0[:])
    print(phi_l)
    rho = np.linalg.solve(phi_l, ul)
    print(rho)

    plt.show()


def prbs(n_step):
    # 生成多項式(MATLABと同じように設定)
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
      12: [6, 8, 11, 12],
      13: [9, 10, 12, 13]}

    if n_step in generation_polynominals.keys():
        gen_poly = np.array(generation_polynominals[n_step])
        # gen_poly = gen_poly - 1

    # データ長
    n_data = 2**n_step - 1
    # 出力
    u = np.zeros(n_data)
    # レジスタの状態数
    x = np.ones(n_step)
    # 計算
    for k in range(0, n_data):
        u[k] = x[-1]
        buff = 0
        # 全ての生成多項式の状態のXOR計算を行う
        for gp in gen_poly:
            buff += x[gp-1]
            if buff >= 2:
                buff = 0
        x = np.insert(x, 0, buff)
        x = np.delete(x, -1)

    print("done")
    if u[0] != 0:
        u = 1 - u
    return u


if __name__ == "__main__":
    main()
