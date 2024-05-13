from robot_tools.trajer import TrajTools, TrajInfo
import numpy as np


if __name__ == "__main__":
    PAINTER_TEST = False
    # 二维测试
    test_data = {
        "0": {"observation": [1, 2, 3], "action": [4, 5, 6], "num": 3},
        "1": {"observation": [7, 8], "action": [10, 11], "num": 2},
        "2": {"observation": [13, 14, 15, 16], "action": [17, 18, 19, 20], "num": 4},
    }
    out_put1 = TrajTools.construct_trajs(test_data, key="observation", num_key="num")
    trajs1, info1 = out_put1
    out_put2 = TrajTools.construct_trajs(test_data, key="observation")
    trajs2, info2 = out_put2

    assert (  # 有nan的情况下无法判断是否相等
        np.nan_to_num(trajs1[0]) == np.nan_to_num(trajs1[0])
    ).all(), f"\n{trajs1[0]}\n{trajs1[0]}"
    assert info1.trajs_num == info2.trajs_num
    assert (info1.each_points_num == info2.each_points_num).all()
    assert info1.max_points_num == info2.max_points_num
    assert info1.features_num == info2.features_num

    # 三维测试
    test_data = {
        "0": {
            "observation": [[1, 2, 3]],  # 单点轨迹
            "action": [[4, 5, 6], [6, 7, 8]],
            "num": 1,
        },
        "1": {  # 唯一最大点轨迹
            "observation": [[5, 6, 0], [6, 7, 0], [7, 8, 0]],
            "action": [[7, 8, 0], [8, 9, 0], [9, 1, 0]],
            "num": 3,
        },
    }
    out_put3 = TrajTools.construct_trajs(test_data, key="observation", num_key="num")
    trajs3, info3 = out_put3
    print("三维测试")
    print(trajs3[0])
    print(trajs3[0][0, 1])

    test_traj_times = TrajTools.traj_times_from_time_trajs(trajs3[0])
    # print("test_traj_times\n")
    # print(test_time_trajs)
    # print(test_traj_times)

    # grow and mixed test
    out_put4 = TrajTools.construct_trajs(
        test_data, key="observation", num_key="num", series_type="h", mixed_type="h"
    )
    trajs4, info4 = out_put4
    print("\ngrow and mixed test")
    print(trajs4[0])

    # delete test
    X, X_info = TrajTools.delete_element_by_traj(trajs4[1], 0, info4)
    Y, Y_info = TrajTools.delete_element_by_traj(trajs4[1], -1, info4)
    print("\ndelete_element_by_traj test")
    print(trajs4[1])
    print(X)
    print(Y)
    assert X_info.max_points_num == info4.max_points_num - 1
    assert Y_info.max_points_num == info4.max_points_num - 1
    assert X_info.trajs_num == info4.trajs_num - 1
    assert Y_info.trajs_num == info4.trajs_num - 1

    A, A_info = TrajTools.delete_mixed_at_traj(trajs4[2], 0, info4)
    B, B_info = TrajTools.delete_mixed_at_traj(trajs4[2], -1, info4)
    print("\ndelete_mixed_at_traj test")
    print(trajs4[2])
    print(A)
    print(B)
    assert A_info.max_points_num == info4.max_points_num
    assert B_info.max_points_num == info4.each_points_num[0]
    Z, Z_info = TrajTools.delete_mixed_at_time(trajs4[2], 0, info4)
    W, W_info = TrajTools.delete_mixed_at_time(trajs4[2], -1, info4)
    print("\ndelete_mixed_at_time test")
    print(trajs4[2])
    print(Z)
    print(W)
    assert Z_info.trajs_num == 1
    # get traj from mixed test
    print("\nget_traj_from_mixed_trajs test")
    traj = TrajTools.get_traj_from_mixed_trajs(trajs4[2], 0, info4.trajs_num)
    print(traj)

    # get traj from series test
    print("\nget_traj_from_series_trajs test")
    traj1 = TrajTools.get_traj_from_series_trajs(trajs4[1], info4.each_points_num, 0)
    traj2 = TrajTools.get_traj_from_series_trajs(
        trajs4[1], info4.each_points_num, -1, info4.trajs_num
    )
    print(traj1)
    print(traj2)

    # delete non test
    print("\ndelete_non test")
    print(traj)
    trajs_non_del = TrajTools.delete_nan(traj, axis=0)
    print(trajs_non_del)
    trajs_non_del = TrajTools.delete_nan(traj, axis=1)
    print(trajs_non_del)

    # mixed_trajs from time_trajs test
    print("\nmixed_trajs_from_time_trajs test")
    mixed_trajs = TrajTools.mixed_trajs_from_time_trajs(
        trajs4[0], info4.trajs_num, info4.max_points_num
    )
    print(mixed_trajs)
    print(trajs4[2])
    assert (TrajTools.delete_nan(mixed_trajs) == TrajTools.delete_nan(trajs4[2])).all()

    # mixed_trajs_from_series_trajs test
    print("\nmixed_trajs_from_series_trajs test")
    print(trajs4[1])
    mixed_trajs = TrajTools.mixed_trajs_from_series_trajs(
        trajs4[1], info4.each_points_num, info4.trajs_num, info4.max_points_num
    )
    print(mixed_trajs)
    assert (TrajTools.delete_nan(mixed_trajs) == TrajTools.delete_nan(trajs4[2])).all()

    # concatenate_trajs test
    print("\nconcatenate_trajs test")
    # 特征扩充
    out_put5 = TrajTools.construct_trajs(
        test_data, key="action", num_key="num", series_type="h", mixed_type="h"
    )
    trajs5, info5 = out_put5
    trajs, info = TrajTools.concatenate_trajs((trajs4[2], info4), (trajs5[2], info5))
    print(trajs)

    # normalize_trajs test
    print("\nnormalize_trajs test")
    print(trajs4[1])
    traj_normed1 = TrajTools.normalize_trajs(trajs4[1])
    traj_normed2 = TrajTools.normalize_trajs(trajs4[1].T, "v")
    assert np.allclose(
        traj_normed1, traj_normed2.T
    ), f"\n{traj_normed1}\n{traj_normed2.T}"

    print(trajs4[2])
    traj_normed3 = TrajTools.normalize_trajs(trajs4[2], has_nan=True)
    traj_normed4 = TrajTools.normalize_trajs(trajs4[2].T, "v", has_nan=True)
    assert np.allclose(
        TrajTools.delete_nan(traj_normed3), TrajTools.delete_nan(traj_normed4.T)
    ), f"\n{traj_normed3}\n{traj_normed4.T}"

    # info construct test
    print("\ninfo construct test")
    print(trajs3)
    info = TrajInfo.consruct(trajs3, "time_trajs")
    assert info == info3
    info = TrajInfo.consruct(trajs4[2], "mixed_trajs", trajs_num=2)
    print(trajs4[2])
    assert info == info4

    # get_sub_mixed_trajs test
    print("\nget_sub_mixed_trajs test")
    print(trajs4[2])
    sub_mixed_trajs, sub_info = TrajTools.get_sub_mixed_trajs(
        trajs4[2], info4, (0, 1), (0, 1)
    )
    print(sub_mixed_trajs)
    sub_mixed_trajs, sub_info = TrajTools.get_sub_mixed_trajs(
        trajs4[2], info4, (1, 3), (0, 1)
    )
    print(sub_mixed_trajs)

    # get_sub_series_trajs test
    print("\nget_sub_series_trajs test")
    print(trajs4[1])
    sub_series_trajs, sub_info = TrajTools.get_sub_series_trajs(
        trajs4[1], info4, (0, 2), (0, 1)
    )
    print(sub_series_trajs)
    print(sub_info.each_points_num)
    print(sub_info.max_points_num)

    # get_trajs test
    print("\nget_trajs test")
    print(sub_info.get_trajs())

    if PAINTER_TEST:
        # TrajsPainter test
        from robot_tools.trajer import TrajsPainter

        print("\nTrajsPainter test")
        obs_painter = TrajsPainter(trajs4[2], info4)
        print(obs_painter._trajs)

        traj_draw = (1,)
        # get axs to draw more on the same figures
        obs_painter.features_self_labels = "experimental group"
        axs = obs_painter.plot_features_with_t(
            (0, 3), traj_draw, (0, 1, 2), return_axs=True
        )
        obs_painter.update_trajs(trajs5[2], info5)
        trajs, info = obs_painter.get_trajs_and_info()
        assert info.features_num == info4.features_num
        print(obs_painter._trajs)
        obs_painter.features_lines = "r"
        obs_painter.features_self_labels = "control group"
        # 可以指定不同的特征重复画(这里没有绘制第二个特征)
        # obs_painter.set_pause(1)
        obs_painter.plot_features_with_t((0, 3), traj_draw, (0, 2), given_axs=axs)

        # plot_2D_features test
        print(obs_painter.get_trajs_and_info()[0])
        axs = obs_painter.plot_2D_features((0, 3), (0, 1), (0, 1), return_axs=True)
        obs_painter.update_trajs(trajs4[2], info4)
        obs_painter.trajs_labels = r"$trajectories_2$"
        obs_painter.trajs_lines = "->r"
        obs_painter.trajs_markersize = 3
        obs_painter.plot_2D_features((0, 3), (0, 1), (0, 1), given_axs=axs)
