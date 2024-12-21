from lib.setup_import_standart import *


def robot_look_at_table(articulation_cont, RMP_cont, PP_cont):
    robot_prim_path = "/World/UR10"
    table_prim_path = "/World/Table"
    table_midpoint, table_mid_X, table_mid_Y, table_mid_Z = calc_object_midpoint(
        table_prim_path
    )
    # print(f"RLT ---- Table midpoint: {table_midpoint}")
    t_pos_X = table_mid_X
    t_pos_Y = table_mid_Y
    t_pos_Z = table_mid_Z + 0.1

    target_position = np.array([t_pos_X, t_pos_Y, t_pos_Z ])
    target_orientation = np.array([1, 1, 1, 0])

    actions = RMP_cont.forward(
        target_end_effector_position=target_position,
        target_end_effector_orientation=target_orientation,
    )

    # Execute actions
    articulation_cont.apply_action(actions)
