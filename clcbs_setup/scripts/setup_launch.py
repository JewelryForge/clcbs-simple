#!/usr/bin/env python3
import argparse
import yaml

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", type=str, required=True, help="input map file")
    parser.add_argument("-o", type=str, required=True, help="output launch file")
    args = parser.parse_args()
    with open(args.i) as map_file:
        config = yaml.load(map_file, Loader=yaml.FullLoader)
    agents, the_map = config['agents'], config['map']
    map_size = the_map['dimensions']
    print('Write Launch File to', args.o)
    with open(args.o, 'w') as fo:
        fo.write(f'<launch>\n')
        state_args = []
        for i, agent in enumerate(agents):
            state_args.append(f'agent{i}/base_link')
            start_state = agent['start']
            x, y, yaw = start_state[0] - map_size[0] / 2, start_state[1] - map_size[1] / 2, start_state[2]
            fo.write(
                f'    <include file="$(find clcbs_gazebo)/launch/robot_spawner.launch">\n'
                f'        <arg name="robot_name" value="agent{i}"/>\n'
                f'        <arg name="init_x" value="{x}"/>\n'
                f'        <arg name="init_y" value="{y}"/>\n'
                f'        <arg name="init_yaw" value="{yaw}"/>\n'
                f'    </include>\n\n'
            )
        fo.write(
            f'    <node pkg="clcbs_gazebo" type="gazebo2tf.py" name="link_tf_publisher"\n'
            f'          args="{" ".join(state_args)}" />\n'
            f'</launch>\n'
        )

        # FIXME: NOT CHANGE THE GAZEBO MAP CONFIG FILE
