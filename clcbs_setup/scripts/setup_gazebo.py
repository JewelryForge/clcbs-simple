#!/usr/bin/env python3
import argparse
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description='Generate map.png from a yaml file')
    parser.add_argument('-i', type=str, required=True)
    parser.add_argument('-o', type=str, required=True)
    args = parser.parse_args()
    yaml.warnings({'YAMLLoadWarning': False})
    with open(args.i, 'r', encoding='utf-8') as f:
        cfg = yaml.load(f.read())
    dimensions = np.array(cfg['map']['dimensions']) * 1.2
    print(f'Write Gazebo World Config File to {args.o}')
    with open(args.o, 'w') as of:
        of.write(f'<?xml version="1.0"?>\n'
                 f'<sdf version="1.5">\n'
                 f'  <model name="heightmap">\n'
                 f'    <static>true</static>\n'
                 f'    <link name="link">\n'
                 f'      <collision name="collision">\n'
                 f'        <geometry>\n'
                 f'          <heightmap>\n'
                 f'            <uri>model://heightmap/map/map.png</uri>\n'
                 f'            <size>{dimensions[0]} {dimensions[1]} 1</size>\n'
                 f'            <pos>0 0 0</pos>\n'
                 f'          </heightmap>\n'
                 f'        </geometry>\n'
                 f'      </collision>\n'
                 f'      <visual name="visual_abcedf">\n'
                 f'        <geometry>\n'
                 f'          <heightmap>\n'
                 f'            <use_terrain_paging>false</use_terrain_paging>\n'
                 f'            <texture>\n'
                 f'              <diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>\n'
                 f'              <normal>file://media/materials/textures/flat_normal.png</normal>\n'
                 f'              <size>1</size>\n'
                 f'            </texture>\n'
                 f'            <texture>\n'
                 f'              <diffuse>file://media/materials/textures/grass_diffusespecular.png</diffuse>\n'
                 f'              <normal>file://media/materials/textures/flat_normal.png</normal>\n'
                 f'              <size>1</size>\n'
                 f'            </texture>\n'
                 f'            <texture>\n'
                 f'              <diffuse>file://media/materials/textures/fungus_diffusespecular.png</diffuse>\n'
                 f'              <normal>file://media/materials/textures/flat_normal.png</normal>\n'
                 f'              <size>1</size>\n'
                 f'            </texture>\n'
                 f'            <blend>\n'
                 f'              <min_height>2</min_height>\n'
                 f'              <fade_dist>5</fade_dist>\n'
                 f'            </blend>\n'
                 f'            <uri>model://heightmap/map/map.png</uri>\n'
                 f'            <size>{dimensions[0]} {dimensions[1]} 1</size>\n'
                 f'            <pos>0 0 0</pos>\n'
                 f'          </heightmap>\n'
                 f'        </geometry>\n'
                 f'      </visual>\n'
                 f'    </link>\n'
                 f'  </model>\n'
                 f'</sdf>\n'
                 )


if __name__ == '__main__':
    main()
