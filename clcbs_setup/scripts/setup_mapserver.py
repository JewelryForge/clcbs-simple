#!/usr/bin/env python3
import argparse
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description='Modify map_server config file')
    parser.add_argument('-i', type=str, required=True)
    parser.add_argument('-o', type=str, required=True)
    args = parser.parse_args()
    yaml.warnings({'YAMLLoadWarning': False})
    with open(args.i, 'r', encoding='utf-8') as f:
        cfg = yaml.load(f.read())
    dimensions = np.array(cfg['map']['dimensions'])

    with open(args.o, 'w') as f:
        print('Write MapServer Config File to', args.o)
        f.write(
            f'image: map/map.png \n'
            f'resolution: {dimensions[0] * 1.2 / 513:.5f} \n'
            f'origin: [{-dimensions[0] * 0.6:.2f}, {-dimensions[0] * 0.6:.2f}, 0.0] \n'
            'occupied_thresh: 0.65 \n'
            'free_thresh: 0.196 \n'
            'negate: 1 \n'
        )


if __name__ == '__main__':
    main()
