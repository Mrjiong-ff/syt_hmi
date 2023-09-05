#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse
import xml.etree.ElementTree as ET


def change_svg_color(input_dir, file_name, color, output_dir):
    tree = ET.parse(os.path.join(input_dir, file_name))
    root = tree.getroot()
    for path in root:
        path.attrib['stroke'] = color
        if 'fill' in path.keys() and path.attrib['fill'] != 'none':
            path.attrib['fill'] = color
    tree.write(os.path.join(output_dir, file_name))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--color', '-c', required=True, type=str, help='#ffffff格式的颜色')
    parser.add_argument('--input_dir', '-i', required=True, type=str, help='输入目录')
    parser.add_argument('--output_dir', '-o', required=True, type=str, help='导出目录')
    args = parser.parse_args()

    input_dir = args.input_dir
    output_dir = args.output_dir
    color = args.color

    files = [item for item in os.listdir(input_dir) if item.endswith('.svg')]
    if not os.path.exists(output_dir):
        os.mkdir(output_dir)
    for file in files:
        change_svg_color(input_dir, file, color, output_dir)
