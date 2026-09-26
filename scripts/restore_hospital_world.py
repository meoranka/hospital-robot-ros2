#!/usr/bin/env python3
"""Restore the Gazebo hospital world at a persistent project path.

Usage:
    python3 scripts/restore_hospital_world.py
    python3 scripts/restore_hospital_world.py --workspace ~/hospital-robot-ros2

The geometry placement is recovered from the previous hospital mapping setup:
    world: hospital_world
    mesh: models/hospital.glb
    model pose: -34.5517 -28.9429 0 1.570796 0 0
    mesh scale: 1 1 1 (SDF default)

This restores the world description only; it does not change the map, launch
ROS, install software or modify the room-navigation node.
"""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import shutil
import struct
import sys
import xml.etree.ElementTree as ET


MODEL_POSE = '-34.5517 -28.9429 0 1.570796 0 0'


def make_world(mesh):
    sdf = ET.Element('sdf', version='1.10')
    world = ET.SubElement(sdf, 'world', name='hospital_world')
    ET.SubElement(world, 'gravity').text = '0 0 -9.8'
    for filename, system in (
        ('gz-sim-physics-system', 'Physics'),
        ('gz-sim-user-commands-system', 'UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'SceneBroadcaster'),
        ('gz-sim-sensors-system', 'Sensors'),
    ):
        plugin = ET.SubElement(world, 'plugin', filename=filename,
                               name='gz::sim::systems::' + system)
        if system == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'

    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '0.5 0.5 0.5 1'
    ET.SubElement(scene, 'background').text = '0.75 0.8 0.85 1'
    sun = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(sun, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(sun, 'cast_shadows').text = 'true'
    ET.SubElement(sun, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(sun, 'specular').text = '0.2 0.2 0.2 1'
    ET.SubElement(sun, 'direction').text = '-0.5 0.1 -0.9'
    attenuation = ET.SubElement(sun, 'attenuation')
    ET.SubElement(attenuation, 'range').text = '1000'
    ET.SubElement(attenuation, 'constant').text = '0.9'
    ET.SubElement(attenuation, 'linear').text = '0.01'
    ET.SubElement(attenuation, 'quadratic').text = '0.001'

    hospital = ET.SubElement(world, 'model', name='hospital')
    ET.SubElement(hospital, 'static').text = 'true'
    ET.SubElement(hospital, 'pose').text = MODEL_POSE
    link = ET.SubElement(hospital, 'link', name='hospital_link')
    for tag in ('visual', 'collision'):
        item = ET.SubElement(link, tag, name='hospital_' + tag)
        geometry = ET.SubElement(item, 'geometry')
        mesh_element = ET.SubElement(geometry, 'mesh')
        ET.SubElement(mesh_element, 'uri').text = mesh.resolve().as_uri()
        ET.SubElement(mesh_element, 'scale').text = '1 1 1'
    ET.indent(sdf, space='  ')
    return ET.tostring(sdf, encoding='utf-8', xml_declaration=True) + b'\n'


def restore(workspace):
    workspace = workspace.expanduser().resolve()
    mesh = workspace / 'models' / 'hospital.glb'
    if not mesh.is_file():
        raise ValueError(
            f'Chyba 3D model: {mesh}\n'
            'Skript nic nezmenil. Najprv treba obnovit models/hospital.glb z projektu.')
    with mesh.open('rb') as stream:
        header = stream.read(12)
    if len(header) != 12 or header[:4] != b'glTF':
        raise ValueError('hospital.glb nie je binarny glTF model (moze ist len o Git LFS odkaz).')
    _, version, declared_size = struct.unpack('<4sII', header)
    if version != 2 or declared_size != mesh.stat().st_size:
        raise ValueError('hospital.glb ma necakanu verziu alebo je neuplny.')

    content = make_world(mesh)
    ET.fromstring(content)
    target = workspace / 'worlds' / 'hospital.sdf'
    if target.is_file() and target.read_bytes() == content:
        print(f'Svet je uz pripraveny: {target}')
        return target
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        backup = target.with_name('hospital.sdf.backup-' + stamp)
        shutil.copy2(target, backup)
        print(f'Zaloha povodneho sveta: {backup}')
    temporary = target.with_suffix('.sdf.new')
    try:
        temporary.write_bytes(content)
        temporary.replace(target)
    finally:
        temporary.unlink(missing_ok=True)
    print(f'Vytvoreny svet: {target}')
    print(f'Model nemocnice: {mesh}')
    print('Povodne posunutie a otocenie modelu bolo zachovane.')
    print('Pri dalsom spusteni pouzi world:= s touto novou cestou.')
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path,
                        default=Path.home() / 'hospital-robot-ros2')
    args = parser.parse_args()
    try:
        restore(args.workspace)
    except (OSError, ValueError, ET.ParseError) as error:
        print(f'Obnova sa nepodarila: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
