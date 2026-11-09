#!/usr/bin/env python3
import math, os, re, shutil, sys

STATIONS = {
    'near':  (-2.8, -1.4, 0.0),
    'far':   (2.8, 1.4, math.pi),
    'front': (-3.0, -2.8, math.pi / 2),
    'rear':  (3.0, 2.8, -math.pi / 2),
}

def main():
    st, cfg = sys.argv[1], os.path.expanduser(sys.argv[2])
    if st not in STATIONS:
        sys.exit('station must be one of: ' + ', '.join(STATIONS))
    live = os.path.join(cfg, 'painting_cell_rail.urdf')
    orig = live + '.orig_station'
    if not os.path.exists(orig):
        shutil.copy(live, orig)
        print('saved pristine copy ->', orig)
    s = open(orig).read()
    x, y, yaw = STATIONS[st]
    pat = re.compile(r'(<joint name="world_to_rail" type="fixed">\s*<parent link="world"/>\s*<child link="rail_base"/>\s*)<origin [^>]*/>')
    if not pat.search(s):
        sys.exit('could not find the world_to_rail joint in the URDF')
    s = pat.sub(lambda m: m.group(1) + '<origin rpy="0 0 %.6f" xyz="%.3f %.3f 0"/>' % (yaw, x, y), s, count=1)
    extra = ['  <!-- visual-only rails of the other stations -->']
    for name, (sx, sy, syaw) in STATIONS.items():
        if name == st:
            continue
        extra.append('''  <link name="rail_vis_%s">
    <visual>
      <origin xyz="2.8 0 0.08"/>
      <geometry><box size="6.0 0.25 0.16"/></geometry>
      <material name="rail_material_%s"><color rgba="0.25 0.28 0.32 1"/></material>
    </visual>
  </link>
  <joint name="world_to_rail_vis_%s" type="fixed">
    <parent link="world"/><child link="rail_vis_%s"/>
    <origin rpy="0 0 %.6f" xyz="%.3f %.3f 0"/>
  </joint>''' % (name, name, name, name, syaw, sx, sy))
    i = s.rindex('</robot>')
    s = s[:i] + '\n'.join(extra) + '\n' + s[i:]
    open(live, 'w').write(s)
    print('station set to: %s  (rail origin x=%.2f y=%.2f yaw=%.0f deg)' % (st, x, y, math.degrees(yaw)))

main()
