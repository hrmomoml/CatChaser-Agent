"""RPO footprint diagnostics using the official foot meshes' sole outlines."""
import numpy as np
import mujoco


def hull(points):
    points = sorted(set(map(tuple, points)))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for point in points:
        while len(lower)>=2 and cross(lower[-2], lower[-1], point)<=0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper)>=2 and cross(upper[-2], upper[-1], point)<=0:
            upper.pop()
        upper.append(point)
    return np.array(lower[:-1]+upper[:-1])


def cache_soles(demo):
    m, d = demo.m, demo.d
    soles = []
    for side in ('left', 'right'):
        geom = m.geom(side+'_foot_collision').id
        body = m.geom_bodyid[geom]
        mesh = m.geom_dataid[geom]
        start, count = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
        vertices = m.mesh_vert[start:start+count]
        world = vertices@d.geom_xmat[geom].reshape(3,3).T+d.geom_xpos[geom]
        local = (world-d.xpos[body])@d.xmat[body].reshape(3,3)
        lowest = float(local[:,2].min())
        polygon = hull(local[local[:,2]<lowest+.003,:2])
        if len(polygon)<3:
            raise RuntimeError('Cannot identify a foot sole polygon')
        soles.append((body, np.column_stack((polygon, np.full(len(polygon),lowest)))))
    return soles


def balance(demo):
    m, d = demo.m, demo.d
    if not hasattr(demo, 'foot_soles'):
        demo.foot_soles = cache_soles(demo)
    feet = []
    for body, points in demo.foot_soles:
        world = points@d.xmat[body].reshape(3,3).T+d.xpos[body]
        feet.extend(world[:,:2])
    polygon = hull(feet)
    center = d.subtree_com[0,:2]
    a, b = polygon, np.roll(polygon,-1,axis=0)
    edge = b-a
    distances = (edge[:,0]*(center[1]-a[:,1])-edge[:,1]*(center[0]-a[:,0]))/np.linalg.norm(edge,axis=1)
    base = m.body('base_link').id
    lean = np.degrees(np.arccos(np.clip(d.xmat[base].reshape(3,3)[2,2],-1,1)))
    floor = m.geom('cat_play_floor').id
    foot_contacts, other = 0, []
    for contact in d.contact:
        if floor in (contact.geom1, contact.geom2):
            geom = contact.geom2 if contact.geom1 == floor else contact.geom1
            if 'foot_' in m.geom(geom).name:
                foot_contacts += 1
            else:
                other.append(m.geom(geom).name or m.body(m.geom_bodyid[geom]).name)
    return dict(lean_deg=float(lean),height=float(d.xpos[base,2]),com_margin=float(distances.min()),
                foot_contacts=foot_contacts,other_contacts=other)
