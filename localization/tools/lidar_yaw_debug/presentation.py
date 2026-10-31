"""Display-only geometry. Does not change localization validity or solver input."""
import math


def rotate(p,a):
    c,s=math.cos(a),math.sin(a)
    return [c*p[0]-s*p[1],s*p[0]+c*p[1]]


def presentation(points,result):
    r=result or {}
    size=r.get('arena_size_m',[3,3]);ext=r.get('laser_to_base',[0,0,0])
    pose=None;kind='none';label='尚无位置参考'
    if r.get('full_pose_valid') and r.get('pose'):
        pose=r['pose'];kind='current';label='当前完整定位'
    elif r.get('partial_constraint_pose'):
        pose=r['partial_constraint_pose'];kind='partial';label='部分更新：未观测方向沿用先前位置'
    elif r.get('candidate_pose'):
        pose=r['candidate_pose'];kind='candidate';label='候选位置：尚在确认，未接受为定位'
    elif r.get('last_full_pose'):
        pose=r['last_full_pose'];kind='historical';label='上次完整位置：当前未定位'
    corners=[[0,0],[size[0],0],[size[0],size[1]],[0,size[1]]]
    laser_corners=[];map_points=[];laser_pose=None
    if pose:
        offset=rotate(ext[:2],pose[2]);laser_pose=[pose[0]+offset[0],pose[1]+offset[1],pose[2]+ext[2]]
        laser_corners=[rotate([c[0]-laser_pose[0],c[1]-laser_pose[1]],-laser_pose[2]) for c in corners]
        for point in points:
            if len(point)!=2 or not all(math.isfinite(v) for v in point):continue
            q=rotate(point,laser_pose[2]);map_points.append([q[0]+laser_pose[0],q[1]+laser_pose[1]])
    side=r.get('vehicle_side_m',.206)
    front=math.radians(r.get('vehicle_front_vertex_deg',0))
    vertices=[[side/math.sqrt(3)*math.cos(front+i*2*math.pi/3),side/math.sqrt(3)*math.sin(front+i*2*math.pi/3)] for i in range(3)]
    vehicle_known=r.get('pose_reference')=='vehicle_center'
    car_laser=[rotate([v[0]-ext[0],v[1]-ext[1]],-ext[2]) for v in vertices] if vehicle_known else []
    center_laser=rotate([-ext[0],-ext[1]],-ext[2]) if vehicle_known else None
    heading_laser=rotate([side*.45-ext[0],-ext[1]],-ext[2]) if vehicle_known else None
    heading_map=None
    car_map=[]
    if vehicle_known and pose:
        q=rotate([side*.45,0],pose[2]);heading_map=[pose[0]+q[0],pose[1]+q[1]]
        for v in vertices:
            q=rotate(v,pose[2]);car_map.append([q[0]+pose[0],q[1]+pose[1]])
    return {'vehicle_heading_in_laser':heading_laser,'vehicle_heading_in_map':heading_map,'vehicle_outline_note':r.get('vehicle_outline_note','approximate triangle'),'vehicle_vertices_in_laser':car_laser,'vehicle_vertices_in_map':car_map,'vehicle_center_in_laser':center_laser,'vehicle_side_m':side,'kind':kind,'label':label,'pose':pose,'laser_pose':laser_pose,'arena_size_m':size,'arena_corners_in_laser':laser_corners,'map_points':map_points,'last_full_age_s':r.get('last_full_age_s'),'coordinate_frame':'map; arbitrary square orientation modulo90'}
