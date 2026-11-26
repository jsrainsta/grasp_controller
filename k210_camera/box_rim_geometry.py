"""Metric rectangular box-rim measurements from the current grayscale image."""

import math

import cv2
import numpy as np


def face_corners(dimensions):
    length, width = np.asarray(dimensions, dtype=float)[:2] / 2
    return np.array([[-length,-width,0],[length,-width,0],
                     [length,width,0],[-length,width,0]])


class RimEvidence:
    def __init__(self, gray, K, D, corners):
        self.gray,self.K,self.D,self.corners=gray,K,D,corners
        enhanced=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8)).apply(gray)
        self.edges=cv2.Canny(enhanced,18,58)
        gx=cv2.Sobel(enhanced,cv2.CV_32F,1,0,ksize=3)
        gy=cv2.Sobel(enhanced,cv2.CV_32F,0,1,ksize=3)
        magnitude=np.maximum(cv2.magnitude(gx,gy),1)
        self.gx,self.gy=gx/magnitude,gy/magnitude
        fraction=np.linspace(.06,.94,36)[None,:,None]
        samples=corners[:,None,:]*(1-fraction)+np.roll(corners,-1,axis=0)[:,None,:]*fraction
        self.points=np.vstack((corners,samples.reshape(-1,3)))
        self.offsets=np.arange(-3,4,dtype=np.float32)

    def measure(self, pose):
        if np.min((self.points@pose[:3,:3].T+pose[:3,3])[:,2])<=0:
            return None
        pixels=cv2.projectPoints(self.points,cv2.Rodrigues(pose[:3,:3])[0],pose[:3,3],self.K,self.D)[0]
        pixels=pixels.reshape(-1,2).astype(np.float32)
        height,width=self.gray.shape
        coverage,distances,measured=[],[],[]
        for index,line in enumerate(pixels[4:].reshape(4,-1,2)):
            inside=((line[:,0]>=6)&(line[:,0]<width-6)&(line[:,1]>=6)&(line[:,1]<height-8))
            visible=line[inside]
            if len(visible)<10 or np.linalg.norm(visible[-1]-visible[0])<22:
                coverage.append(None);distances.append(None);measured.append(None)
                continue
            vector=pixels[(index+1)%4]-pixels[index]
            normal=np.array([-vector[1],vector[0]])/np.linalg.norm(vector)
            probe=visible[:,None,:]+self.offsets[None,:,None]*normal
            x,y=probe[:,:,0],probe[:,:,1]
            on_edge=cv2.remap(self.edges,x,y,cv2.INTER_NEAREST)>0
            alignment=np.abs(cv2.remap(self.gx,x,y,cv2.INTER_NEAREST)*normal[0]
                             +cv2.remap(self.gy,x,y,cv2.INTER_NEAREST)*normal[1])
            cost=np.where(on_edge & (alignment>=.8),np.abs(self.offsets),4)
            indices=np.argmin(cost,axis=1)
            error=cost[np.arange(len(visible)),indices]
            matched=error<4
            coverage.append(float(matched.mean()))
            distances.append(float(error.mean()))
            measured.append(probe[np.arange(len(visible)),indices][matched])
        return {'corners_pixel':pixels[:4], 'coverage':coverage,
                'distances':distances,'edge_points':measured}


def overview_rim(gray, K, D, camera_board, dimensions, identity_polygons):
    """Propose boxes from measured line pairs and associate identity afterwards."""
    corners=face_corners(dimensions)
    evidence=RimEvidence(gray,K,D,corners)
    enhanced=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8)).apply(gray)
    lines=cv2.createLineSegmentDetector().detect(enhanced)[0]
    if lines is None or not identity_polygons:
        return None
    plane=camera_board.copy()
    plane[:3,3]-=dimensions[2]*plane[:3,2]
    pixels=lines.reshape(-1,2,2).astype(np.float64)
    pixels=pixels[np.linalg.norm(pixels[:,1]-pixels[:,0],axis=1)>17]
    if len(pixels)==0:
        return None
    uv=cv2.undistortPoints(pixels.reshape(-1,1,2),K,D).reshape(-1,2,2)
    rays=np.concatenate((uv,np.ones((*uv.shape[:2],1))),axis=2)
    normal=plane[:3,2]
    denominator=rays@normal
    good=np.all(np.abs(denominator)>.05,axis=1)
    rays,denominator=rays[good],denominator[good]
    depths=float(plane[:3,3]@normal)/denominator
    good=np.all(depths>0,axis=1)
    local=(rays[good]*depths[good,:,None]-plane[:3,3])@plane[:3,:2]
    lengths=np.linalg.norm(local[:,1]-local[:,0],axis=1)
    length,width,_=dimensions
    indices=np.argsort(-lengths)
    segments=local[indices[(lengths[indices]>=.65)&(lengths[indices]<=length+2)]]
    sides=joined_side_segments(segments,length)
    ends=[local[i] for i in indices if .65<=lengths[i]<=width+1.2]
    seeds=[]
    def consider(center,direction,perpendicular):
        pose=plane.copy()
        pose[:3,3]+=plane[:3,:2]@center
        pose[:3,:2]=plane[:3,:2]@np.column_stack((direction,perpendicular))
        if any(np.linalg.norm(pose[:3,3]-prior[:3,3])<.15
               and abs(pose[:3,0]@prior[:3,0])>.999 for _,prior in seeds):
            return
        rim=evidence.measure(pose)
        if rim is None or not associated(rim['corners_pixel'],identity_polygons):
            return
        score=np.mean([4 if d is None else d for d in rim['distances']])
        seeds.append((score,pose))

    for i,first in enumerate(sides):
        direction=first[1]-first[0];direction/=np.linalg.norm(direction)
        perpendicular=np.array([-direction[1],direction[0]])
        a0,a1=sorted(first@direction)
        side_coordinate=first.mean(0)@perpendicular
        for end in ends:
            vector=end[1]-end[0];span=np.linalg.norm(vector)
            if span<width*.6 or abs((vector/span)@direction)>.18:
                continue
            end_coordinate=end.mean(0)@direction
            if min(abs(end_coordinate-a0),abs(end_coordinate-a1))>1.4:
                continue
            side_sign=np.sign(end.mean(0)@perpendicular-side_coordinate)
            center_across=side_coordinate+side_sign*width/2
            if abs(end.mean(0)@perpendicular-center_across)>.7:
                continue
            along_sign=np.sign((a0+a1)/2-end_coordinate)
            consider(direction*(end_coordinate+along_sign*length/2)+perpendicular*center_across,
                     direction,perpendicular)
        for second in sides[i+1:]:
            vector=second[1]-second[0];vector/=np.linalg.norm(vector)
            if abs(vector@direction)<.985:
                continue
            across=np.array([first.mean(0)@perpendicular,second.mean(0)@perpendicular])
            if abs(abs(across[0]-across[1])-width)>.45:
                continue
            intervals=np.sort(np.array([first@direction,second@direction]),axis=1)
            if min(intervals[:,1])-max(intervals[:,0])<1.6:
                continue
            starts=[float(intervals.mean())]
            for end in ends:
                vector=end[1]-end[0];vector/=np.linalg.norm(vector)
                if abs(vector@direction)>.18 or abs(end.mean(0)@perpendicular-across.mean())>width/2:
                    continue
                coordinate=end.mean(0)@direction
                if abs(coordinate-intervals[:,0].mean())<1:
                    starts.append(coordinate+length/2)
                if abs(coordinate-intervals[:,1].mean())<1:
                    starts.append(coordinate-length/2)
            for middle in starts:
                center=direction*middle+perpendicular*across.mean()
                consider(center,direction,perpendicular)
    candidates=[]
    for _,pose in sorted(seeds,key=lambda item:item[0])[:12]:
        fitted=refine_grounded_rim(evidence,pose,allow_cropped=True)
        if fitted and associated(fitted[1]['corners_pixel'],identity_polygons):
            candidates.append(fitted)
    if not candidates:
        return None
    candidates.sort(key=lambda item:item[1]['mean_edge_distance_px'])
    pose,rim=candidates[0]
    if any(other_rim['mean_edge_distance_px']<=rim['mean_edge_distance_px']+.12
           and (np.linalg.norm(other[:3,3]-pose[:3,3])>.4 or abs(other[:3,0]@pose[:3,0])<.995)
           for other,other_rim in candidates[1:]):
        return None
    return pose,rim


def joined_side_segments(segments, length):
    """Join straight rim fragments across low-contrast spans on the same line."""
    if len(segments)==0:
        return []
    vectors=segments[:,1]-segments[:,0]
    directions=vectors/np.linalg.norm(vectors,axis=1)[:,None]
    centers=segments.mean(axis=1)
    used=np.zeros(len(segments),dtype=bool)
    result=[]
    for index,direction in enumerate(directions):
        if used[index]:
            continue
        normal=np.array([-direction[1],direction[0]])
        across=float(centers[index]@normal)
        matches=(np.abs(directions@direction)>.99)&(np.abs(centers@normal-across)<.15)
        matches&=np.abs((centers-centers[index])@direction)<length
        support=np.linalg.norm(vectors[matches],axis=1).sum()
        if support<2.2:
            continue
        used|=matches
        coordinates=segments[matches]@direction
        if coordinates.max()-coordinates.min()>=2.5:
            result.append((support,np.outer([coordinates.min(),coordinates.max()],direction)+across*normal))
    return [side for _,side in sorted(result,key=lambda item:-item[0])[:120]]


def associated(corners, identity_polygons):
    return any(all(cv2.pointPolygonTest(corners,tuple(map(float,p)),True)>=-2 for p in polygon)
               for polygon in identity_polygons)


def refine_grounded_rim(evidence, nominal, *, allow_cropped=False, prior_pose=None):
    """Use the previous pose only to bound the search for current edge evidence."""
    accepted=[]
    for along in (-.6,-.4,-.2,0.,.2,.4,.6):
        for across in (-.4,-.2,0.,.2,.4):
            for angle in (-6,-3,0,3,6):
                c,s=math.cos(math.radians(angle)),math.sin(math.radians(angle))
                pose=nominal.copy()
                pose[:3,:3]=nominal[:3,:3]@np.array([[c,-s,0],[s,c,0],[0,0,1]])
                pose[:3,3]+=nominal[:3,:2]@[along,across]
                rim=evidence.measure(pose)
                if rim is None:
                    continue
                visible=[i for i,value in enumerate(rim['coverage']) if value is not None]
                if not {0,2}.issubset(visible) or len(visible)<(3 if allow_cropped else 4):
                    continue
                long_sides=[rim['coverage'][i] for i in (0,2)]
                if (min(long_sides)<.42 or max(long_sides)<.65
                        or any(rim['coverage'][i]<.58 for i in visible if i%2)):
                    continue
                score=float(np.mean([rim['distances'][i] for i in visible]))
                accepted.append((score,along,across,angle,pose,rim))
    if not accepted:
        return None
    score,along,across,angle,pose,rim=min(accepted,key=lambda row:row[0])
    if any(other<=score+.12 and (math.hypot(a-along,b-across)>=.28 or abs(yaw-angle)>=5)
           for other,a,b,yaw,*_ in accepted):
        if prior_pose is None:
            return None
        close=[row for row in accepted if row[0]<=score+.12]
        def continuity(row):
            candidate=row[4]
            shift=float(np.linalg.norm(candidate[:3,3]-prior_pose[:3,3]))
            cosine=float(np.clip(candidate[:3,0]@prior_pose[:3,0],-1.,1.))
            angle_deg=math.degrees(math.acos(cosine))
            return (shift/.28+angle_deg/5.,row[0])
        score,along,across,angle,pose,rim=min(close,key=continuity)
        rim={**rim,'candidate_selection':'previous_confirmed_pose_tiebreak',
             'near_tied_candidate_count':len(close),
             'prior_translation_delta_cm':float(np.linalg.norm(pose[:3,3]-prior_pose[:3,3]))}
    return pose,{**rim,'mean_edge_distance_px':score}


def free_pose_from_lines(evidence, rim, nominal):
    """Recover the observed end in 3D from its edge and the two measured sides."""
    lines={}
    residuals=[]
    focal=float(min(evidence.K[0,0],evidence.K[1,1]))
    for index,points in enumerate(rim['edge_points']):
        if points is None or rim['coverage'][index]<.6:
            continue
        normalized=cv2.undistortPoints(points.astype(np.float64).reshape(-1,1,2),evidence.K,evidence.D).reshape(-1,2)
        dx,dy,x,y=cv2.fitLine(normalized,cv2.DIST_HUBER,0,.01,.01).ravel()
        line=np.array([-dy,dx,dy*x-dx*y],np.float64)
        residual=np.abs(normalized@line[:2]+line[2])*focal
        if np.quantile(residual,.9)>3.0:
            continue
        lines[index]=line
        residuals.extend(residual)
    if not {0,2}.issubset(lines) or not ({1,3}&lines.keys()):
        return []
    long_axis=np.cross(lines[0],lines[2])
    if np.linalg.norm(long_axis)<1e-5:
        return []
    long_axis/=np.linalg.norm(long_axis)
    if long_axis@nominal[:3,0]<0:
        long_axis=-long_axis
    length=np.ptp(evidence.corners[:,0])
    width=np.ptp(evidence.corners[:,1])
    candidates=[]
    for end in (1,3):
        if end not in lines:
            continue
        width_axis=np.cross(lines[end],long_axis)
        if np.linalg.norm(width_axis)<1e-5:
            continue
        width_axis/=np.linalg.norm(width_axis)
        if width_axis@nominal[:3,1]<0:
            width_axis=-width_axis
        rays=np.array([np.cross(lines[side],lines[end]) for side in (0,2)])
        if np.min(abs(rays[:,2]))<1e-5:
            continue
        rays/=rays[:,2,None]
        depths=np.linalg.lstsq(np.column_stack((-rays[0],rays[1])),width*width_axis,rcond=None)[0]
        if np.min(depths)<=0 or np.linalg.norm(depths[1]*rays[1]-depths[0]*rays[0]-width*width_axis)>.1:
            continue
        pose=np.eye(4)
        pose[:3,:3]=np.column_stack((long_axis,width_axis,np.cross(long_axis,width_axis)))
        pose[:3,3]=(rays*depths[:,None]).mean(axis=0)-(length/2 if end==1 else -length/2)*long_axis
        check=evidence.measure(pose)
        if check is None or any(check['coverage'][i] is None or check['coverage'][i]<.55 for i in lines):
            continue
        score=float(np.mean([check['distances'][i] for i in lines]))
        error=float(np.sqrt(np.mean(np.square(residuals))))
        if error<=1.5:
            candidates.append((score,error,pose))
    return candidates


def lift_rim_pose(evidence, nominal):
    """Measure a free face without using its print or assuming support height."""
    measured=[]
    for along in (-1.5,-1.2,-.9,-.6,-.3,0.,.3,.6,.9,1.2,1.5):
        for across in (-.45,0.,.45):
            for normal in (-.6,0.,.6):
                for pitch in (-18,-12,-6,0,6,12,18):
                    c,s=math.cos(math.radians(pitch)),math.sin(math.radians(pitch))
                    seed=nominal.copy()
                    seed[:3,:3]=nominal[:3,:3]@np.array([[c,0,s],[0,1,0],[-s,0,c]])
                    seed[:3,3]+=nominal[:3,:3]@[along,across,normal]
                    rim=evidence.measure(seed)
                    if rim is None:
                        continue
                    measured.extend(free_pose_from_lines(evidence,rim,nominal))
    return sorted(measured,key=lambda candidate:candidate[0])
