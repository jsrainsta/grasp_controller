"""Known blue/white box on a calibrated plane, with OCR identity evidence.

Not a general object detector. Fits a metric rectangle to measured image edges
and requires blue-mask support; a checkerboard edge alone cannot satisfy it.
"""
import math
import cv2
import numpy as np


def fit_outline(image, ocr, K, D, T_base_camera, T_base_board):
    fit = _fit_outline(image, ocr, K, D, T_base_camera, T_base_board)
    if fit and fit['image_geometry_supported']:
        return fit
    # Small blue fringes on the reference board can skew the initial rectangle.
    retry = _fit_outline(image, ocr, K, D, T_base_camera, T_base_board,
                         min_component_area=200)
    if retry and retry['image_geometry_supported']:
        return retry
    # Refit weak blue/white edges using color and account for visible sides.
    if fit is not None:
        edge_retry = _fit_outline(image, ocr, K, D, T_base_camera, T_base_board,
                                  use_color_and_side_geometry=True)
        if edge_retry and edge_retry['image_geometry_supported']:
            return edge_retry
        # The color mask seeds this known box. When color spills beyond its
        # top, use agreement between fits and require evidence on every edge.
        fits=[r for r in (fit,retry,edge_retry) if r and r['edge_median_px']<1.5
              and r['edge_p90_px']<3 and max(r['edge_p90_by_side_px'])<3.25]
        if len(fits)==1:
            combined=_fit_outline(image,ocr,K,D,T_base_camera,T_base_board,
                                  min_component_area=200,use_color_and_side_geometry=True)
            if (combined and combined['edge_median_px']<1.5 and combined['edge_p90_px']<3
                    and max(combined['edge_p90_by_side_px'])<3.25):
                fits.append(combined)
        if len(fits)>=2:
            best=min(fits,key=lambda r:r['edge_p90_px'])
            # The same rectangle can start at either end of its long edge.
            def corners_agree(first,second):
                corners=np.asarray(first['corners_pixel'])
                return min(np.max(np.linalg.norm(np.roll(second['corners_pixel'],shift,axis=0)-corners,axis=1))
                           for shift in (0,2))<3.2
            if all(corners_agree(best,r) for r in fits):
                return {**best,'image_geometry_supported':True,
                        'acceptance_basis':'consistent_sharp_rim_across_fit_variants'}
            gray_fits=[r for r in fits if not r['side_geometry_used']]
            if (len(gray_fits)==2 and corners_agree(*gray_fits) and edge_retry in fits
                    and max(r['gray_edge_p90_px'] for r in gray_fits)<=2
                    and edge_retry['gray_edge_p90_px']-max(r['gray_edge_p90_px'] for r in gray_fits)>=1):
                best=min(gray_fits,key=lambda r:r['gray_edge_p90_px'])
                return {**best,'image_geometry_supported':True,
                        'acceptance_basis':'consistent_grayscale_rim_with_lower_common_edge_error'}
            return {**fit,'rejection_reason':'edge_fits_disagree_on_box_corners'}
    return fit


def _fit_outline(image, ocr, K, D, T_base_camera, T_base_board,
                 min_component_area=20, *, use_color_and_side_geometry=False):
    seed_rows = [r for r in ocr.get('lines',[]) if r.get('text')=='阿昔洛韦乳膏' and r.get('confidence',0)>=.8]
    if not seed_rows:
        return None
    seed = np.array(seed_rows[0]['polygon'],np.float32)
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv,np.array([84,65,45],np.uint8),np.array([135,255,255],np.uint8))
    num,labels,stats,centers = cv2.connectedComponentsWithStats(blue)
    center = seed.mean(0)
    available = [i for i in range(1,num) if stats[i,cv2.CC_STAT_AREA]>=200
                 and np.linalg.norm(centers[i]-center)<140]
    if not available:
        return None
    # White printing separates the blue end panels from the blue body. Keep
    # nearby components together rather than fitting only the largest panel.
    nearby = [i for i in range(1,num) if stats[i,cv2.CC_STAT_AREA]>=min_component_area
              and np.linalg.norm(centers[i]-center)<140]
    blue = np.uint8(np.isin(labels,nearby))*255
    ys,xs = np.nonzero(blue)
    if len(xs)<200:
        return None
    # Blue surface pixels initialize pose; known dimensions determine white ends.
    pixel = np.column_stack((xs,ys)).astype(np.float64)
    xy = cv2.undistortPoints(pixel.reshape(-1,1,2),K,D).reshape(-1,2)
    rays = (T_base_camera[:3,:3]@np.column_stack((xy,np.ones(len(xy)))).T).T
    n = T_base_board[:3,2]
    origin = T_base_board[:3,3]-n*2.5
    distance = ((origin-T_base_camera[:3,3])@n)/(rays@n)
    world = T_base_camera[:3,3]+rays*distance[:,None]
    plane = (world-origin)@T_base_board[:3,:2]
    rect = cv2.minAreaRect(plane.astype(np.float32))
    quad = cv2.boxPoints(rect)
    e = np.roll(quad,-1,axis=0)-quad
    axis = e[np.argmax(np.linalg.norm(e,axis=1))]
    initial = np.array([*rect[0],math.atan2(axis[1],axis[0])])
    inv = np.linalg.inv(T_base_camera)
    rv = cv2.Rodrigues(inv[:3,:3])[0]
    tv = inv[:3,3]
    gray_edges = cv2.Canny(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),25,90)
    canny = gray_edges
    if use_color_and_side_geometry:
        canny = cv2.bitwise_or(canny,cv2.Canny(image[:,:,0],25,90))
    dist = cv2.distanceTransform(255-canny,cv2.DIST_L2,3)
    gray_dist = cv2.distanceTransform(255-gray_edges,cv2.DIST_L2,3) if use_color_and_side_geometry else dist
    local = np.array([[-5.25,-1.7],[5.25,-1.7],[5.25,1.7],[-5.25,1.7]])
    weight = np.linspace(.05,.95,24)[:,None]
    samples = np.concatenate([local[i]*(1-weight)+local[(i+1)%4]*weight for i in range(4)])
    all_points = np.concatenate((local,samples))
    h,w = image.shape[:2]

    def evaluate(parameters, details=False):
        u,v,theta = parameters
        c,s = math.cos(theta),math.sin(theta)
        plane_points = all_points@np.array([[c,s],[-s,c]])+np.array([u,v])
        world_points = origin+plane_points@T_base_board[:3,:2].T
        pixels = cv2.projectPoints(world_points,rv,tv,K,D)[0].reshape(-1,2)
        # Keep the full projected outline and the interpolation footprint in
        # frame. Per-edge evidence below handles intact boxes near the border.
        if np.any(pixels[:,0]<2) or np.any(pixels[:,0]>w-3) or np.any(pixels[:,1]<2) or np.any(pixels[:,1]>h-3):
            return (1e6,None) if details else 1e6
        q = pixels[:4].astype(np.float32)
        if any(cv2.pointPolygonTest(q,tuple(map(float,p)),False)<-1 for p in seed):
            return (1e5,None) if details else 1e5
        mask = np.zeros((h,w),np.uint8)
        cv2.fillConvexPoly(mask,np.rint(q).astype(np.int32),1)
        top_count = int(np.count_nonzero(mask[ys,xs]))
        raw_support = top_count / len(xs)
        support = raw_support
        if use_color_and_side_geometry:
            # The top outline is fitted to image edges. Blue pixels on the
            # known box's projected side walls should not pull it off that rim.
            bottom = cv2.projectPoints(world_points[:4]+n*2.5,rv,tv,K,D)[0].reshape(-1,2)
            silhouette = cv2.convexHull(np.rint(np.vstack((q,bottom))).astype(np.int32))
            cv2.fillConvexPoly(mask,silhouette,1)
            side_count = max(0,int(np.count_nonzero(mask[ys,xs]))-top_count)
            support = top_count / max(1,len(xs)-side_count)
        boundary = pixels[4:].astype(np.float32)
        errors = cv2.remap(dist,boundary[:,0].reshape(1,-1),boundary[:,1].reshape(1,-1),cv2.INTER_LINEAR).ravel()
        score = float(np.mean(np.minimum(errors,5))+8*(1-support))
        if use_color_and_side_geometry:
            score += float(np.quantile(errors,.9)) + 8*max(0,.9-raw_support)
        if not details:
            return score
        median,p90=float(np.median(errors)),float(np.quantile(errors,.9))
        gray_errors=cv2.remap(gray_dist,boundary[:,0].reshape(1,-1),boundary[:,1].reshape(1,-1),cv2.INTER_LINEAR)
        record = {'corners_pixel':q.tolist(),'center_pixel':pixels[:4].mean(0).tolist(),
            'edge_median_px':median, 'edge_p90_px':p90,
            'edge_p90_by_side_px':np.quantile(errors.reshape(4,-1),.9,axis=1).tolist(),
            'gray_edge_p90_px':float(np.quantile(gray_errors,.9)),
            'blue_fraction_inside':support,'fit_score':score,
            'minimum_blue_component_area_px':min_component_area,
            'raw_blue_fraction_inside_top':raw_support,
            'side_geometry_used':use_color_and_side_geometry,
            'method':'known_blue_white_box_metric_rectangle_edge_fit',
            'image_geometry_supported':bool(median<1.5 and p90<3 and support>.9 and raw_support>.8)}
        return score,record

    best = initial.copy()
    score = evaluate(best)
    # Coarse multistart within 2 cm, then bounded coordinate descent.
    starts = [initial+np.array([u,v,t]) for u in (-1,0,1) for v in (-1,0,1)
              for t in (-.12,0,.12)]
    starts = sorted(starts,key=evaluate)[:4]
    for current in starts:
        value = evaluate(current)
        for spatial,angular in ((.5,.06),(.2,.025),(.08,.01),(.03,.004)):
            for iteration in range(12):
                options = [current]+[current+np.eye(3)[j]*sign*step
                     for j,step in enumerate((spatial,spatial,angular)) for sign in (-1,1)]
                proposed = min(options,key=evaluate)
                new_value = evaluate(proposed)
                if new_value >= value-1e-6:
                    break
                current,value = proposed,new_value
            if np.linalg.norm(current[:2]-initial[:2])>2.5:
                value = 1e6
                break
        if value<score:
            best,score = current,value
    _,record = evaluate(best,True)
    return record
