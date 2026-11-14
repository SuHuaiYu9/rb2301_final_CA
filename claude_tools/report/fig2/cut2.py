import cv2, numpy as np
B = '/tmp/claude-1000/-home-jacky-Downloads-rb2301-final-project-main/046b2aa8-8f72-439a-aad2-61e73ef2cc4b/'
# rough hand outlines (pixel coords of each photo); GrabCut refines the edge between the two bands
OUT = {
 '19': [(63,10),(78,6),(92,10),(125,9),(137,6),(150,10),(153,28),(150,55),(152,95),(150,145),(160,147),(172,155),
        (174,175),(171,192),(158,196),(148,186),(135,180),(62,180),(56,198),(42,202),(30,195),(28,165),(30,145),
        (22,132),(12,118),(12,98),(25,88),(40,85),(50,70),(52,52),(61,48),(61,30)],
 '20': [(60,10),(75,5),(90,8),(150,4),(165,4),(177,12),(177,35),(176,90),(185,96),(205,97),(210,110),(210,150),
        (200,160),(185,165),(190,175),(208,178),(214,190),(213,232),(200,240),(188,236),(180,230),(65,230),(55,238),
        (40,238),(32,228),(32,185),(40,175),(55,170),(50,160),(42,150),(42,105),(48,98),(62,92),(62,35)],
}
vis_all = []
for n, P in OUT.items():
    img = cv2.imread(B + f'images/{n}.png')[..., :3]
    poly = np.zeros(img.shape[:2], np.uint8); cv2.fillPoly(poly, [np.int32(P)], 255)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    inner, outer = cv2.erode(poly, k), cv2.dilate(poly, k)
    mask = np.full(img.shape[:2], cv2.GC_BGD, np.uint8)
    mask[outer > 0] = cv2.GC_PR_BGD; mask[poly > 0] = cv2.GC_PR_FGD; mask[inner > 0] = cv2.GC_FGD
    cv2.grabCut(img, mask, None, np.zeros((1, 65)), np.zeros((1, 65)), 8, cv2.GC_INIT_WITH_MASK)
    m = np.where((mask == 1) | (mask == 3), 255, 0).astype(np.uint8)
    nl, lab, st, _ = cv2.connectedComponentsWithStats(m)
    m = np.where(lab == 1 + np.argmax(st[1:, cv2.CC_STAT_AREA]), 255, 0).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    cv2.imwrite(B + f'scratchpad/fig2/robot_mask_{n}.png', m)
    vis = img.copy(); vis[m == 0] = (vis[m == 0] * 0.25 + np.array([255, 0, 255]) * 0.75).astype(np.uint8)
    vis_all.append(cv2.resize(vis, (int(vis.shape[1] * 300 / vis.shape[0]), 300)))
cv2.imwrite(B + 'scratchpad/fig2/cut2_check.png', np.hstack([vis_all[0], np.full((300, 10, 3), 255, np.uint8), vis_all[1]]))
