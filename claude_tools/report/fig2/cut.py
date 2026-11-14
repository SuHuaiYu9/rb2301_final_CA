import cv2, numpy as np
img = cv2.imread('images/17.png')
P = np.array([(55,48),(82,35),(100,40),(140,22),(175,18),(210,25),(222,45),(232,50),(277,70),(282,140),(300,150),
              (330,152),(347,165),(350,225),(335,242),(300,248),(285,255),(260,262),(255,282),(290,288),(300,320),
              (300,370),(290,388),(255,388),(242,360),(230,350),(80,350),(60,360),(35,358),(25,335),(25,292),
              (40,272),(70,262),(72,240),(65,175),(58,145),(52,60)], np.int32)
poly = np.zeros(img.shape[:2], np.uint8); cv2.fillPoly(poly, [P], 255)
k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
inner, outer = cv2.erode(poly, k), cv2.dilate(poly, k)
mask = np.full(img.shape[:2], cv2.GC_BGD, np.uint8)
mask[outer > 0] = cv2.GC_PR_BGD; mask[poly > 0] = cv2.GC_PR_FGD; mask[inner > 0] = cv2.GC_FGD
bg, fg = np.zeros((1, 65)), np.zeros((1, 65))
cv2.grabCut(img, mask, None, bg, fg, 8, cv2.GC_INIT_WITH_MASK)
m = np.where((mask == 1) | (mask == 3), 255, 0).astype(np.uint8)
n, lab, st, _ = cv2.connectedComponentsWithStats(m)
m = np.where(lab == 1 + np.argmax(st[1:, cv2.CC_STAT_AREA]), 255, 0).astype(np.uint8)
m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
cv2.imwrite('scratchpad/fig2/robot_mask.png', m)
vis = img.copy(); vis[m == 0] = (vis[m == 0] * 0.25 + np.array([255, 0, 255]) * 0.75).astype(np.uint8)
cv2.imwrite('scratchpad/fig2/robot_cut_check.png', cv2.resize(vis, None, fx=1.5, fy=1.5))
