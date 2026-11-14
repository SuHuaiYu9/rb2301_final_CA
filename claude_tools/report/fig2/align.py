import cv2, numpy as np
a = cv2.imread('images/16.webp'); b = cv2.imread('images/18.webp')
ga, gb = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY), cv2.cvtColor(b, cv2.COLOR_BGR2GRAY)
# use only the course area; ignore player bars, the banner and people at the top corners
m = np.zeros_like(ga); m[340:1180, :] = 255
s = cv2.SIFT_create(4000)
ka, da = s.detectAndCompute(ga, m); kb, db = s.detectAndCompute(gb, m)
mt = cv2.BFMatcher().knnMatch(da, db, k=2)
good = [x for x, y in mt if x.distance < 0.7 * y.distance]
pa = np.float32([ka[g.queryIdx].pt for g in good]); pb = np.float32([kb[g.trainIdx].pt for g in good])
H, inl = cv2.findHomography(pa, pb, cv2.RANSAC, 4.0)
print(len(good), int(inl.sum())); print(np.round(H, 4))
err = np.linalg.norm(cv2.perspectiveTransform(pa[inl.ravel() == 1][None], H)[0] - pb[inl.ravel() == 1], axis=1)
print('median inlier err px', np.median(err))
np.save('scratchpad/fig2/H.npy', H)
# check: blend warped a over b
w = cv2.warpPerspective(a, H, (b.shape[1], b.shape[0]))
cv2.imwrite('scratchpad/fig2/check_align.jpg', cv2.addWeighted(w, 0.5, b, 0.5, 0)[300:1190])
