import cv2
import numpy as np
import matplotlib.pyplot as plt



if __name__ == "__main__":
    map_0 = cv2.imread('./2dmap.png')
    map_0 = cv2.cvtColor(map_0, cv2.COLOR_RGB2GRAY)
    map_0[map_0 > 0] = 255
    # plt.imshow(map_0, cmap='gray')
    # plt.show()

    kernel_3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    map_1 = cv2.dilate(map_0, kernel_3)  # 膨胀

    map_1_uint8 = map_1.astype(np.uint8)  # 转换为uint8
    contours, _ = cv2.findContours(
        map_1_uint8, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # 寻找连通域

    areas = [cv2.contourArea(cnt) for cnt in contours]
    indexes = np.argsort(areas)[::-1]
    map_2 = np.zeros([map_1.shape[0], map_1.shape[1]])
    AREA_THRESHOLD = 30
    for idx in indexes:
        print(idx, areas[idx])
        if areas[idx] > AREA_THRESHOLD:
            cv2.drawContours(map_2, contours, idx, 1, cv2.FILLED)
    
    map_3 = cv2.erode(map_2, kernel_3)  # 腐蚀

    plt.subplot(2, 2, 1)
    plt.imshow(map_0, cmap='gray')

    plt.subplot(2, 2, 2)
    plt.imshow(map_1, cmap='gray')

    plt.subplot(2, 2, 3)
    plt.imshow(map_2*255, cmap='gray')

    plt.subplot(2, 2, 4)
    plt.imshow(map_3*255, cmap='gray')
    plt.show()
    
