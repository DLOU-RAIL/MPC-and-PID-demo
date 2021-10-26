import math
def circles():
    x0, y0 = (0, 0)
    r = 30.0  # 半径
    angle = - 180  # x轴的夹角
    i = 1
    x11 = []
    y11 = []
    if i < 5:
        while i < 5:
            angle += 36
            x1 = x0 + r * math.cos(angle * math.pi / 180)
            y1 = math.sqrt(r ** 2 - (x1 - x0) ** 2) + y0
            x11.append(x1)
            y11.append(y1)
            i += 1
    if i >= 5:
        while i < 11:
            angle += 36
            x1 = x0 + r * math.cos(angle * math.pi / 180)
            y1 = - math.sqrt(r ** 2 - (x1 - x0) ** 2) + y0
            # y1 = - math.sqrt(r ** 2 - x1 ** 2)
            x11.append(x1)
            y11.append(y1)
            i += 1

    cx = x11
    cy = y11
    print(x11)
    print(y11)
    return cx, cy

if __name__ == '__main__':

    circles()
