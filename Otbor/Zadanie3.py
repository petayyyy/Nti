import networkx as nx
#import matplotlib as plt

def dist(x, y):
    return ((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2) ** 0.5

def line_intersection(line1, line2):
    x1, y1, x2, y2 = line1[0][0], line1[0][1], line1[1][0], line1[1][1]
    x3, y3, x4, y4 = line2[0][0], line2[0][1], line2[1][0], line2[1][1]

    if not x1 <= x2:
        x1, y1, x2, y2 = x2, y2, x1, y1
    if not x3 <= x4:
        x3, y3, x4, y4 = x4, y4, x3, y3

    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return True

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    if (x1 < x and x2 > x and x3 < x < x4) or (y1 < y and y2 > y and y3 < y and y4 > y):
        return False
    else:
        return True


n = int(input())
start = list(map(int, input().split()))
finish = list(map(int, input().split()))
G = nx.Graph()

##plt.text(start[0], start[1], 'start')
##plt.text(finish[0], finish[1], 'finish')
PointsLine = []
Point = {'start': start.copy(), 'finish': finish.copy()}


for i in range(1, n - 1):
    x1, y1, x2, y2, x3, y3 = map(int, input().split())
    A = 'A' + str(i)
    B = 'B' + str(i)
    C = 'C' + str(i)
    Point[A] = [x1, y1]
    Point[B] = [x2, y2]
    Point[C] = [x3, y3]
    G.add_edge(A, B, weight=dist([x1, y1], [x2, y2]))
    G.add_edge(B, C, weight=dist([x2, y2], [x3, y3]))
    G.add_edge(A, C, weight=dist([x1, y1], [x3, y3]))
    PointsLine.append([[x1, y1], [x2, y2]])
    PointsLine.append([[x2, y2], [x3, y3]])
    PointsLine.append([[x1, y1], [x3, y3]])
##    plt.plot([x1, x2], [y1, y2])
##    plt.plot([x2, x3], [y2, y3])
##    plt.plot([x1, x3], [y1, y3])
##    plt.text(x1, y1, A + ' (' + str(x1) + ', ' + str(y1) + ')')
##    plt.text(x2, y2, B + ' (' + str(x2) + ', ' + str(y2) + ')')
##    plt.text(x3, y3, C + ' (' + str(x3) + ', ' + str(y3) + ')')

for i in Point:
    for j in Point:
        if i == j or i[1] == j[1]:
            continue
        T = False
        S = [Point[i], Point[j]]
        if S not in PointsLine:
            for line in PointsLine:
                if not line_intersection(S, line):
                    T = True
            if not T:
                G.add_edge(i, j, weight=dist(Point[i], Point[j]))
            
for track in nx.shortest_path(G, 'start', 'finish', weight='weight'):
    print(*Point[track])
