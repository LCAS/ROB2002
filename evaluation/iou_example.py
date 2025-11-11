from rectangle import Rectangle

A = Rectangle(0, 0, 20, 20)
B = Rectangle(10, 10, 20, 20)

C = A & B # intersection rectangle

print(A, A.area)
print(B, B.area)
print(C, C.area)

print(A.iou(B)) # intersection over union

