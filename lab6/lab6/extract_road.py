import cv2
import numpy 


image = cv2.imread('/home/trace_paradox/shot.png')

def mouse(event,x,y,flags,param):
	if event==cv2.EVENT_LBUTTONDOWN:
		b=image[y,x,0]
		g=image[y,x,1]
		r=image[y,x,2]
		print("B:",b)
		print("G:",g)
		print("R:",r)

cv2.namedWindow('mouse')
cv2.setMouseCallback('mouse',mouse)


cv2.imshow("original image", image)
cv2.imshow("mouse", image)
cv2.waitKey(0)
cv2.destroyAllWindows()


# light_line = numpy.array([255,255,0])
light_line = numpy.array([155,155,0])
dark_line = numpy.array([255,255,10])
# dark_line = numpy.array([155,155,155])
mask = cv2.inRange(image, light_line,dark_line)
cv2.imshow('mask', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

canny= cv2.Canny(mask,30,5)
cv2.imshow('edge', canny)
cv2.waitKey(0)
cv2.destroyAllWindows()
print(canny.shape)

r1=400;c1=100
img = canny[r1:r1+200,c1:c1+512]
cv2.imshow('crop', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

edge=[]
row =150

for i in range (512):
    if(img[row,i]==255):
        edge.append(i)
print(edge)


if(len(edge)==4):
    left_edge=edge[0]
    right_edge=edge[2]
    print(edge)
if(len(edge)==3):
    if(edge[1]-edge[0] > 5): 
        left_edge=edge[0]
        right_edge=edge[1]
    else:
        left_edge=edge[0]
        right_edge=edge[2]

road_width=(right_edge-left_edge)
frame_mid = left_edge + (road_width/2)
mid_point = 512/2
img[row,int(mid_point)]=255
print(mid_point)
error=mid_point-frame_mid 

if(error < 0):
    action="Go Right"
else :
    action="Go Left"

print("error", error)


img[row,int(frame_mid)]=255
print("mid point of the frame", frame_mid)


f_image = cv2.putText(img, action, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 1, cv2.LINE_AA)
cv2.imshow('final image',f_image)
cv2.waitKey(0)
cv2.destroyAllWindows()