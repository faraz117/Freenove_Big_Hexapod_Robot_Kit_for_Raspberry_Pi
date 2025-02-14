import math

def restriction(var,v_min,v_max):
        if var < v_min:
            return v_min
        elif var > v_max:
            return v_max
        else:
            return var


def coordinateToAngle(ox,oy,oz,l1=33,l2=90,l3=110):
        a=math.pi/2-math.atan2(oz,oy)
        x_3=0
        x_4=l1*math.sin(a)
        x_5=l1*math.cos(a)
        l23=math.sqrt((oz-x_5)**2+(oy-x_4)**2+(ox-x_3)**2)
        w=restriction((ox-x_3)/l23,-1,1)
        v=restriction((l2*l2+l23*l23-l3*l3)/(2*l2*l23),-1,1)
        u=restriction((l2**2+l3**2-l23**2)/(2*l3*l2),-1,1)
        b=math.asin(round(w,2))-math.acos(round(v,2))
        c=math.pi-math.acos(round(u,2))
        a=round(math.degrees(a))
        b=round(math.degrees(b))
        c=round(math.degrees(c))
        return a,b,c