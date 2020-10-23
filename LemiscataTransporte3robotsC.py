#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 24 00:45:30 2018

@author: turtlebot_master
"""

import math
import numpy as np
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped



#Se define la trayectoria y parámetros deseados del cluster

class Lemiscata:
    def __init__(self,Periodo,Radio,pd,qd,betacd):
        #Declaración de variables para las obtenidas por el optitrack
        self.theta1meas = 0 
        self.x1meas = 0
        self.y1meas = 0
        self.theta2meas = 0 
        self.x2meas = 0
        self.y2meas = 0
        self.theta3meas=0
        self.x3meas=0
        self.y3meas=0
        #PAra guardar datos
        self.theta1=[]
        self.theta2=[]
        self.theta3=[]
        self.y1 = []
        self.x1 =[]
        self.y2 = []
        self.x2 = []
        self.y3 = []
        self.x3 = []
        self.theta1dot = []
        self.theta2dot = []
        self.theta3dot = []
        self.x1des = []
        self.y1des = []
        self.x2des = []
        self.y2des = []
        self.x3des = []
        self.y3des = []
        self.x1dese = []
        self.y1dese = []
        self.x2dese = []
        self.y2dese = []
        self.x3dese = []
        self.y3dese = []
        #Para velocidades lineales deseadas
        self.x1dotdest = []
        self.y1dotdest = []
        self.x2dotdest = []
        self.y2dotdest = []
        self.x3dotdest = []
        self.y3dotdest = []
        self.x1dotdesc = []
        self.y1dotdesc = []
        self.x2dotdesc = []
        self.y2dotdesc = []
        self.x3dotdesc = []
        self.y3dotdesc = []
        self.x1dotdest2 = []
        self.y1dotdest2 =[]
        self.x2dotdest2 = []
        self.y2dotdest2 =[]
        self.x3dotdest2 = []
        self.y3dotdest2 =[]
        self.x1dotdest3 = []
        self.y1dotdest3 =[]
        self.x2dotdest3 = []
        self.y2dotdest3 =[]
        self.x3dotdest3 = []
        self.y3dotdest3 =[]
        #Tiempo
        self.time = []
        self.tiempo = []
        self.timeclusmeas = []
        self.timerobotsmeas = []
        self.timectrl = []
        #Robot 3 virtual
        self.x3virtual = []
        self.y3virtual =[]
        self.theta3virtual =[]
        #Memorias para angulos medidos
        self.theta1meas_Aux2mem=[]
        self.theta2meas_Aux2mem=[]
        self.theta3meas_Aux2mem=[]
        self.theta1measmem=[]
        self.theta2measmem=[]
        self.theta3measmem=[]
        #Condiciones iniciales 
        self.theta1meas_Aux2mem.append(0)
        self.theta2meas_Aux2mem.append(0)
        self.theta3meas_Aux2mem.append(0)
        self.theta1measmem.append(0)
        self.theta2measmem.append(0)
        self.theta3measmem.append(0)
        #Memorias para thetacd y thetacd_dot
        self.thetacdmem=[]
        self.ydmem=[]
        self.ymem=[]
        self.thetacddmem=[]
        self.ymem0=[]
        self.thetacdmem0=[]
        self.ymem1=[]
        self.thetacdmem1=[]
        self.ymem2=[]
        self.thetacdmem2=[]
        self.ymem3=[]
        self.thetacdmem3=[]
        self.ymem4=[]
        self.thetacdmem4=[]
        self.ymem5=[]
        self.thetacdmem5=[]
        
        self.thetacddmemD2=[]
        self.ydmemD2=[]
        self.thetacddmemD3=[]
        self.ydmemD3=[]
        self.thetacddmemD4=[]
        self.ydmemD4=[]
        #Valores iniciales de las memorias
        self.thetacdmem.append(math.pi)
        self.ymem.append(math.pi)
        self.thetacddmem.append(0)
        self.ydmem.append(0)
        self.thetacddmemD2.append(0)
        self.ydmemD2.append(0)
        self.thetacddmemD3.append(0)
        self.ydmemD3.append(0)
        self.thetacddmemD4.append(0)
        self.ydmemD4.append(0)
        self.thetacdmem1.append(math.pi)
        self.ymem1.append(math.pi)
        self.thetacdmem2.append(math.pi)
        self.ymem2.append(math.pi)
        self.thetacdmem3.append(math.pi)
        self.ymem3.append(math.pi)
        self.thetacdmem4.append(math.pi)
        self.ymem4.append(math.pi)
        self.thetacdmem5.append(0)
        self.ymem5.append(0)
        
        self.thetacdmem0.append(math.pi)
        self.ymem0.append(math.pi)
        #Memorias para los angulos deseados de los robots
        self.theta1dAmem=[]
        self.theta2dAmem=[]
        self.theta3dAmem=[]
        self.theta1dmem=[]
        self.theta2dmem=[]
        self.theta3dmem=[]
        
        self.theta1dAmem2=[]
        self.theta2dAmem2=[]
        self.theta3dAmem2=[]
        self.theta1dmem2=[]
        self.theta2dmem2=[]
        self.theta3dmem2=[]
        
        self.theta1dAmem3=[]
        self.theta2dAmem3=[]
        self.theta3dAmem3=[]
        self.theta1dmem3=[]
        self.theta2dmem3=[]
        self.theta3dmem3=[]
        
        
        #Valores iniciales de las memorias
        self.theta1dAmem.append(0)
        self.theta2dAmem.append(0)
        self.theta3dAmem.append(0)
        self.theta1dmem.append(0)
        self.theta2dmem.append(0)
        self.theta3dmem.append(0)
        
        self.theta1dAmem2.append(0)
        self.theta2dAmem2.append(0)
        self.theta3dAmem2.append(0)
        self.theta1dmem2.append(0)
        self.theta2dmem2.append(0)
        self.theta3dmem2.append(0)
        
        self.theta1dAmem3.append(0)
        self.theta2dAmem3.append(0)
        self.theta3dAmem3.append(0)
        self.theta1dmem3.append(0)
        self.theta2dmem3.append(0)
        self.theta3dmem3.append(0)
        #Memorias para el cluster medido
        self.thetacmem=[]
        self.thetacAmem=[]
        #Condiciones iniciales para las memorias
        self.thetacmem.append(math.pi)
        self.thetacAmem.append(math.pi)
        #Variables del cluster deseadas
        self.xcdes= []
        self.ycdes= []
        self.thetacdes = []
        self.phi1des = []
        self.phi2des = []
        self.phi3des = []
        self.pdes = []
        self.qdes = []
        self.betades = []
        #Variables del cluster medidas
        self.xc= []
        self.yc= []
        self.thetac = []
        self.phi1 = []
        self.phi2 = []
        self.phi3 = []
        self.p = []
        self.q = []
        self.beta = []
        #Errores del cluster
        self.exc = []
        self.eyc = []
        self.etheta = []
        self.ep = []
        self.eq = []
        self.ebeta = []
        self.ephi1 = []
        self.ephi2 = []
        self.ephi3 = []
        #Errores mapeados
        self.ex1_map = []
        self.ey1_map = []
        self.ex2_map = []
        self.ey2_map = []
        self.ex3_map = []
        self.ey3_map = []
        #Errores thetas
        self.etheta1 = []
        self.etheta2 = []
        self.etheta3 = []
        #Errores de posicion
        self.ex1 = []
        self.ey1 = []
        self.ex2 = []
        self.ey2 = []
        self.ex3 = []
        self.ey3 = []
        #Derivadas deseadas del cluster
        self.Der1thetacd=[]
        self.Der1xcdd = []
        self.Der1ycdd = []
        self.Der2thetacd=[]
        self.Der2xcdd = []
        self.Der2ycdd = []
        self.Der3thetacd=[]
        self.Der3xcdd = []
        self.Der3ycdd = []
        self.Der4thetacd=[]
        self.Der4xcdd = []
        self.Der4ycdd = []
        print "Trayectoria para Transporte de objeto"
        self.T = Periodo
        self.omega = (2.*math.pi)/self.T
        self.rad = Radio
        # Parametros deseados p,q, y beta constante
        self.pd = pd
        self.qd = qd
        self.betacd = betacd
    def Trayectoria(self,time):
        omega=float(self.omega)
        rad=float(self.rad)
        t=float(time)
        
        def xcdd(t):
            xcdd=-rad*omega*(math.sin(omega*t))
            return xcdd
        def ycdd(t):
            ycdd=(rad/2.)*2.*omega*(math.cos(2.*omega*t))
            return ycdd
        self.xcd= (self.rad)*math.cos((self.omega)*t) #(self.rad*math.cos(self.omega*t))/(1.+(math.sin(self.omega*t))**2)
        self.ycd= ((self.rad)/2.)*math.sin(2.*(self.omega)*t) #((self.rad*math.sin(self.omega*t))*math.cos(self.omega*t))/(1.+(math.sin(self.omega*t))**2) 
        self.thetacd = math.pi+(math.atan2(float(xcdd(t)),float(ycdd(t))))#math.pi+math.atan2(self.xcdd,self.ycdd)
        self.phi1d = math.pi
        self.phi2d = math.pi
        self.phi3d = math.pi

        #print self.thetacd
        return self.xcd,self.ycd,self.thetacd,self.phi1d,self.phi2d,self.phi3d,self.pd,self.qd,self.betacd
        
    def DerivadaTrayectoria(self,t,dt):
        delta = t-dt
        omega=self.omega
        rad=self.rad
        
        def xcdd(t):
            xcdd=-rad*omega*(math.sin(omega*t))
            return xcdd
        def ycdd(t):
            ycdd=(rad/2.)*2.*omega*(math.cos(2.*omega*t))
            return ycdd
        def theta(t):
            thetacd = math.atan2(xcdd(t),ycdd(t))
            return thetacd
        self.xcdd= xcdd(t)
        self.ycdd= ycdd(t)
        self.thetacdd = (theta(t)-theta(delta))/dt                

        self.thetacddmem.append(self.thetacdd)
        lthetacdd=len(self.thetacddmem)
        lthetacddc=lthetacdd-2
        lydmem=len(self.ydmem)
        lydmemc=lydmem-1
        
        edt=self.thetacdd-self.thetacddmem[lthetacddc]
        if abs(edt) > 1 :
            ntd=self.ydmem[lydmemc]
        else:
            ntd=self.thetacddmem[lthetacddc]+edt
            
        self.ydmem.append(ntd)

        self.Der1thetacd.append(ntd)
        self.Der1xcdd.append(self.xcdd)
        self.Der1ycdd.append(self.ycdd)
        
        return self.xcdd,self.ycdd,ntd,self.phi1dd,self.phi2dd,self.phi3dd,self.pdd,self.qdd,self.betacdd
    
    def DerivadaTrayectoria2(self,t,dt):  
        delta = t-dt
        omega=self.omega
        rad=self.rad
        
        def xcdd(t):
            xcdd=-rad*omega*(math.sin(omega*t))
            return xcdd
        def ycdd(t):
            ycdd=(rad/2.)*2.*omega*(math.cos(2.*omega*t))
            return ycdd
        def theta(t):
            thetacd = math.atan2(xcdd(t),ycdd(t))
            return thetacd
        xcdd2=xcdd(t)
        ycdd2=ycdd(t)
        thetacdd = (theta(t)-theta(delta))/dt  

        self.thetacddmemD2.append(thetacdd)
        lthetacdd=len(self.thetacddmemD2)
        lthetacddc=lthetacdd-2
        lydmem=len(self.ydmemD2)
        lydmemc=lydmem-1
        
        edt=thetacdd-self.thetacddmemD2[lthetacddc]
        if abs(edt) > 1 :
            ntd=self.ydmemD2[lydmemc]
        else:
            ntd=self.thetacddmemD2[lthetacddc]+edt
            
        self.ydmemD2.append(ntd)
        #print self.thetacdd, self.thetacddmem[lthetacddc], ntd
        
        self.pdd = 0.
        self.qdd = 0.
        self.betacdd = 0.
        self.phi1dd = 0.
        self.phi2dd = 0.
        self.phi3dd = 0.
        
        self.Der2thetacd.append(ntd)
        self.Der2xcdd.append(xcdd2)
        self.Der2ycdd.append(ycdd2)
        
        return xcdd2,ycdd2,ntd,self.phi1dd,self.phi2dd,self.phi3dd,self.pdd,self.qdd,self.betacdd
        
        
    def DerivadaTrayectoria3(self,t,dt):  
        delta = t-dt
        omega=self.omega
        rad=self.rad
        
        def xcdd(t):
            xcdd=-rad*omega*(math.sin(omega*t))
            return xcdd
        def ycdd(t):
            ycdd=(rad/2.)*2.*omega*(math.cos(2.*omega*t))
            return ycdd
        def theta(t):
            thetacd = math.atan2(xcdd(t),ycdd(t))
            return thetacd
        xcdd2=xcdd(t)
        ycdd2=ycdd(t)
        thetacdd = (theta(t)-theta(delta))/dt  

        self.thetacddmemD3.append(thetacdd)
        lthetacdd=len(self.thetacddmemD3)
        lthetacddc=lthetacdd-2
        lydmem=len(self.ydmemD3)
        lydmemc=lydmem-1
        
        edt=thetacdd-self.thetacddmemD3[lthetacddc]
        if abs(edt) > 1 :
            ntd=self.ydmemD3[lydmemc]
        else:
            ntd=self.thetacddmemD3[lthetacddc]+edt
            
        self.ydmemD3.append(ntd)
        #print self.thetacdd, self.thetacddmem[lthetacddc], ntd
        
        self.pdd = 0.
        self.qdd = 0.
        self.betacdd = 0.
        self.phi1dd = 0.
        self.phi2dd = 0.
        self.phi3dd = 0.
        

        self.Der3thetacd.append(ntd)
        self.Der3xcdd.append(xcdd2)
        self.Der3ycdd.append(ycdd2)
        
        return xcdd2,ycdd2,ntd,self.phi1dd,self.phi2dd,self.phi3dd,self.pdd,self.qdd,self.betacdd
        
        
    def DerivadaTrayectoria4(self,t,dt):  
        delta = t-dt
        omega=self.omega
        rad=self.rad
        
        def xcdd(t):
            xcdd=-rad*omega*(math.sin(omega*t))
            return xcdd
        def ycdd(t):
            ycdd=(rad/2.)*2.*omega*(math.cos(2.*omega*t))
            return ycdd
        def theta(t):
            thetacd = math.atan2(xcdd(t),ycdd(t))
            return thetacd
        xcdd2=xcdd(t)
        ycdd2=ycdd(t)
        thetacdd = (theta(t)-theta(delta))/dt  

        self.thetacddmemD4.append(thetacdd)
        lthetacdd=len(self.thetacddmemD4)
        lthetacddc=lthetacdd-2
        lydmem=len(self.ydmemD4)
        lydmemc=lydmem-1
        
        edt=thetacdd-self.thetacddmemD4[lthetacddc]
        if abs(edt) > 1 :
            ntd=self.ydmemD4[lydmemc]
        else:
            ntd=self.thetacddmemD4[lthetacddc]+edt
            
        self.ydmemD4.append(ntd)
        #print self.thetacdd, self.thetacddmem[lthetacddc], ntd
        
        self.pdd = 0.
        self.qdd = 0.
        self.betacdd = 0.
        self.phi1dd = 0.
        self.phi2dd = 0.
        self.phi3dd = 0.
        

        self.Der4thetacd.append(ntd)
        self.Der4xcdd.append(xcdd2)
        self.Der4ycdd.append(ycdd2)
        
        return xcdd2,ycdd2,ntd,self.phi1dd,self.phi2dd,self.phi3dd,self.pdd,self.qdd,self.betacdd

    #Cinematica inversa para los robots virtuales

    #def CinInvVir(self,t):
    #    Tr=self.Trayectoria(t)
    #    xc=Tr[0]
    #    yc=Tr[1]
    #    thetacdes=Tr[2]
    #    self.thetacdmem1.append(thetacdes)
        
    #    lthetacd=len(self.thetacdmem1)
    #    lthetacdc=lthetacd-2
    #    lymem1=len(self.ymem1)
    #    lymem1c=lymem1-1
        
    #    ethetacd=thetacdes-self.thetacdmem1[lthetacdc]
    #    if ethetacd >= 6.2:
    #        y=self.ymem1[lymem1c]-(2.0*math.pi-abs(thetacdes))
    #    elif ethetacd <= -6.2:
    #        y=self.ymem1[lymem1c]+(abs(thetacdes))
    #    else:
    #        y=self.ymem1[lymem1c]+ethetacd;
            
    #    self.ymem1.append(y)
        
    #    ra = (((self.qd+self.pd*math.cos(self.betacd))**2)+(self.pd*math.sin(self.betacd))**2)**(0.5)
    #    x3 = xc + (0.3333)*ra*math.sin(y)+self.qd*math.sin((self.betacd*0.5)-y)
    #    y3 = yc + (0.3333)*ra*math.cos(y)-self.qd*math.cos((self.betacd*0.5)-y)
    #    theta3 = self.phi3d - y 
    #    self.x3meas = x3
    #    self.y3meas = y3
    #    self.theta3meas = theta3
        
    #    return  x3, y3, theta3
    
    #Cinematica inversa para los errores

    def CinInv(self,exc,eyc,etheta,ephi1,ephi2,ephi3,ep,eq,ebeta):
        r = (((eq+ep*math.cos(ebeta))**2)+(ep*math.sin(ebeta))**2)**(0.5)
        x1 = exc+(0.3333)*r*math.sin(etheta)
        y1 = eyc+(0.3333)*r*math.cos(etheta)
        theta1 = ephi1 - etheta
        x2 = exc + (0.3333)*r*math.sin(etheta)-ep*math.sin((ebeta*0.5)+etheta)
        y2 = eyc + (0.3333)*r*math.cos(etheta)-ep*math.cos((ebeta*0.5)+etheta)
        theta2 = ephi2 - etheta
        x3 = exc + (0.3333)*r*math.sin(etheta)+eq*math.sin((ebeta*0.5)-etheta)
        y3 = eyc + (0.3333)*r*math.cos(etheta)-eq*math.cos((ebeta*0.5)-etheta)
        theta3 = ephi3 - etheta 
               
        return  x1, y1, theta1, x2, y2, theta2, x3, y3, theta3
        
    #Cinematica inversa para las variables deseadas de los robots
        
    def CinInv2(self,xc,yc,theta,phi1,phi2,phi3,p,q,beta):
        r = (((q+p*math.cos(beta))**2)+(p*math.sin(beta))**2)**(0.5)
        x1 = xc+(0.3333)*r*math.sin(theta)
        y1 = yc+(0.3333)*r*math.cos(theta)
        theta1 = phi1 - theta
        x2 = xc + (0.3333)*r*math.sin(theta)-p*math.sin((beta*0.5)+theta)
        y2 = yc + (0.3333)*r*math.cos(theta)-p*math.cos((beta*0.5)+theta)
        theta2 = phi2 - theta
        x3 = xc + (0.3333)*r*math.sin(theta)+q*math.sin((beta*0.5)-theta)
        y3 = yc + (0.3333)*r*math.cos(theta)-q*math.cos((beta*0.5)-theta)
        theta3 = phi3 - theta 
        
        #self.x1dese=x1
        #self.y1dese=y1
        #self.x2dese=x2
        #self.y2dese=y2
        #self.x3dese=x3
        #self.y3dese=y3
        
        return  x1, y1, theta1, x2, y2, theta2, x3, y3, theta3

    # Cinematica directa para obtener las variables en el estado del cluster 
    
    def CinDir(self,x1,x2,x3,y1,y2,y3,theta1,theta2,theta3):
        xt = (0.6666)*x1-(0.3333)*(x2+x3)
        yt = (0.6666)*y1-(0.3333)*(y2+y3)
        thetacA = math.atan2(xt,yt)
        self.thetacAmem.append(thetacA)
        lthetacAmem=len(self.thetacAmem)
        lthetacAmemc=lthetacAmem-2
        lthetacmem=len(self.thetacmem)
        lthetacmemc=lthetacmem-1
        
        ethetac=thetacA-self.thetacAmem[lthetacAmemc]
        if ethetac >= 6.2:
            thetac=self.thetacmem[lthetacmemc]-(math.pi-abs(thetacA))
        elif ethetac <= -6.2:
            thetac=self.thetacmem[lthetacmemc]+(math.pi-abs(thetacA))
        else:
            thetac=self.thetacmem[lthetacmemc]+ethetac
        self.thetacmem.append(thetac)
        
        xc = (0.3333)*(x1+x2+x3)
        yc = (0.3333)*(y1+y2+y3)
        phi1meas = theta1+thetac
        phi2meas = theta2+thetac
        phi3meas = theta3+thetac
        p = (((x1-x2)**2)+(y1-y2)**2)**(0.5)
        q = (((x3-x1)**2)+(y1-y3)**2)**(0.5)
        beta = math.acos(((p**2)+(q**2)-((x3-x2)**2)-((y3-y2)**2))*0.5/(p*q))
        
        #print thetacA, thetac
        #print x3,y3
        
        return xc, yc, thetac, phi1meas, phi2meas, phi3meas, p, q, beta
        
        # Jacobiano inverso para transformar las velocidades en el espacio del cluster
        # al espacio de los robots
        
    def JacobInv(self,t,dt,thetacdes):
        DerTr=self.DerivadaTrayectoria(t,dt)
        xdd=DerTr[0]
        ydd=DerTr[1]
        thetacdd=DerTr[2]
        #print ydd
        D = self.qd + self.pd*math.cos(self.betacd)
        E = self.pd + self.qd*math.cos(self.betacd)
        F = self.pd*math.cos((self.betacd*0.5)+thetacdes)
        G = self.pd*math.sin((self.betacd*0.5)+thetacdes)
        H = self.qd*math.cos((self.betacd*0.5)-thetacdes)
        I = self.qd*math.sin((self.betacd*0.5)-thetacdes)
        r = (((self.qd+self.pd*math.cos(self.betacd))**2)+(self.pd*math.sin(self.betacd))**2)**(0.5)
        J = ((-self.pd*self.qd*0.3333)/(r))*(math.sin(thetacdes))*(math.sin(self.betacd))
        K = ((-self.pd*self.qd*0.3333)/(r))*(math.cos(thetacdes))*(math.sin(self.betacd))
        
        Jinv1 = np.array([(1, 0, (r*math.cos(thetacdes))*0.3333, 0, 0, 0, (E*math.sin(thetacdes))*0.3333/(r), (D*math.sin(thetacdes))*0.3333/(r), J )])
        Jinv2 = np.array([(0, 1, (-r*math.sin(thetacdes))*0.3333, 0, 0, 0, (E*math.cos(thetacdes))*0.3333/(r), (D*math.cos(thetacdes))*0.3333/(r), K )])
        Jinv3 = np.array([(0, 0, -1, 1, 0, 0, 0, 0, 0)])
        Jinv4 = np.array([(1, 0, ((r*math.cos(thetacdes))*0.3333)-F, 0, 0, 0, ((E*math.sin(thetacdes))*0.3333/(r))-(G/self.pd), (D*math.sin(thetacdes))*0.3333/(r), J-(F*0.5) )])
        Jinv5 = np.array([(0, 1, ((-r*math.sin(thetacdes))*0.3333)+G, 0, 0, 0, ((E*math.cos(thetacdes))*0.3333/(r))-(F/self.pd), (D*math.cos(thetacdes))*0.3333/(r), K+(G*0.5) )])
        Jinv6 = np.array([(0, 0, -1, 0, 1, 0, 0, 0, 0)])
        Jinv7 = np.array([(1, 0, ((r*math.cos(thetacdes))*0.3333)-H, 0, 0, 0, ((E*math.sin(thetacdes))*0.3333/(r)), (D*math.sin(thetacdes))*0.3333/(r)+(I/self.qd), J+(H*0.5) )])
        Jinv8 = np.array([(0, 1, ((-r*math.sin(thetacdes))*0.3333)-I, 0, 0, 0, ((E*math.cos(thetacdes))*0.3333/(r)), ((D*math.cos(thetacdes))*0.3333/(r))-(H/self.qd), K+(I*0.5) )])
        Jinv9 = np.array([(0, 0, -1, 0, 0, 1, 0, 0, 0)])
        
        Jinv=np.array([[Jinv1], [Jinv2], [Jinv3], [Jinv4], [Jinv5], [Jinv6], [Jinv7], [Jinv8], [Jinv9]])
        
        Cvec2 = np.array([(xdd,ydd, thetacdd, self.phi1dd, self.phi2dd, self.phi3dd, self.pdd, self.qdd, self.betacdd)])
        Cvec = np.transpose(Cvec2)
        
        Rvec = np.dot(Jinv,Cvec)
        #print thetacdes
            
        return Rvec
        
    def JacobInv2(self,t,dt,thetacdes):
        DerTr=self.DerivadaTrayectoria2(t,dt)
        xdd=DerTr[0]
        ydd=DerTr[1]
        thetacdd=DerTr[2]
        D = self.qd + self.pd*math.cos(self.betacd)
        E = self.pd + self.qd*math.cos(self.betacd)
        F = self.pd*math.cos((self.betacd*0.5)+thetacdes)
        G = self.pd*math.sin((self.betacd*0.5)+thetacdes)
        H = self.qd*math.cos((self.betacd*0.5)-thetacdes)
        I = self.qd*math.sin((self.betacd*0.5)-thetacdes)
        r = (((self.qd+self.pd*math.cos(self.betacd))**2)+(self.pd*math.sin(self.betacd))**2)**(0.5)
        J = ((-self.pd*self.qd*0.3333)/(r))*(math.sin(thetacdes))*(math.sin(self.betacd))
        K = ((-self.pd*self.qd*0.3333)/(r))*(math.cos(thetacdes))*(math.sin(self.betacd))
        
        Jinv1 = np.array([(1, 0, (r*math.cos(thetacdes))*0.3333, 0, 0, 0, (E*math.sin(thetacdes))*0.3333/(r), (D*math.sin(thetacdes))*0.3333/(r), J )])
        Jinv2 = np.array([(0, 1, (-r*math.sin(thetacdes))*0.3333, 0, 0, 0, (E*math.cos(thetacdes))*0.3333/(r), (D*math.cos(thetacdes))*0.3333/(r), K )])
        Jinv3 = np.array([(0, 0, -1, 1, 0, 0, 0, 0, 0)])
        Jinv4 = np.array([(1, 0, ((r*math.cos(thetacdes))*0.3333)-F, 0, 0, 0, ((E*math.sin(thetacdes))*0.3333/(r))-(G/self.pd), (D*math.sin(thetacdes))*0.3333/(r), J-(F*0.5) )])
        Jinv5 = np.array([(0, 1, ((-r*math.sin(thetacdes))*0.3333)+G, 0, 0, 0, ((E*math.cos(thetacdes))*0.3333/(r))-(F/self.pd), (D*math.cos(thetacdes))*0.3333/(r), K+(G*0.5) )])
        Jinv6 = np.array([(0, 0, -1, 0, 1, 0, 0, 0, 0)])
        Jinv7 = np.array([(1, 0, ((r*math.cos(thetacdes))*0.3333)-H, 0, 0, 0, ((E*math.sin(thetacdes))*0.3333/(r)), (D*math.sin(thetacdes))*0.3333/(r)+(I/self.qd), J+(H*0.5) )])
        Jinv8 = np.array([(0, 1, ((-r*math.sin(thetacdes))*0.3333)-I, 0, 0, 0, ((E*math.cos(thetacdes))*0.3333/(r)), ((D*math.cos(thetacdes))*0.3333/(r))-(H/self.qd), K+(I*0.5) )])
        Jinv9 = np.array([(0, 0, -1, 0, 0, 1, 0, 0, 0)])
        
        Jinv=np.array([[Jinv1], [Jinv2], [Jinv3], [Jinv4], [Jinv5], [Jinv6], [Jinv7], [Jinv8], [Jinv9]])
        
        Cvec2 = np.array([(xdd,ydd, thetacdd, self.phi1dd, self.phi2dd, self.phi3dd, self.pdd, self.qdd, self.betacdd)])
        Cvec = np.transpose(Cvec2)
        
        Rvec = np.dot(Jinv,Cvec)
            
        return Rvec
        
    def JacobInv3(self,t,dt,thetacdes):
        DerTr=self.DerivadaTrayectoria3(t,dt)
        xdd=DerTr[0]
        ydd=DerTr[1]
        thetacdd=DerTr[2]
        D = self.qd + self.pd*math.cos(self.betacd)
        E = self.pd + self.qd*math.cos(self.betacd)
        F = self.pd*math.cos((self.betacd*0.5)+thetacdes)
        G = self.pd*math.sin((self.betacd*0.5)+thetacdes)
        H = self.qd*math.cos((self.betacd*0.5)-thetacdes)
        I = self.qd*math.sin((self.betacd*0.5)-thetacdes)
        r = (((self.qd+self.pd*math.cos(self.betacd))**2)+(self.pd*math.sin(self.betacd))**2)**(0.5)
        J = ((-self.pd*self.qd*0.3333)/(r))*(math.sin(thetacdes))*(math.sin(self.betacd))
        K = ((-self.pd*self.qd*0.3333)/(r))*(math.cos(thetacdes))*(math.sin(self.betacd))
        
        Jinv1 = np.array([(1, 0, (r*math.cos(thetacdes))*0.3333, 0, 0, 0, (E*math.sin(thetacdes))*0.3333/(r), (D*math.sin(thetacdes))*0.3333/(r), J )])
        Jinv2 = np.array([(0, 1, (-r*math.sin(thetacdes))*0.3333, 0, 0, 0, (E*math.cos(thetacdes))*0.3333/(r), (D*math.cos(thetacdes))*0.3333/(r), K )])
        Jinv3 = np.array([(0, 0, -1, 1, 0, 0, 0, 0, 0)])
        Jinv4 = np.array([(1, 0, ((r*math.cos(thetacdes))*0.3333)-F, 0, 0, 0, ((E*math.sin(thetacdes))*0.3333/(r))-(G/self.pd), (D*math.sin(thetacdes))*0.3333/(r), J-(F*0.5) )])
        Jinv5 = np.array([(0, 1, ((-r*math.sin(thetacdes))*0.3333)+G, 0, 0, 0, ((E*math.cos(thetacdes))*0.3333/(r))-(F/self.pd), (D*math.cos(thetacdes))*0.3333/(r), K+(G*0.5) )])
        Jinv6 = np.array([(0, 0, -1, 0, 1, 0, 0, 0, 0)])
        Jinv7 = np.array([(1, 0, ((r*math.cos(thetacdes))*0.3333)-H, 0, 0, 0, ((E*math.sin(thetacdes))*0.3333/(r)), (D*math.sin(thetacdes))*0.3333/(r)+(I/self.qd), J+(H*0.5) )])
        Jinv8 = np.array([(0, 1, ((-r*math.sin(thetacdes))*0.3333)-I, 0, 0, 0, ((E*math.cos(thetacdes))*0.3333/(r)), ((D*math.cos(thetacdes))*0.3333/(r))-(H/self.qd), K+(I*0.5) )])
        Jinv9 = np.array([(0, 0, -1, 0, 0, 1, 0, 0, 0)])
        
        Jinv=np.array([[Jinv1], [Jinv2], [Jinv3], [Jinv4], [Jinv5], [Jinv6], [Jinv7], [Jinv8], [Jinv9]])
        
        Cvec2 = np.array([(xdd,ydd, thetacdd, self.phi1dd, self.phi2dd, self.phi3dd, self.pdd, self.qdd, self.betacdd)])
        Cvec = np.transpose(Cvec2)
        
        Rvec = np.dot(Jinv,Cvec)
            
        return Rvec
        
    def JacobInv4(self,t,dt,thetacdes):
        DerTr=self.DerivadaTrayectoria4(t,dt)
        xdd=DerTr[0]
        ydd=DerTr[1]
        thetacdd=DerTr[2]
        D = self.qd + self.pd*math.cos(self.betacd)
        E = self.pd + self.qd*math.cos(self.betacd)
        F = self.pd*math.cos((self.betacd*0.5)+thetacdes)
        G = self.pd*math.sin((self.betacd*0.5)+thetacdes)
        H = self.qd*math.cos((self.betacd*0.5)-thetacdes)
        I = self.qd*math.sin((self.betacd*0.5)-thetacdes)
        r = (((self.qd+self.pd*math.cos(self.betacd))**2)+(self.pd*math.sin(self.betacd))**2)**(0.5)
        J = ((-self.pd*self.qd*0.3333)/(r))*(math.sin(thetacdes))*(math.sin(self.betacd))
        K = ((-self.pd*self.qd*0.3333)/(r))*(math.cos(thetacdes))*(math.sin(self.betacd))
        
        Jinv1 = np.array([(1, 0, (r*math.cos(thetacdes))*0.3333, 0, 0, 0, (E*math.sin(thetacdes))*0.3333/(r), (D*math.sin(thetacdes))*0.3333/(r), J )])
        Jinv2 = np.array([(0, 1, (-r*math.sin(thetacdes))*0.3333, 0, 0, 0, (E*math.cos(thetacdes))*0.3333/(r), (D*math.cos(thetacdes))*0.3333/(r), K )])
        Jinv3 = np.array([(0, 0, -1, 1, 0, 0, 0, 0, 0)])
        Jinv4 = np.array([(1, 0, ((r*math.cos(thetacdes))*0.3333)-F, 0, 0, 0, ((E*math.sin(thetacdes))*0.3333/(r))-(G/self.pd), (D*math.sin(thetacdes))*0.3333/(r), J-(F*0.5) )])
        Jinv5 = np.array([(0, 1, ((-r*math.sin(thetacdes))*0.3333)+G, 0, 0, 0, ((E*math.cos(thetacdes))*0.3333/(r))-(F/self.pd), (D*math.cos(thetacdes))*0.3333/(r), K+(G*0.5) )])
        Jinv6 = np.array([(0, 0, -1, 0, 1, 0, 0, 0, 0)])
        Jinv7 = np.array([(1, 0, ((r*math.cos(thetacdes))*0.3333)-H, 0, 0, 0, ((E*math.sin(thetacdes))*0.3333/(r)), (D*math.sin(thetacdes))*0.3333/(r)+(I/self.qd), J+(H*0.5) )])
        Jinv8 = np.array([(0, 1, ((-r*math.sin(thetacdes))*0.3333)-I, 0, 0, 0, ((E*math.cos(thetacdes))*0.3333/(r)), ((D*math.cos(thetacdes))*0.3333/(r))-(H/self.qd), K+(I*0.5) )])
        Jinv9 = np.array([(0, 0, -1, 0, 0, 1, 0, 0, 0)])
        
        Jinv=np.array([[Jinv1], [Jinv2], [Jinv3], [Jinv4], [Jinv5], [Jinv6], [Jinv7], [Jinv8], [Jinv9]])
        
        Cvec2 = np.array([(xdd,ydd, thetacdd, self.phi1dd, self.phi2dd, self.phi3dd, self.pdd, self.qdd, self.betacdd)])
        Cvec = np.transpose(Cvec2)
        
        Rvec = np.dot(Jinv,Cvec)
            
        return Rvec

    # Funcion para obtener los valores de thetas deseados consideran las
    # restricciones no holonomas
    
    
    
    def thetaidcmd(self,t,dt):
        
        thetacdes1=self.Trayectoria(t)[2]
        self.thetacdmem2.append(thetacdes1)

        lthetacd=len(self.thetacdmem2)
        lthetacdc=lthetacd-2
        lymem2=len(self.ymem2)
        lymem2c=lymem2-1
        
        ethetacd=thetacdes1-self.thetacdmem2[lthetacdc]
        if ethetacd >= 5.5:
            thetacdes=self.ymem2[lymem2c]-(2.0*math.pi-abs(thetacdes1))
        elif ethetacd <= -5.5:
            thetacdes=self.ymem2[lymem2c]+(abs(thetacdes1))
        else:
            thetacdes=self.ymem2[lymem2c]+ethetacd;
            
        self.ymem2.append(thetacdes)
        #print t,dt,thetacdes
        
        JI=self.JacobInv(t,dt,thetacdes)
        
        x1dotcmd = JI[0]
        y1dotcmd = JI[1]
        
        self.x1dotdest.append(((x1dotcmd[0])[0])[0])
        self.y1dotdest.append(((y1dotcmd[0])[0])[0])
        
        #print t, x1dotcmd
        
        theta1dA = -math.atan2(x1dotcmd,y1dotcmd)
        self.theta1dAmem.append(theta1dA)
        ltheta1dAmem=len(self.theta1dAmem)
        ltheta1dAmemc=ltheta1dAmem-2
        ltheta1dmem=len(self.theta1dmem)
        ltheta1dmemc=ltheta1dmem-1
        
        etheta1d=theta1dA-self.theta1dAmem[ltheta1dAmemc]
        if etheta1d >= 5.5:
            theta1d=self.theta1dmem[ltheta1dmemc]-(math.pi-abs(theta1dA))
        elif etheta1d <= -5.5:
            theta1d=self.theta1dmem[ltheta1dmemc]+(math.pi-abs(theta1dA))
        else: 
            theta1d=self.theta1dmem[ltheta1dmemc]+etheta1d
        self.theta1dmem.append(theta1d)
        #print theta1d
        
        x2dotcmd = JI[3]
        y2dotcmd = JI[4]
        
        self.x2dotdest.append(((x2dotcmd[0])[0])[0])
        self.y2dotdest.append(((y2dotcmd[0])[0])[0])
        
        #print x2dotcmd, y2dotcmd
        theta2dA = -math.atan2(x2dotcmd,y2dotcmd)
        self.theta2dAmem.append(theta2dA)
        ltheta2dAmem=len(self.theta2dAmem)
        ltheta2dAmemc=ltheta2dAmem-2
        ltheta2dmem=len(self.theta2dmem)
        ltheta2dmemc=ltheta2dmem-1
        
        etheta2d=theta2dA-self.theta2dAmem[ltheta2dAmemc]
        if etheta2d >= 5.5:
            theta2d=self.theta2dmem[ltheta2dmemc]-(math.pi-abs(theta2dA))
        elif etheta2d <= -5.5:
            theta2d=self.theta2dmem[ltheta2dmemc]+(math.pi-abs(theta2dA))
        else: 
            theta2d=self.theta2dmem[ltheta2dmemc]+etheta2d
        self.theta2dmem.append(theta2d)
        #print theta2d, theta2dA
        

        x3dotcmd = JI[6]
        y3dotcmd = JI[7]
        
        self.x3dotdest.append(((x3dotcmd[0])[0])[0])
        self.y3dotdest.append(((y3dotcmd[0])[0])[0])        
        
        theta3dA = -math.atan2(x3dotcmd,y3dotcmd)
        self.theta3dAmem.append(theta3dA)
        ltheta3dAmem=len(self.theta3dAmem)
        ltheta3dAmemc=ltheta3dAmem-2
        ltheta3dmem=len(self.theta3dmem)
        ltheta3dmemc=ltheta3dmem-1
        
        etheta3d=theta3dA-self.theta3dAmem[ltheta3dAmemc]
        if etheta3d >= 5.5:
            theta3d=self.theta3dmem[ltheta3dmemc]-(math.pi-abs(theta3dA))
        elif etheta3d <= -5.5:
            theta3d=self.theta3dmem[ltheta3dmemc]+(math.pi-abs(theta3dA))
        else: 
            theta3d=self.theta3dmem[ltheta3dmemc]+etheta3d
        self.theta3dmem.append(theta3d)
        
        
        #Para la trayectoria deseada de los robots

        TrayDes=self.Trayectoria(t)        
        xc = TrayDes[0]
        yc = TrayDes[1]
        phi1 = TrayDes[3]
        phi2 = TrayDes[4]
        phi3 = TrayDes[5]
        p = TrayDes[6]
        q = TrayDes[7]
        beta = TrayDes[8]
        
        TrDes = self.CinInv2(xc, yc, thetacdes1, phi1, phi2, phi3, p, q, beta)
        x1des = TrDes[0]
        y1des = TrDes[1]
        x2des = TrDes[3]
        y2des = TrDes[4]
        x3des = TrDes[6]
        y3des = TrDes[7]
        
        self.x1des.append(x1des)
        self.y1des.append(y1des)
        self.x2des.append(x2des)
        self.y2des.append(y2des)
        self.x3des.append(x3des)
        self.y3des.append(y3des)
        
        self.ex1.append(self.x1meas-x1des)
        self.ey1.append(self.y1meas-y1des)
        self.ex2.append(self.x2meas-x2des)
        self.ey2.append(self.y2meas-y2des)
        self.ex3.append(self.x3meas-x3des)
        self.ey3.append(self.y3meas-y3des)
        
        #print t, theta1d
        return theta1d,theta2d,theta3d
        
    def thetaidcmd2(self,t,dt):
        
        thetacdes1=self.Trayectoria(t)[2]
        self.thetacdmem4.append(thetacdes1)

        lthetacd=len(self.thetacdmem4)
        lthetacdc=lthetacd-2
        lymem2=len(self.ymem4)
        lymem2c=lymem2-1
        
        ethetacd=thetacdes1-self.thetacdmem4[lthetacdc]
        if ethetacd >= 5.5:
            thetacdes=self.ymem4[lymem2c]-(2.0*math.pi-abs(thetacdes1))
        elif ethetacd <= -5.5:
            thetacdes=self.ymem4[lymem2c]+(abs(thetacdes1))
        else:
            thetacdes=self.ymem4[lymem2c]+ethetacd;
            
        self.ymem4.append(thetacdes)
        #print t,dt,thetacdes
        
        JI=self.JacobInv3(t,dt,thetacdes)
        
        x1dotcmd = JI[0]
        y1dotcmd = JI[1]
        
        self.x1dotdest2.append(((x1dotcmd[0])[0])[0])
        self.y1dotdest2.append(((y1dotcmd[0])[0])[0])
        
        theta1dA = -math.atan2(x1dotcmd,y1dotcmd)
        self.theta1dAmem2.append(theta1dA)
        ltheta1dAmem=len(self.theta1dAmem2)
        ltheta1dAmemc=ltheta1dAmem-2
        ltheta1dmem=len(self.theta1dmem2)
        ltheta1dmemc=ltheta1dmem-1
        
        etheta1d=theta1dA-self.theta1dAmem2[ltheta1dAmemc]
        if etheta1d >= 5.5:
            theta1d=self.theta1dmem2[ltheta1dmemc]-(math.pi-abs(theta1dA))
        elif etheta1d <= -5.5:
            theta1d=self.theta1dmem2[ltheta1dmemc]+(math.pi-abs(theta1dA))
        else: 
            theta1d=self.theta1dmem2[ltheta1dmemc]+etheta1d
        self.theta1dmem2.append(theta1d)
        #print theta1d
        
        x2dotcmd = JI[3]
        y2dotcmd = JI[4]
        
        self.x2dotdest2.append(((x2dotcmd[0])[0])[0])
        self.y2dotdest2.append(((y2dotcmd[0])[0])[0])
        
        #print x2dotcmd, y2dotcmd
        theta2dA = -math.atan2(x2dotcmd,y2dotcmd)
        self.theta2dAmem2.append(theta2dA)
        ltheta2dAmem=len(self.theta2dAmem2)
        ltheta2dAmemc=ltheta2dAmem-2
        ltheta2dmem=len(self.theta2dmem2)
        ltheta2dmemc=ltheta2dmem-1
        
        etheta2d=theta2dA-self.theta2dAmem2[ltheta2dAmemc]
        if etheta2d >= 5.5:
            theta2d=self.theta2dmem2[ltheta2dmemc]-(math.pi-abs(theta2dA))
        elif etheta2d <= -5.5:
            theta2d=self.theta2dmem2[ltheta2dmemc]+(math.pi-abs(theta2dA))
        else: 
            theta2d=self.theta2dmem2[ltheta2dmemc]+etheta2d
        self.theta2dmem2.append(theta2d)
        #print theta2d, theta2dA
        

        x3dotcmd = JI[6]
        y3dotcmd = JI[7]
        
        self.x3dotdest2.append(((x3dotcmd[0])[0])[0])
        self.y3dotdest2.append(((y3dotcmd[0])[0])[0])        
        
        theta3dA = -math.atan2(x3dotcmd,y3dotcmd)
        self.theta3dAmem2.append(theta3dA)
        ltheta3dAmem=len(self.theta3dAmem2)
        ltheta3dAmemc=ltheta3dAmem-2
        ltheta3dmem=len(self.theta3dmem2)
        ltheta3dmemc=ltheta3dmem-1
        
        etheta3d=theta3dA-self.theta3dAmem2[ltheta3dAmemc]
        if etheta3d >= 5.5:
            theta3d=self.theta3dmem2[ltheta3dmemc]-(math.pi-abs(theta3dA))
        elif etheta3d <= -5.5:
            theta3d=self.theta3dmem2[ltheta3dmemc]+(math.pi-abs(theta3dA))
        else: 
            theta3d=self.theta3dmem2[ltheta3dmemc]+etheta3d
        self.theta3dmem2.append(theta3d)
        
        return theta1d,theta2d,theta3d
        
    def thetaidcmd3(self,t,dt):
        
        thetacdes1=self.Trayectoria(t)[2]
        self.thetacdmem5.append(thetacdes1)

        lthetacd=len(self.thetacdmem5)
        lthetacdc=lthetacd-2
        lymem2=len(self.ymem5)
        lymem2c=lymem2-1
        
        ethetacd=thetacdes1-self.thetacdmem5[lthetacdc]
        if ethetacd >= 5.5:
            thetacdes=self.ymem5[lymem2c]-(2.0*math.pi-abs(thetacdes1))
        elif ethetacd <= -5.5:
            thetacdes=self.ymem5[lymem2c]+(abs(thetacdes1))
        else:
            thetacdes=self.ymem5[lymem2c]+ethetacd;
            
        self.ymem5.append(thetacdes)
        #print t,dt,thetacdes
        
        JI=self.JacobInv4(t,dt,thetacdes)
        
        x1dotcmd = JI[0]
        y1dotcmd = JI[1]
        
        self.x1dotdest3.append(((x1dotcmd[0])[0])[0])
        self.y1dotdest3.append(((y1dotcmd[0])[0])[0])
        
        theta1dA = -math.atan2(x1dotcmd,y1dotcmd)
        self.theta1dAmem3.append(theta1dA)
        ltheta1dAmem=len(self.theta1dAmem3)
        ltheta1dAmemc=ltheta1dAmem-2
        ltheta1dmem=len(self.theta1dmem3)
        ltheta1dmemc=ltheta1dmem-1
        
        etheta1d=theta1dA-self.theta1dAmem3[ltheta1dAmemc]
        if etheta1d >= 5.5:
            theta1d=self.theta1dmem3[ltheta1dmemc]-(math.pi-abs(theta1dA))
        elif etheta1d <= -5.5:
            theta1d=self.theta1dmem3[ltheta1dmemc]+(math.pi-abs(theta1dA))
        else: 
            theta1d=self.theta1dmem3[ltheta1dmemc]+etheta1d
        self.theta1dmem3.append(theta1d)
        #print theta1d
        
        x2dotcmd = JI[3]
        y2dotcmd = JI[4]
        
        self.x2dotdest3.append(((x2dotcmd[0])[0])[0])
        self.y2dotdest3.append(((y2dotcmd[0])[0])[0])
        
        #print x2dotcmd, y2dotcmd
        theta2dA = -math.atan2(x2dotcmd,y2dotcmd)
        self.theta2dAmem3.append(theta2dA)
        ltheta2dAmem=len(self.theta2dAmem3)
        ltheta2dAmemc=ltheta2dAmem-2
        ltheta2dmem=len(self.theta2dmem3)
        ltheta2dmemc=ltheta2dmem-1
        
        etheta2d=theta2dA-self.theta2dAmem3[ltheta2dAmemc]
        if etheta2d >= 5.5:
            theta2d=self.theta2dmem3[ltheta2dmemc]-(math.pi-abs(theta2dA))
        elif etheta2d <= -5.5:
            theta2d=self.theta2dmem3[ltheta2dmemc]+(math.pi-abs(theta2dA))
        else: 
            theta2d=self.theta2dmem3[ltheta2dmemc]+etheta2d
        self.theta2dmem3.append(theta2d)
        #print theta2d, theta2dA
        

        x3dotcmd = JI[6]
        y3dotcmd = JI[7]
        
        self.x3dotdest3.append(((x3dotcmd[0])[0])[0])
        self.y3dotdest3.append(((y3dotcmd[0])[0])[0])        
        
        theta3dA = -math.atan2(x3dotcmd,y3dotcmd)
        self.theta3dAmem3.append(theta3dA)
        ltheta3dAmem=len(self.theta3dAmem3)
        ltheta3dAmemc=ltheta3dAmem-2
        ltheta3dmem=len(self.theta3dmem3)
        ltheta3dmemc=ltheta3dmem-1
        
        etheta3d=theta3dA-self.theta3dAmem3[ltheta3dAmemc]
        if etheta3d >= 5.5:
            theta3d=self.theta3dmem3[ltheta3dmemc]-(math.pi-abs(theta3dA))
        elif etheta3d <= -5.5:
            theta3d=self.theta3dmem3[ltheta3dmemc]+(math.pi-abs(theta3dA))
        else: 
            theta3d=self.theta3dmem3[ltheta3dmemc]+etheta3d
        self.theta3dmem3.append(theta3d)
        
        return theta1d,theta2d,theta3d
            
            # Funcion que calcula las derivadas de los angulos de comando
            # en el espacio de los robots
            
    def thetaidcmdd(self,t,dt):
        delta = t-dt
        thetacmd=self.thetaidcmd(t,dt)
        thetacmd2=self.thetaidcmd2(delta,dt)
        theta1dcmd = (thetacmd[0]-thetacmd2[0])/dt
        theta2dcmd = (thetacmd[1]-thetacmd2[1])/dt
        theta3dcmd = (thetacmd[2]-thetacmd2[2])/dt
        #print theta1dcmd,theta2dcmd,theta3dcmd
        #print t
        return theta1dcmd, theta2dcmd, theta3dcmd

    #Funcion que manda las variables de los robots (1 virtual, 2 reales) para
    #transformarlas en variables del cluster
    
    def cvir(self,t):
        x2Real = self.x2meas
        y2Real = self.y2meas
        theta2Real = self.theta2meas
        x3Vir = self.x3meas #CIV[0]
        y3Vir = self.y3meas #CIV[1]
        theta3Vir = self.theta3meas #CIV[2]
        x1Real = self.x1meas #Aqui van los valores obtenidos
        y1Real = self.y1meas # del optitrack
        theta1Real = self.theta1meas #
        self.timeclusmeas.append(t)

        
        ClusterVirt = self.CinDir(x1Real, x2Real, x3Vir, y1Real, y2Real, y3Vir, theta1Real, theta2Real, theta3Vir)
        
        
        #print "Variables Globales cvir: [%f, %f, %f]"%(x1Real,y1Real,theta1Real)    
        return ClusterVirt
        
        # Funcion que calcula los errores con dos robots virtuales
        
    def errcluster(self,t):
        ClsVir = self.cvir(t)
        xc = ClsVir[0]
        yc = ClsVir[1]
        thetac = ClsVir[2]
        p = ClsVir[6]
        q = ClsVir[7]
        beta = ClsVir[8]
        phi1 = ClsVir[3]
        phi2 = ClsVir[4]
        phi3 = ClsVir[5]
        
        #print t
        
        exc = self.Trayectoria(t)[0]-xc
        eyc = self.Trayectoria(t)[1]-yc
        etheta = thetac #No se usa esta diferencia, en su lugar se toma thetac medida
        ep = self.pd-p
        eq = self.qd-q
        ebeta = self.betacd-beta
        ephi1 = self.phi1d-phi1
        ephi2 = self.phi2d-phi2
        ephi3 = self.phi3d-phi3
        
        self.xc.append(xc)
        self.yc.append(yc)
        self.thetac.append(thetac)
        #print thetac
        self.phi1.append(phi1)
        self.phi2.append(phi2)
        self.phi3.append(phi3)
        self.p.append(p)
        self.q.append(q)
        self.beta.append(beta)
        
        #print "Error sin signo %f"%(exc)
        return exc,eyc,etheta,ephi1,ephi2,ephi3,ep,eq,ebeta

    def errrobots(self,t):
        #Los errores en el espacio del cluster son multiplicados por por una
        #constante, en este caso de -1
        e = self.errcluster(t)
        exc = -1.0*e[0]
        eyc = -1.0*e[1]
        etheta = e[2] 
        ep = -1.6*e[6]
        eq = -1.6*e[7]
        ebeta = -e[8]
        ephi1 = -e[3]
        ephi2 = -e[4]
        ephi3 = -e[5]


        
        #Memorias para graficar
        self.exc.append(exc/1.0)
        self.eyc.append(eyc/1.0)
        
        self.ep.append((ep/1.0))
        self.eq.append((eq/1.0))
        self.ebeta.append(ebeta/1.0)
        self.ephi1.append(ephi1)
        self.ephi2.append(ephi2)
        self.ephi3.append(ephi3)
        
        Try=self.Trayectoria(t)
        xc = Try[0]
        yc = Try[1]
        thetac1 = Try[2]
        self.thetacdmem3.append(thetac1)

        lthetacd=len(self.thetacdmem3)
        lthetacdc=lthetacd-2
        lymem3=len(self.ymem3)
        lymem3c=lymem3-1
        
        ethetacd=thetac1-self.thetacdmem3[lthetacdc]
        if ethetacd >= 6.2:
            thetac=self.ymem3[lymem3c]-(2.0*math.pi-abs(thetac1))
        elif ethetacd <= -6.2:
            thetac=self.ymem3[lymem3c]+(abs(thetac1))
        else:
            thetac=self.ymem3[lymem3c]+ethetacd;
            
        self.ymem3.append(thetac)
        phi1 = Try[3]
        phi2 = Try[4]
        phi3 = Try[5]
        p = Try[6]
        q = Try[7]
        beta = Try[8]
        
        self.xcdes.append(xc)
        self.ycdes.append(yc)
        self.thetacdes.append(thetac)
        self.etheta.append((etheta)-thetac)
        self.phi1des.append(phi1)
        self.phi2des.append(phi2)
        self.phi3des.append(phi3)
        self.pdes.append(p)
        self.qdes.append(q)
        self.betades.append(beta)        

        
        Rerror = self.CinInv(exc,eyc,etheta,ephi1,ephi2,ephi3,ep,eq,ebeta)
        #Rerror = self.CinInv2(xc,yc,thetac,ephi1,ephi2,ephi3,p,q,beta)
        
        return Rerror[0],Rerror[1],Rerror[2],Rerror[3],Rerror[4],Rerror[5],Rerror[6],Rerror[7],Rerror[8]

    def VelCtrl(self,t,dt):
        
        #Llama a los errores mapeados en x y y
        Rerror = self.errrobots(t)
        ex1_ctrl = Rerror[0]
        ey1_ctrl = Rerror[1]
        ex2_ctrl = Rerror[3]
        ey2_ctrl = Rerror[4]
        ex3_ctrl = Rerror[6]
        ey3_ctrl = Rerror[7]
        
        
        #ex1_ctrl = self.x1meas-Rerror[0] 
        #ey1_ctrl = self.y1meas-Rerror[1] 
        #ex2_ctrl = self.x2meas-Rerror[3] 
        #ey2_ctrl = self.y2meas-Rerror[4] 
        #ex3_ctrl = self.x3meas-Rerror[6]
        #ey3_ctrl = self.y3meas-Rerror[7] 
        
        self.ex1_map.append(ex1_ctrl)
        self.ey1_map.append(ey1_ctrl)
        self.ex2_map.append(ex2_ctrl)
        self.ey2_map.append(ey2_ctrl)
        self.ex3_map.append(ex3_ctrl)
        self.ey3_map.append(ey3_ctrl)
        
        #Calcula los errores de thetas
        thetaidcmd=self.thetaidcmd3(t,dt)
        etheta1_ctrl = self.theta1meas - thetaidcmd[0]  #theta1, theta2  y theta3 
        etheta2_ctrl = self.theta2meas - thetaidcmd[1]  #son los ángulos
        etheta3_ctrl = self.theta3meas - thetaidcmd[2]  #medidos del optitrack
        
        self.etheta1.append(etheta1_ctrl)
        self.etheta2.append(etheta2_ctrl)
        self.etheta3.append(etheta3_ctrl)
        
        #print "theta1meas VelCtrl: [%f]"%(theta1meas)
        
        #Llama a las velocidades en x y y deseadas mapeadas por el Jacobiano
        thetacdes1=self.Trayectoria(t)[2]
        self.thetacdmem0.append(thetacdes1)

        lthetacd=len(self.thetacdmem0)
        lthetacdc=lthetacd-2
        lymem0=len(self.ymem0)
        lymem0c=lymem0-1
        
        ethetacd=thetacdes1-self.thetacdmem0[lthetacdc]
        if ethetacd >= 6.2:
            thetacdes=self.ymem0[lymem0c]-(2.0*math.pi-abs(thetacdes1))
        elif ethetacd <= -6.2:
            thetacdes=self.ymem0[lymem0c]+(abs(thetacdes1))
        else:
            thetacdes=self.ymem0[lymem0c]+ethetacd;
            
        self.ymem0.append(thetacdes)
        #print thetacdes
        
        JacobInv=self.JacobInv2(t,dt,thetacdes)
        x1dotcmd = JacobInv[0]
        y1dotcmd = JacobInv[1]
        x2dotcmd = JacobInv[3]
        y2dotcmd = JacobInv[4]
        x3dotcmd = JacobInv[6]
        y3dotcmd = JacobInv[7]
        
        x1c = x1dotcmd[0] 
        y1c = y1dotcmd[0] 
        x2c = x2dotcmd[0]
        y2c = y2dotcmd[0] 
        x3c = x3dotcmd[0] 
        y3c = y3dotcmd[0] 
        
        print t,y3c,x3c
        
        self.x1dotdesc.append((x1c[0])[0])
        self.y1dotdesc.append((y1c[0])[0])
        self.x2dotdesc.append((x2c[0])[0])
        self.y2dotdesc.append((y2c[0])[0])
        self.x3dotdesc.append((x3c[0])[0])
        self.y3dotdesc.append((y3c[0])[0])
        
        #print (x1c[0])[0], (y1c[0])[0]
        
        #Llama a las velocidades rotacionales con restricciones holonómicas
        thetaidcmdd=self.thetaidcmdd(t,dt)
        theta1dotcmd = thetaidcmdd[0]
        theta2dotcmd = thetaidcmdd[1]
        theta3dotcmd = thetaidcmdd[2]
        
        self.theta1dot.append(theta1dotcmd)
        self.theta2dot.append(theta2dotcmd)
        self.theta3dot.append(theta3dotcmd)
        
        #print theta1dotcmd,theta2dotcmd,theta3dotcmd
        
        #Ganancias de controladores
        k1_1 = 0.5
        k1_2 = 0.5
        k1_3 = 0.7
        k2_1 = 0.5
        k2_2 = 0.5
        k2_3 = 0.7
        k3_1 = 0.5
        k3_2 = 0.5
        k3_3 = 0.7
        
        #Señales virtuales de controles
        V1_x = x1dotcmd - k1_1*ex1_ctrl
        V1_y = y1dotcmd - k1_2*ey1_ctrl
        V1_theta = theta1dotcmd - k1_3*etheta1_ctrl
        
        #print V1_x,x1dotcmd,ex1_ctrl
        #print V1_y,y1dotcmd,ey1_ctrl
        
        V2_x = x2dotcmd - k2_1*ex2_ctrl
        V2_y = y2dotcmd - k2_2*ey2_ctrl
        V2_theta = theta2dotcmd - k2_3*etheta2_ctrl    
        
        V3_x = x3dotcmd - k3_1*ex3_ctrl
        V3_y = y3dotcmd - k3_2*ey3_ctrl
        V3_theta = theta3dotcmd - k3_3*etheta3_ctrl   
        
        return V1_x, V1_y, V1_theta, V2_x, V2_y, V2_theta, V3_x, V3_y, V3_theta
    
    def SeCtrl(self,t, dt):
        V = self.VelCtrl(t,dt)
        V1_1 = V[0]
        V1_2 = V[1]
        V1_3 = V[2]
        V2_1 = V[3]
        V2_2 = V[4]
        V2_3 = V[5]
        
        V3_1 = V[6]
        V3_2 = V[7]
        V3_3 = V[8]
        
    
        
        U1 = V1_2*math.cos(self.theta1meas)-V1_1*math.sin(self.theta1meas)
        W1 = V1_3
        U2 = V2_2*math.cos(self.theta2meas)-V2_1*math.sin(self.theta2meas)
        W2 = V2_3
        U3 = V3_2*math.cos(self.theta3meas)-V3_1*math.sin(self.theta3meas)
        W3 = V3_3
        
        self.timectrl.append(t)
        #print t
        #print "thetameas SeCtrl: [%f]"%(theta1meas)
        return U1, W1, U2, W2, U3, W3
        
        
    def callback1(self,msg):
        # Copying for simplicity
        position = msg.pose.position
        quat = msg.pose.orientation
        #rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
        #rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))
        x1meas_menos = position.x
        self.x1meas = -x1meas_menos
        self.x1.append(self.x1meas)
        self.y1meas = position.z
        self.y1.append(self.y1meas)
        #print x1meas, y1meas
        
        # Also print Roll, Pitch, Yaw
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        #rospy.loginfo("Euler Angles: %s"%str(euler))  
        angu = tuple(euler)
        roll = angu[0]
        absroll = abs(roll)
        pitch = angu[1]
        abspitch = abs(pitch)
        #yaw = angu[2]
        
        #print "Angulos: [%f,%f,%f]"%(roll,pitch,yaw)
        if pitch > 0:
            if absroll > 3:
                theta1measAux = math.pi*0.5 + (math.pi*0.5 - pitch)
            else:
                theta1measAux = pitch
        else:
            if absroll > 3:
                theta1measAux = math.pi + abspitch
            else:
                theta1measAux = 1.5*math.pi + math.pi*0.5 + pitch
                
        if theta1measAux > math.pi:
            theta1measAux2 = -(math.pi-(theta1measAux-math.pi))                
        else:
            theta1measAux2 = theta1measAux
            
        self.theta1meas_Aux2mem.append(theta1measAux2)
        ltheta1meas_Aux2mem=len(self.theta1meas_Aux2mem)
        b=ltheta1meas_Aux2mem-2
        ltheta1measmem=len(self.theta1measmem)
        ltheta1measmemc=ltheta1measmem-1
        
        etheta1measAux = theta1measAux2-self.theta1meas_Aux2mem[b]
        #absetheta1meas = abs(etheta1meas)
        if etheta1measAux > 6.2:
            self.theta1meas = self.theta1measmem[ltheta1measmemc]-(math.pi-abs(theta1measAux2))
        elif etheta1measAux < -6.2:
            self.theta1meas = self.theta1measmem[ltheta1measmemc]+(math.pi-abs(theta1measAux2))
        else:
            self.theta1meas = self.theta1measmem[ltheta1measmemc]+etheta1measAux
            
        self.theta1measmem.append(self.theta1meas)
        self.timerobotsmeas.append(self.tiempo)
            
    
    def callback2(self,msg):
        # Copying for simplicity
        position = msg.pose.position
        quat = msg.pose.orientation
        #rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
        #rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))
        x2meas_menos = position.x
        self.x2meas = -x2meas_menos
        self.x2.append(self.x2meas)
        self.y2meas = position.z
        self.y2.append(self.y2meas)
        #print x1meas, y1meas
        
        # Also print Roll, Pitch, Yaw
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        #rospy.loginfo("Euler Angles: %s"%str(euler))  
        angu = tuple(euler)
        roll = angu[0]
        absroll = abs(roll)
        pitch = angu[1]
        abspitch = abs(pitch)
        #yaw = angu[2]
        
        #print "Angulos: [%f,%f,%f]"%(roll,pitch,yaw)
        
        if pitch > 0:
            if absroll > 3:
                theta2measAux = math.pi*0.5 + (math.pi*0.5 - pitch)
            else:
                theta2measAux = pitch
        else:
            if absroll > 3:
                theta2measAux = math.pi + abspitch
            else:
                theta2measAux = 1.5*math.pi + math.pi*0.5 + pitch

        if theta2measAux > math.pi:
            theta2measAux2 = -(math.pi-(theta2measAux-math.pi))
        else:
            theta2measAux2 = theta2measAux
        
        self.theta2meas_Aux2mem.append(theta2measAux2)
        ltheta2meas_Aux2mem=len(self.theta2meas_Aux2mem)
        b=ltheta2meas_Aux2mem-2
        ltheta2measmem=len(self.theta2measmem)
        ltheta2measmemc=ltheta2measmem-1
        
        etheta2measAux = theta2measAux2-self.theta2meas_Aux2mem[b]
        #absetheta1meas = abs(etheta1meas)
        if etheta2measAux > 6.2:
            self.theta2meas = self.theta2measmem[ltheta2measmemc]-(math.pi-abs(theta2measAux2))
        elif etheta2measAux < -6.2:
            self.theta2meas = self.theta2measmem[ltheta2measmemc]+(math.pi-abs(theta2measAux2))
        else:
            self.theta2meas = self.theta2measmem[ltheta2measmemc]+etheta2measAux
            
        self.theta2measmem.append(self.theta2meas)
        #print self.theta2meas
        
    def callback3(self,msg):
        # Copying for simplicity
        position = msg.pose.position
        quat = msg.pose.orientation
        #rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
        #rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))
        x3meas_menos = position.x
        self.x3meas = -x3meas_menos
        self.x3.append(self.x3meas)
        self.y3meas = position.z
        self.y3.append(self.y3meas)
        #print x1meas, y1meas
        
        # Also print Roll, Pitch, Yaw
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        #rospy.loginfo("Euler Angles: %s"%str(euler))  
        angu = tuple(euler)
        roll = angu[0]
        absroll = abs(roll)
        pitch = angu[1]
        abspitch = abs(pitch)
        #yaw = angu[2]
        
        #print "Angulos: [%f,%f,%f]"%(roll,pitch,yaw)
        
        if pitch > 0:
            if absroll > 3:
                theta3measAux = math.pi*0.5 + (math.pi*0.5 - pitch)
            else:
                theta3measAux = pitch
        else:
            if absroll > 3:
                theta3measAux = math.pi + abspitch
            else:
                theta3measAux = 1.5*math.pi + math.pi*0.5 + pitch

        if theta3measAux > math.pi:
            theta3measAux2 = -(math.pi-(theta3measAux-math.pi))
        else:
            theta3measAux2 = theta3measAux
        
        self.theta3meas_Aux2mem.append(theta3measAux2)
        ltheta3meas_Aux2mem=len(self.theta3meas_Aux2mem)
        b=ltheta3meas_Aux2mem-2
        ltheta3measmem=len(self.theta3measmem)
        ltheta3measmemc=ltheta3measmem-1
        
        etheta3measAux = theta3measAux2-self.theta3meas_Aux2mem[b]
        #absetheta1meas = abs(etheta1meas)
        if etheta3measAux > 6.2:
            self.theta3meas = self.theta3measmem[ltheta3measmemc]-(math.pi-abs(theta3measAux2))
        elif etheta3measAux < -6.2:
            self.theta3meas = self.theta3measmem[ltheta3measmemc]+(math.pi-abs(theta3measAux2))
        else:
            self.theta3meas = self.theta3measmem[ltheta3measmemc]+etheta3measAux
            
        self.theta3measmem.append(self.theta3meas)
        
        #print self.theta3meas
            
    def Start(self):
        pub1 = rospy.Publisher('turtle1/cmd_vel_mux/input/navi', Twist, queue_size=100000)
        pub2 = rospy.Publisher('turtle2/cmd_vel_mux/input/navi', Twist, queue_size=100000)
        pub3 = rospy.Publisher('turtle3/cmd_vel_mux/input/navi', Twist, queue_size=100000)
        rospy.Subscriber("/vrpn_client_node/turtle1/pose", PoseStamped, self.callback1)
        rospy.Subscriber("/vrpn_client_node/turtle2/pose", PoseStamped, self.callback2)
        rospy.Subscriber("/vrpn_client_node/turtle3/pose", PoseStamped, self.callback3)
        rospy.init_node('Vel', anonymous=True)
        #rospy.init_node('goal_listener', anonymous=True)
        rate = rospy.Rate(100) # 100hz
        start_time = rospy.get_time()
        prev_time = start_time
        #Graf = Graficador.Graficar()
        #Graf.inicio(start_time)
        move_cmd1 = Twist()
        move_cmd2 = Twist()
        move_cmd3 = Twist()
        while not rospy.is_shutdown():
            #print self.theta2meas
            t = (rospy.get_time() - start_time)
            self.tiempo=t
            self.time.append(t)
            print t
            dt = rospy.get_time()-prev_time
            prev_time = rospy.get_time()
            #i = int(t)
            #gt = np.ones(loops)
            #gt[i]=i

        
            #rospy.Subscriber("/vrpn_client_node/turtle1/pose", PoseStamped, self.callback1)
            #rospy.Subscriber("/vrpn_client_node/turtle2/pose", PoseStamped, self.callback2)
            SC = self.SeCtrl(t,dt) 
            vel_ang_1A = SC[1]
            if vel_ang_1A > 1.0:
                vel_ang_1 = 1.0
            elif vel_ang_1A < -1.0:
                vel_ang_1 = -1.0
            else:
                vel_ang_1 = vel_ang_1A
                
                        
            vel_ang_2A = SC[3]
            if vel_ang_2A > 1.0:
                vel_ang_2 = 1.0
            elif vel_ang_2A < -1.0:
                vel_ang_2 = -1.0
            else:
                vel_ang_2 = vel_ang_2A
                
            vel_ang_3A = SC[5]
            if vel_ang_3A > 1.0:
                vel_ang_3 = 1
            elif vel_ang_3A < -1.0:
                vel_ang_3 = -1.0
            else:
                vel_ang_3 = vel_ang_3A
            #               
            
            move_cmd1.linear.x = -SC[0]
            move_cmd1.angular.z = vel_ang_1 #SC[1]#vel_ang_1
            move_cmd2.linear.x = -SC[2]
            move_cmd2.angular.z = vel_ang_2 #SC[3]
            move_cmd3.linear.x = -SC[4]
            move_cmd3.angular.z = vel_ang_3
            
            #move_cmd1.linear.x = -self.SeCtrl(t,dt)[0]
            #move_cmd1.angular.z = self.SeCtrl(t,dt)[1]#vel_ang_1
            #move_cmd2.linear.x = -self.SeCtrl(t,dt)[2]
            #move_cmd2.angular.z = self.SeCtrl(t,dt)[3]
            
            #print move_cmd1.linear.x,move_cmd1.angular.z
            #print move_cmd2.linear.x,move_cmd2.angular.z
            #print t,self.theta1meas
            #rospy.loginfo(move_cmd1)
            pub1.publish(move_cmd1)
            pub2.publish(move_cmd2)
            pub3.publish(move_cmd3)
            rate.sleep()
            
    def t1(self):
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/thetac.txt',"w")
        for elemento in self.ymem2:
           f.write('%s \n' % elemento)
        f.close()
        
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x1dotdest.txt',"w")
        for elemento in self.x1dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y1dotdest.txt',"w")
        for elemento in self.y1dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x2dotdest.txt',"w")
        for elemento in self.x2dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y2dotdest.txt',"w")
        for elemento in self.y2dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x3dotdest.txt',"w")
        for elemento in self.x3dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y3dotdest.txt',"w")
        for elemento in self.y3dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        #Velocidades lineales deseadas para el control
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x1dotdesc.txt',"w")
        for elemento in self.x1dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y1dotdesc.txt',"w")
        for elemento in self.y1dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x2dotdesc.txt',"w")
        for elemento in self.x2dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y2dotdesc.txt',"w")
        for elemento in self.y2dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x3dotdesc.txt',"w")
        for elemento in self.x3dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y3dotdesc.txt',"w")
        for elemento in self.y3dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        
    def t(self):
        #Tiempo
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/ControlesTurtle1/tiempo.txt',"w")
        for elemento in self.time:
           f.write('%s \n' % elemento)
        f.close()    
        
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/tiempo.txt',"w")
        for elemento in self.time:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/tiempoclusmeas.txt',"w")
        for elemento in self.timeclusmeas:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/tiemporobotsmeas.txt',"w")
        for elemento in self.timerobotsmeas:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/Errores/tiempo.txt',"w")
        for elemento in self.time:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/tiempo.txt',"w")
        for elemento in self.time:
           f.write('%s \n' % elemento)
        f.close() 
        
        #Trayectoria del robot virtual
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posturtle3xVir.txt',"w")
        for elemento in self.x3virtual:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posturtle3yVir.txt',"w")
        for elemento in self.y3virtual:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/thetaturtle3Vir.txt',"w")
        for elemento in self.theta3virtual:
           f.write('%s \n' % elemento)
        f.close() 
        
        #Trayectoria Robots
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/thetaturtle1.txt',"w")
        for elemento in self.theta1measmem:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posturtlex1.txt',"w")
        for elemento in self.x1:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posturtley1.txt',"w")
        for elemento in self.y1:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/thetaturtle2.txt',"w")
        for elemento in self.theta2measmem:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posturtlex2.txt',"w")
        for elemento in self.x2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posturtley2.txt',"w")
        for elemento in self.y2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/thetaturtle3.txt',"w")
        for elemento in self.theta3measmem:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posturtlex3.txt',"w")
        for elemento in self.x3:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posturtley3.txt',"w")
        for elemento in self.y3:
           f.write('%s \n' % elemento)
        f.close()
        
        #Trayectoria deseada cluster
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posclusterdesx.txt',"w")
        for elemento in self.xcdes:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posclusterdesy.txt',"w")
        for elemento in self.ycdes:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/thetaclusterdes.txt',"w")
        for elemento in self.thetacdes:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/phi1clusterdes.txt',"w")
        for elemento in self.phi1des:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/phi2clusterdes.txt',"w")
        for elemento in self.phi2des:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/phi3clusterdes.txt',"w")
        for elemento in self.phi3des:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/pclusterdes.txt',"w")
        for elemento in self.pdes:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/qclusterdes.txt',"w")
        for elemento in self.qdes:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/betaclusterdes.txt',"w")
        for elemento in self.betades:
           f.write('%s \n' % elemento)
        f.close()
        
        #Trayectoria medida cluster
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posclustermeasx.txt',"w")
        for elemento in self.xc:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/Posclustermeasy.txt',"w")
        for elemento in self.yc:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/thetaclustermeas.txt',"w")
        for elemento in self.thetac:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/phi1clustermeas.txt',"w")
        for elemento in self.phi1:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/phi2clustermeas.txt',"w")
        for elemento in self.phi2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/phi3clustermeas.txt',"w")
        for elemento in self.phi3:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/pclustermeas.txt',"w")
        for elemento in self.p:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/qclustermeas.txt',"w")
        for elemento in self.q:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/GraficasTurtle1/betaclustermeas.txt',"w")
        for elemento in self.beta:
           f.write('%s \n' % elemento)
        f.close()        
        
        #Errores del cluster
         
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/Errores/Errxc.txt',"w")
        for elemento in self.exc:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/Errores/Erryc.txt',"w")
        for elemento in self.eyc:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/Errores/Ethetac.txt',"w")
        for elemento in self.etheta:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/Errores/Errp.txt',"w")
        for elemento in self.ep:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/Errores/Errq.txt',"w")
        for elemento in self.eq:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/Errores/Errbeta.txt',"w")
        for elemento in self.ebeta:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/Errores/Errphi1.txt',"w")
        for elemento in self.ephi1:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/Errores/Errphi2.txt',"w")
        for elemento in self.ephi2:
           f.write('%s \n' % elemento)
        f.close() 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/Errores/Errphi3.txt',"w")
        for elemento in self.ephi3:
           f.write('%s \n' % elemento)
        f.close() 
        
        #angulos de los robots deseados
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta1des.txt',"w")
        for elemento in self.theta1dmem:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta2des.txt',"w")
        for elemento in self.theta2dmem:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta3des.txt',"w")
        for elemento in self.theta3dmem:
           f.write('%s \n' % elemento)
        f.close()
        
        #angulos de los robots deseados sin correccion
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta1desA.txt',"w")
        for elemento in self.theta1dAmem:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta2desA.txt',"w")
        for elemento in self.theta2dAmem:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta3desA.txt',"w")
        for elemento in self.theta3dmem:
           f.write('%s \n' % elemento)
        f.close()
        
        
        #Velocidades angulares deseadas 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x1dotdest.txt',"w")
        for elemento in self.x1dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y1dotdest.txt',"w")
        for elemento in self.y1dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x2dotdest.txt',"w")
        for elemento in self.x2dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y2dotdest.txt',"w")
        for elemento in self.y2dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x3dotdest.txt',"w")
        for elemento in self.x3dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y3dotdest.txt',"w")
        for elemento in self.y3dotdest:
           f.write('%s \n' % elemento)
        f.close()
        
#Velocidades angulares deseadas 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x1dotdest2.txt',"w")
        for elemento in self.x1dotdest2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y1dotdest2.txt',"w")
        for elemento in self.y1dotdest2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x2dotdest2.txt',"w")
        for elemento in self.x2dotdest2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y2dotdest2.txt',"w")
        for elemento in self.y2dotdest2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x3dotdest2.txt',"w")
        for elemento in self.x3dotdest2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y3dotdest2.txt',"w")
        for elemento in self.y3dotdest2:
           f.write('%s \n' % elemento)
        f.close()
        
        #Velocidades lineales deseadas para el control
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x1dotdesc.txt',"w")
        for elemento in self.x1dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y1dotdesc.txt',"w")
        for elemento in self.y1dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x2dotdesc.txt',"w")
        for elemento in self.x2dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y2dotdesc.txt',"w")
        for elemento in self.y2dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x3dotdesc.txt',"w")
        for elemento in self.x3dotdesc:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y3dotdesc.txt',"w")
        for elemento in self.y3dotdesc:
           f.write('%s \n' % elemento)
           
         #Posiciones deseadas 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x1des.txt',"w")
        for elemento in self.x1des:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y1des.txt',"w")
        for elemento in self.y1des:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x2des.txt',"w")
        for elemento in self.x2des:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y2des.txt',"w")
        for elemento in self.y2des:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/x3des.txt',"w")
        for elemento in self.x3des:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/y3des.txt',"w")
        for elemento in self.y3des:
           f.write('%s \n' % elemento)
        f.close()
           
        #Errores de los robots
           
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ex1.txt',"w")
        for elemento in self.ex1:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ex2.txt',"w")
        for elemento in self.ex2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ex3.txt',"w")
        for elemento in self.ex3:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ey1.txt',"w")
        for elemento in self.ey1:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ey2.txt',"w")
        for elemento in self.ey2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ey3.txt',"w")
        for elemento in self.ey3:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/etheta1.txt',"w")
        for elemento in self.etheta1:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/etheta2.txt',"w")
        for elemento in self.etheta2:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/etheta3.txt',"w")
        for elemento in self.etheta3:
           f.write('%s \n' % elemento)
        f.close()
        
        #Derivada de los angulos deseados
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta1dot.txt',"w")
        for elemento in self.theta1dot:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta2dot.txt',"w")
        for elemento in self.theta2dot:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta3dot.txt',"w")
        for elemento in self.theta3dot:
           f.write('%s \n' % elemento)
        f.close()
        
        #Derivadas deseadas cluster
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der1thetacd.txt',"w")
        for elemento in self.Der1thetacd:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der1xcdd.txt',"w")
        for elemento in self.Der1xcdd:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der1ycdd.txt',"w")
        for elemento in self.Der1ycdd:
           f.write('%s \n' % elemento)
        f.close()
        
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der2thetacd.txt',"w")
        for elemento in self.Der2thetacd:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der2xcdd.txt',"w")
        for elemento in self.Der2xcdd:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der2ycdd.txt',"w")
        for elemento in self.Der2ycdd:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der3thetacd.txt',"w")
        for elemento in self.Der3thetacd:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der3xcdd.txt',"w")
        for elemento in self.Der3xcdd:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der3ycdd.txt',"w")
        for elemento in self.Der3ycdd:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der4thetacd.txt',"w")
        for elemento in self.Der4thetacd:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der4xcdd.txt',"w")
        for elemento in self.Der4xcdd:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/Der4ycdd.txt',"w")
        for elemento in self.Der4ycdd:
           f.write('%s \n' % elemento)
        f.close()
        
        #Thetas deseadas delos robots 
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta1desc.txt',"w")
        for elemento in self.theta1dmem3:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta2desc.txt',"w")
        for elemento in self.theta2dmem3:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/theta3desc.txt',"w")
        for elemento in self.theta3dmem3:
           f.write('%s \n' % elemento)
        f.close()
        
        #Errores mapeados
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ex1map.txt',"w")
        for elemento in self.ex1_map:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ey1map.txt',"w")
        for elemento in self.ey1_map:
           f.write('%s \n' % elemento)
           
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ex2map.txt',"w")
        for elemento in self.ex2_map:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ey2map.txt',"w")
        for elemento in self.ey2_map:
           f.write('%s \n' % elemento)
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ex3map.txt',"w")
        for elemento in self.ex3_map:
           f.write('%s \n' % elemento)
        f.close()
        
        f = open ('/home/turtlebot_master/Dropbox/Tesis/Experimentacion/VelocidadesAngulares/ey3map.txt',"w")
        for elemento in self.ey3_map:
           f.write('%s \n' % elemento)
        
        
if __name__ == '__main__':
    try:
        Tr=Lemiscata(200.0,1.5,0.55,0.55,1.5708) #Periodo,Radio,pd,qd,betacd1.2217
        Tr.Start()
        Tr.t()
    except rospy.ROSInterruptException: pass
        
