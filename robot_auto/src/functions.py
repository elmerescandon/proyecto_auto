# -*- coding: utf-8 -*-

import numpy as np 

# Funciones de representaciones espaciales 
# para facilitar el movimiento del quadrotor

def rotaxis(a,axis):
	c = np.cos
	s = np.sin
	if axis == 'x':	
		R = np.array([[1,0,0],
					  [0,c(a),-s(a)],
					  [0,s(a),c(a)]])
	elif axis == 'y':	
		R = np.array([[c(a),0,s(a)],
					  [0,1,0],
					  [-s(a),0,c(a)]])
	elif axis == 'z':	
		R = np.array([[c(a),-s(a),0],
					  [s(a),c(a),0],
					  [0,0,1]])
	return R

def rot2quat(R):
    """
    Funcion que retorna el vector de quaterion unitario
    a partir de una matriz de rotacion.
    No considera la forma adicional de operar cuando el angulo es 180
    Lo de vuelve de la forma:
    q = (w,ex,ey,ez)
    """
    omega = ((1+R[0, 0]+R[1, 1]+R[2, 2])**0.5)*0.5
    ex = (1/(4*omega))*(R[2, 1]-R[1, 2])
    ey = (1/(4*omega))*(R[0, 2]-R[2, 0])
    ez = (1/(4*omega))*(R[1, 0]-R[0, 1])
    q = np.array([ex,ey,ez,omega])
    return q

def quat2axisangle(q):
	# Matriz que retorna el eje-ángulo asociado a la rotación
	e_ = np.linalg.norm(q[0:3])
	u = q[0:3]/e_
	theta = 2*np.arctan2(e_,q[3])
	return np.array([u,theta])