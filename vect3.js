'use strict'

class Vect3D extends Array {
	static fromXYZ(X, Y, Z) {
		const result = new Vect3D()
		result[0] = X
		result[1] = Y
		result[2] = Z
		return result
	}
	
	static fromVect(U) {
		const result = new Vect3D()
		result[0] = U[0]
		result[1] = U[1]
		result[2] = U[2]
		return result
	}
	
	summ(U) {
		const result = new Vect3D()
		result[0] = this[0] + U[0]
		result[1] = this[1] + U[1]
		result[2] = this[2] + U[2]
		return result
	}
	
	subt(U) {
		const result = new Vect3D()
		result[0] = this[0] - U[0]
		result[1] = this[1] - U[1]
		result[2] = this[2] - U[2]
		return result		
	}
	
	byScalar(k) {
		const result = new Vect3D()
		result[0] = this[0] * k
		result[1] = this[1] * k
		result[2] = this[2] * k
		return result	
	}
	
	dot(U) {
		return this[0] * U[0] + this[1] * U[1] + this[2] * U[2]
	}
	
	cross(U) {
		const result = new Vect3D()
		result[0] = this[1] * U[2] - this[2] * U[1]
		result[1] = this[2] * U[0] - this[0] * U[2]
		result[2] = this[0] * U[1] - this[1] * U[0]
		return result
	}
	
	abs(){
		return Math.sqrt(this[0] * this[0] + this[1] * this[1] + this[2] * this[2])
	}
	
	delta(U) {
		const dX = this[0] - U[0]
		const dY = this[1] - U[1]
		const dZ = this[2] - U[2]		
		return Math.sqrt(dX * dX + dY * dY + dZ * dZ)
	}
	
	normalize() {
		const result = new Vect3D()
		const _abs = 1 / Math.sqrt(this[0] * this[0] + this[1] * this[1] + this[2] * this[2])
		result[0] = this[0] * _abs
		result[1] = this[1] * _abs
		result[2] = this[2] * _abs
		return result
	}
	
	angle(U, orientir = DEFAULT_ORIENTIR) {
		const crossUV = this.cross(U)
		const absCrossUV = crossUV.abs()
		const dotUV = this.dot(U)
		const sign = crossUV.dot(orientir) > 0 ? 1 : -1
		return Math.atan(absCrossUV / dotUV) * sign
	}

	clone() {
		const result = new Vect3D()
		result[0] = this[0]
		result[1] = this[1]
		result[2] = this[2]
		return result
	}
}

const DEFAULT_ORIENTIR = Vect3D.fromXYZ(0.0, 1.0, 0.0)

class Plane {
	static fromPoints(A, B, C) {
		const BA = B.subt(A)
		const BC = B.subt(C)
		norm = BA.cross(BC).normalize()
		point = B
		return new Plane(point, norm)
	}
	
	static fromVects(U, V) {
		const norm = U.cross(V).normalize()
		const point = Vect3D.fromXYZ(0, 0, 0)
	}
	
	constructor(P, N) {
		this.point = Vect3D.fromVect(P)
		this.norm = Vect3D.fromVect(N)
	}
	
	reset(P, N) {
		this.point = Vect3D.fromVect(P)
		this.norm = Vect3D.fromVect(N)
		return this
	}
	
	projectPoint(P) {
		const t = this.point.subt(P).dot(this.norm) / this.norm.dot(this.norm)
		return P.summ(this.norm.byScalar(t))
	}

	lineIntersect(direct, initPoint) {
		const t = this.point.subt(initPoint).dot(this.norm) / this.norm.dot(direct)
		return initPoint.summ(direct.byScalar(t))
	}
	
	getPlaneAngle(P) {
		return this.n.angle(P.n)
	}
}

module.exports = {
	Vect3D,
	Plane,
}
