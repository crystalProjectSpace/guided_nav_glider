'use strict'
const { Vect3D, Plane } = require('./vect3.js')
// adx for small sub/transsonic drone with long thin wing
// cxa0 = 0.025, Apolar = 0.16, dCdY = 0.08
const ALPHA_SAFE = 25.5
const g = 9.81
const GLOBAL_VERT = Vect3D.fromXYZ(0, 1, 0)

const GROUND_PLANE = new Plane(
	Vect3D.fromXYZ(0, 0, 0),
	GLOBAL_VERT,
)

const linterp = function(YV, XV, X) {
	let i = 0
	while(XV[i + 1] < X) { i++ }
	const i1 = i + 1
	
	return YV[i] + (YV[i1] - YV[i]) * (X - XV[i]) / (XV[i1] - XV[i])
}

const getRo = function(H) {
	return 1.2 * Math.exp(-0.0001261 * H)
}

const targetingError = function(coord, velocity, target) {
	const delta_cr = target.subt(coord)
	const lateral_axis = GLOBAL_VERT.cross(delta_cr)
	const vert_axis = lateral_axis.cross(delta_cr)

	const collisionPlane = new Plane(target, delta_cr)
	const targetingPlane = new Plane(target, lateral_axis)
	const localHorizon = new Plane(target, vert_axis)

	const collision = collisionPlane.lineIntersect(velocity, coord)
	const cross = localHorizon.projectPoint(collision).subt(coord)
	const ascend = targetingPlane.projectPoint(collision).subt(coord)
	return [velocity.angle(cross), velocity.angle(ascend), coord.subt(target).abs()]
}

const derivatives = function(state, params, controls, t) {
	const alpha = controls.AoA(state, t)
	const gamma = controls.roll(state, t)
	
	const V = state[0]
	const Th = state[1]
	const Psi = state[2]
	const H = state[4]
	const CTH = Math.cos(Th)
	const STH = Math.sin(Th)
	
	const CPSI = Math.cos(Psi)
	const SPSI = Math.sin(Psi)
	
	const CG = Math.cos(gamma)
	const SG = Math.sin(gamma)
	
	const Ro = getRo(H)
		
	const QS = 0.5 * Ro * V * V * params.S

	const Xa = params.Cxa(alpha) * QS
	const Ya = params.Cya(alpha) * QS
	const MV = params.m * V
		
	return [
		(- params.m * g * STH - Xa) / params.m,
		(Ya * CG - params.m * g * CTH) / MV,
		Ya * SG / MV,
		V * CTH * CPSI,
		V * STH,
		V * CTH * SPSI,
	]
}

const integrate = function(initialState, params, controls, observe, flag, dT) {
	const result = [{t: 0, state: initialState.slice()}]
	const K_buffer = [0, 0, 0, 0, 0, 0]
	const dT_05 = 0.5 * dT
	let i = 0
	let tau = 0
	
	while(!flag(result[i].state, tau)) {
		const activeState = result[i].state
		observe(activeState, dT)
		
		const deriv_0 = derivatives(activeState, params, controls, tau)

		K_buffer[0] = activeState[0] + deriv_0[0] * dT
		K_buffer[1] = activeState[1] + deriv_0[1] * dT
		K_buffer[2] = activeState[2] + deriv_0[2] * dT
		K_buffer[3] = activeState[3] + deriv_0[3] * dT
		K_buffer[4] = activeState[4] + deriv_0[4] * dT
		K_buffer[5] = activeState[5] + deriv_0[5] * dT

		const deriv_1 = derivatives(K_buffer, params, controls, tau)
		tau += dT
		i++
		result.push({
			state: [
				activeState[0] + (deriv_0[0] + deriv_1[0]) * dT_05,
				activeState[1] + (deriv_0[1] + deriv_1[1]) * dT_05,
				activeState[2] + (deriv_0[2] + deriv_1[2]) * dT_05,
				activeState[3] + (deriv_0[3] + deriv_1[3]) * dT_05,
				activeState[4] + (deriv_0[4] + deriv_1[4]) * dT_05,
				activeState[5] + (deriv_0[5] + deriv_1[5]) * dT_05,
			],
			t: tau
		})
	}
	
	return result
}

const createObserver = function(targeting, ctrlPoint) {
	const coords = Vect3D.fromXYZ(0, 0, 0)
	const VECT = Vect3D.fromXYZ(0, 0, 0)

	return function(state, dT) {
		const CTH = Math.cos(state[1])
		const STH = Math.sin(state[1])
		const CPSI = Math.cos(state[2])
		const SPSI = Math.sin(state[2])

		coords[0] = state[3]
		coords[1] = state[4]
		coords[2] = state[5]

		VECT[0] = CTH * CPSI
		VECT[1] = STH 
		VECT[2] = CTH * SPSI
		
		const cr_projection = GROUND_PLANE.projectPoint(coords)
		const v_projection = GROUND_PLANE.projectPoint(VECT)
		const delta_cr = coords.subt(ctrlPoint)

		if (targeting.visActive) {
			targeting.visPrev[0] = targeting.visActive[0]
			targeting.visPrev[1] = targeting.visActive[1]
			targeting.visPrev[2] = targeting.visActive[2]
		}

		targeting.visActive = ctrlPoint.subt(cr_projection)
		
		const ascend = v_projection.angle(VECT)
		targeting.dVisAngle = targeting.visPrev ? (targeting.visActive.angle(targeting.visPrev)) / dT : 0
		targeting.ascend = ascend
		targeting.distance = delta_cr.abs() * Math.cos(ascend)
		targeting.miss = targetingError(coords, VECT, ctrlPoint)
	}
}

 const createAlphaFunc = function(ctrlParams, vehicle) {
	const { alphaBase, kTh } = ctrlParams
	return function(state, t) {
		if(t < 2.5) return 0
		const { targeting } = vehicle
		let alpha = targeting.alpha
		let dAlpha = 0
		if (targeting.distance > 2000) {
			dAlpha = state[1] * 57.3 * kTh
		} else {
			dAlpha = -targeting.miss[1] * 57.3 * 5
//			console.log(state[1] * 57.3, (targeting.miss[1]*57.3).toFixed(2))			
		}
		if(dAlpha < -12.5) return -12.5
		if(dAlpha > 12.5) return 12.5
		alpha += dAlpha
		if(alpha < -ALPHA_SAFE) return -ALPHA_SAFE
		if(alpha > ALPHA_SAFE) return ALPHA_SAFE
		targeting.alpha = alpha
		return targeting.alpha
	}
}


const createGammaFunc = function(ctrlParams, vehicle) {
	const { rollStart, rollEnd, rollPerSecond } = ctrlParams
	return function(state, t) {
		const targeting = vehicle.targeting
		if(t < rollStart) return 0
		if(t > rollEnd) return 0
		let dGamma = 0
		if(vehicle.targeting.distance < 2000) {
			dGamma = -targeting.miss[0] * 0
		} else {
			dGamma = targeting.dVisAngle * 375.
		}
		
		if(dGamma < -rollPerSecond) dGamma = -rollPerSecond
		if(dGamma > rollPerSecond) dGamma = rollPerSecond
		let res = targeting.gamma + dGamma
		if(res > 1.57) res = 1.57
		if(res < -1.57) res = -1.57
		targeting.gamma = res
		return targeting.gamma
	}
}

module.exports = {
	linterp,
	createAlphaFunc,
	createGammaFunc,
	createObserver,
	integrate,
}