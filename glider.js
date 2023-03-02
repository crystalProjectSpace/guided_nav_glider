'use strict'

const {
	linterp,
	createAlphaFunc,
	createGammaFunc,
	createObserver,
	integrate,
} = require('./utils.js')

const { Vect3D } = require('./vect3.js')

class Glider {
	constructor(m, S, CXA_vect, CYA_vect, AOA_vect) {
		this.params = {
			S,
			m,
			Cxa: function(alpha) {
				return linterp(CXA_vect, AOA_vect, alpha)
			},
			Cya: function(alpha) {
				return linterp(CYA_vect, AOA_vect, alpha)
			},
		}
		
		this.controls = {
			AoA: null,
			roll: null,
		}
		
		this.observe = null
		
		this.targeting = {
			visActive: null,
			visPrev: Vect3D.fromXYZ(0, 0, 0),
			dVisAngle: 0,
			distance: 1E10,
			ascend: 0,
			alpha: 0,
			gamma: 0,
			dive: 0,
			miss: 0,
		}
	}
	
	setPitchControl(pitchFunction) {
		this.controls.AoA = pitchFunction
	}
	
	setRollControl(rollFunction) {
		this.controls.roll = rollFunction
	}
	
	setObserver(observeFunction, ctrlPoint) {
		this.observe = observeFunction(this.targeting, ctrlPoint)
	}
	
	getTrajectory(initialState, flag, dT) {
		return integrate(
			initialState,
			this.params,
			this.controls,
			this.observe,
			flag,
			dT,
		)
	}
}

const defaultFlightEnd = function(state, t) {
	return state[4] < 0
}

const Cxa_glider = [0.9466,	0.4346,	0.2554,	0.1274,	0.0905, 0.0619, 0.0414, 0.0291, 0.0250, 0.0291, 0.0414, 0.0619, 0.0905, 0.1274, 0.2554, 0.4346, 0.9466]
const Cya_glider = [-2.4000, -1.6000, -1.2000, -0.8000, -0.6400, -0.4800, -0.3200, -0.1600, 0.0000, 0.1600, 0.3200, 0.4800, 0.6400, 0.8000, 1.2000, 1.6000, 2.4000]
const AoA_glider = [-30,		-20,	-15,		-10,	-8,		-6,		-4,		- 2,		0,	2,		4,		6,		8,		10,		15,		20,		30]
const M0 = 75
const S0 = 0.5
const dT_base = 0.05

const INIT_STATE = [425, 5/57.3, 0, 0, 1750, 0]

const ALPHA_CTRL_PRM = {
	alphaBase: 2.15,
	kTh: -10.95,
	alphaDive: -3.5,
}

const GAMMA_CTRL_PRM = {
	rollStart: 20.5,
	rollEnd: 200,
	rollPerSecond: 45  * 0.1 / 57.3,
}

const testGlider = new Glider(M0, S0, Cxa_glider, Cya_glider, AoA_glider)
testGlider.setPitchControl(createAlphaFunc(ALPHA_CTRL_PRM, testGlider))
testGlider.setRollControl(createGammaFunc(GAMMA_CTRL_PRM, testGlider))
testGlider.setObserver(createObserver, Vect3D.fromXYZ(4500, 0, -4500))

const gliderTrajectory = testGlider.getTrajectory(INIT_STATE, defaultFlightEnd, dT_base)

const response = gliderTrajectory.reduce((res, row, i) => {
	const { state, t } = row
	if (i % 10 !== 0) return res
	res += [
		t.toFixed(1),
		state[0].toFixed(1),
		(state[1] * 57.3).toFixed(2),
		(state[2] * 57.3).toFixed(2),
		state[3].toFixed(0),
		state[4].toFixed(0),
		state[5].toFixed(0),
	].join('\t') + '\n'
	
	return res
}, '')

console.log(response)
