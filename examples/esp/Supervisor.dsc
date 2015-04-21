Transition	unCalibrated	startCommand	calibrating
Transition	calibrating	calFail		uncalibrated
Transition	calibrating	calSuccess	standing
Transition	standing	doneStanding	ready
Transition	ready		accWalkCommand	accelerating
Transition	ready		walkCommand	walking
Transition	accelerating	upToSpeed	walking
Transition	accelerating	noCommand	ready
Transition	walking		noCommand	ready
Transition	walking		stopCommand	decellerating
Transition	decelerating	doneDecel	ready
Initial		unCalibrated