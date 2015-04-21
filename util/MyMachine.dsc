Transition	liftingTripod	tripodReady	firstHalf
Transition firstHalf          halfDone      secondHalf
Transition secondHalf         allDone       firstHalf
Transition firstHalf          noCommand     loweringTripod
Transition secondHalf         noCommand     loweringTripod
Transition loweringTripod     tripodDown    walkingDone
Transition walkingDone        standAdjEv    walkingDone
Initial liftingTripod
