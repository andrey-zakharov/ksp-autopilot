@LAZYGLOBAL ON.

parameter targetNode is 0.

function execAlive { parameter nd.
    for n in ALLNODES {
        if n = nd {
            return true.
        }
    }
    return false.
}

function execAim { parameter aimV.
    local aimH to aimV:NORMALIZED.
    local tEnd to TIME:SECONDS + 20.

    wait until VANG(SHIP:FACING:FOREVECTOR, aimH) <= 1 or TIME:SECONDS > tEnd.
    PRINT "Aligned (angle=" + ROUND(VANG(SHIP:FACING:FOREVECTOR, aimH), 2) + "deg).".
}

function sumBurnTime { parameter needDeltaV.
    local remainDeltaV to needDeltaV.
    local currentStage to SHIP:STAGENUM.
    local burnTime to 0.
    until remainDeltaV <= 0 or currentStage < 0 {
        local sdv to SHIP:STAGEDELTAV(currentStage).
        //print "stage: " + currentStage + " " + sdv:CURRENT + " " + sdv:DURATION.

        if sdv:CURRENT >= remainDeltaV {
            set burnTime to burnTime + (remainDeltaV * sdv:DURATION / sdv:CURRENT).
            set remainDeltaV to 0.
        } else { // not enough fuel in current stage
            set remainDeltaV to remainDeltaV - sdv:CURRENT.
            set burnTime to burnTime + sdv:DURATION.
        }
        //print burnTime + " " + remainDeltaV.
        
        set currentStage to currentStage - 1.
    }
    return burnTime.
}

function execMain {
    sas off.
    set WARP to 0.
    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    unlock THROTTLE.
    unlock STEERING.
    if targetNode = 0 {
        if not HASNODE {
            print "No node.".
            return false.
        }
        set targetNode to NEXTNODE.
    }

    if not execAlive(targetNode) {
        print "No node.".
        return false.
    }

    local dvMag to targetNode:DELTAV:MAG.

    if dvMag <> dvMag or dvMag < 0.1 {
        print "Bad node dV.".
        return false.
    }

    local burnTime to sumBurnTime(dvMag).
    local startEta to burnTime / 2 + MAX(2, MIN(20, SHIP:MASS / 10)).



    lock STEERING to targetNode:DELTAV.
    execAim(targetNode:DELTAV).

    print "NODE dV=" + ROUND(dvMag, 1) +
          " burn=" + ROUND(burnTime, 1) +
          " T-" + ROUND(startEta, 1).

    kuniverse:timewarp:warpto(targetNode:time - startEta).

    if not execAlive(targetNode) {
        print "Node disappeared.".
        unlock STEERING.
        return false.
    }

    execAim(targetNode:DELTAV).

    if not execAlive(targetNode) {
        unlock STEERING.
        return false.
    }

    wait until time:seconds >= targetNode:time - burnTime / 2.
    print "Burning...".

    until not execAlive(targetNode) or targetNode:DELTAV:MAG <= 0.5 {
        if targetNode:DELTAV:MAG > 10 {
            set SHIP:CONTROL:PILOTMAINTHROTTLE to 1.
        } else if targetNode:DELTAV:MAG > 2 {
            set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.3.
        } else {
            set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.1.
        }

        wait 0.05.
    }

    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    unlock THROTTLE.

    if execAlive(targetNode) {
        remove targetNode.
        print "Node executed successfully!".
    }

    unlock STEERING.
    sas on.

    return true.
}

execMain().