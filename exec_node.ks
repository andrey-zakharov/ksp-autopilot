@LAZYGLOBAL ON.
RUNONCEPATH("math").
RUNONCEPATH("dis").
RUNONCEPATH("stg").
RUNONCEPATH("m_lib").


parameter targetNode is 0.
parameter dvLimit is 0.5. // what limit to check
parameter autoStage is TRUE.

function execAlive { parameter nd.
    for n in ALLNODES {
        if n = nd {
            return true.
        }
    }
    return false.
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
    execAim(targetNode:DELTAV, targetNode:time - startEta - 5 - time:seconds).

    print "NODE dV=" + ROUND(dvMag, 1) +
          " burn=" + ROUND(burnTime, 1) +
          " T-" + ROUND(startEta, 1).

    kuniverse:timewarp:warpto(targetNode:time - startEta - 5).

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

    print "waiting " + disTime(targetNode:time - burnTime / 2 - TIME:SECONDS).
    wait until time:seconds >= targetNode:time - burnTime / 2.
    lock minDvLimit to 5 * 0.05 * SHIP:AVAILABLETHRUST / SHIP:MASS.  // один тик цикла ниже - в пять раз больше - лимит  прожига
    print "Burning with minDv= " + round(minDvLimit, 1) + "m/s...".

    local nextStageTime is TIME:SECONDS + 1.

    until not execAlive(targetNode) or targetNode:DELTAV:MAG <= dvLimit {
        set now to TIME:SECONDS.
        if autoStage AND now >= nextStageTime AND shouldStage() {
            STAGE.
            SET nextStageTime TO now + 1.
            wait 0.
        }

        local thrCmd to 1.
        if targetNode:DELTAV:MAG < minDvLimit {
            // Плавная тяга пропорционально остатку
            local acc to MAX(0.1, SHIP:MAXTHRUST / SHIP:MASS).
            set thrCmd to clamp(targetNode:DELTAV:MAG / acc, 0.05, 1).
        }


        set SHIP:CONTROL:PILOTMAINTHROTTLE to thrCmd.
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