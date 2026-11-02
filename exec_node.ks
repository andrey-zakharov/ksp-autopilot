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

function getStageBurnTime { parameter stageNum. parameter needDeltaV.


}

function sumBurnTime { parameter needDeltaV.
    local remainDeltaV to needDeltaV.
    local currentStage to SHIP:STAGENUM.
    local burnTime to 0.
    until remainDeltaV <= 0 or currentStage < 0 {
        local thr to 0.
        local isp to 0.
        local totalEngines to 0.
        for e in SHIP:ENGINES {
            if e:STAGE = currentStage and not e:FLAMEOUT {
                set thr to thr + e:AVAILABLETHRUST.
                set isp to isp + e:visp.
                set totalEngines to totalEngines + 1.
            }
        }

        if thr <= 0.01 {
            set thr to SHIP:MAXTHRUST.
        }
        print currentStage + " " + totalEngines + " " + thr + " " + isp.

        set isp to isp / totalEngines.
        local ve to isp * CONSTANT:g0.
        local accel to MAX(0.1, thr / MAX(0.001, SHIP:MASS) * 0.95). // approx, as counts all ship mass, not staged
        local dvMag to MIN(remainDeltaV, SHIP:STAGEDELTAV(currentStage):CURRENT).
        set burnTime to burnTime + MAX(1, (ve / accel) * (1 - (1 / (CONSTANT:E ^ (dvMag / ve))))).
        set remainDeltaV to remainDeltaV - dvMag.
        set currentStage to currentStage - 1.
        print burnTime + " " + remainDeltaV.
    }
    print sumBurnTime2(needDeltaV).
    return burnTime.
}

function sumBurnTime2 { parameter needDeltaV.
    local remainDeltaV to needDeltaV.
    local currentStage to SHIP:STAGENUM.
    local burnTime to 0.
    until remainDeltaV <= 0 or currentStage < 0 {
        local sdv to SHIP:STAGEDELTAV(currentStage).
        print "stage: " + currentStage + " " + sdv:CURRENT + " " + sdv:DURATION.
        // 940 m/s
        //stage: 3 240.063385009766 10.1149234771729
        //stage: 2 2215.4560546875 563.882385253906
        //stage: 1 2330.35424804688 225.552947998047
        //stage: 0 0 0

        if sdv:CURRENT >= remainDeltaV {
            set burnTime to burnTime + (remainDeltaV * sdv:DURATION / sdv:CURRENT).
            set remainDeltaV to 0.
        } else { // not enough fuel in current stage
            set remainDeltaV to remainDeltaV - sdv:CURRENT.
            set burnTime to burnTime + sdv:DURATION.
        }
        print burnTime + " " + remainDeltaV.
        
        set currentStage to currentStage - 1.
    }
    return burnTime.
}

function execMain {
    sas off.
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

    set WARP to 0.
    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    unlock THROTTLE.
    unlock STEERING.

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
    set SAS to true.

    return true.
}

execMain().