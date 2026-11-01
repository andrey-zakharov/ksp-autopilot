@lazyglobal off.

// exec_node.ks
parameter targetNode is nextnode.

run utils.

function execNodeExists { parameter eNode.
    for nd in ALLNODES {
        if nd = eNode {
            return true.
        }
    }
    return false.
}

function sumAvThrust {
    // Реальная тяга активных двигателей с учётом thrustLimit, без включения
    local totalThrust TO 0.
    FOR eng IN SHIP:ENGINES {
        IF eng:IGNITION AND NOT eng:FLAMEOUT {
            SET totalThrust TO totalThrust + eng:AVAILABLETHRUST.
        }
    }
    IF totalThrust <= 0.01 { return SHIP:MAXTHRUST. }
    return totalThrust.
}

function getAlignmentAngle { PARAMETER nd.
    IF nd:DELTAV:MAG < 0.01 { RETURN 0. }
    LOCAL dotVal TO VDOT(SHIP:FACING:FOREVECTOR, nd:DELTAV:NORMALIZED).
    SET dotVal TO clamp(dotVal, -1, 1).
    RETURN ARCCOS(dotVal).
}

function execMain {
    local nodeTime to targetNode:TIME.
    local dvVec to targetNode:DELTAV.
    local dvMag to dvVec:MAG.

    if not isFiniteVal(dvMag) or dvMag < 0.1 {
        remove targetNode.
        return true.
    }

    local accel to MAX(0.1, sumAvThrust() / MAX(0.001, SHIP:MASS) * 0.95).
    local burnTime to MAX(1, dvMag / accel * 1.10).
    local startEta to burnTime / 2.

    set WARP to 0.
    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    // Разворот ДО варпа
    local faceVec to safeLookDirUp(dvVec, SHIP:UP:FOREVECTOR).
    lock steering to faceVec.

    print "NODE dV=" + ROUND(dvMag, 1) +
          " burn=" + ROUND(burnTime, 1) +
          " T-" + ROUND(startEta, 1).

    local alignStart is TIME:SECONDS.
    until getAlignmentAngle(targetNode) < 1 OR (TIME:SECONDS - alignStart > 120) OR not hasnode {
        wait 0.5.
    }
    // Варп только после выравнивания
    until targetNode:ETA <= startEta + 30 {
        if targetNode:ETA > 600 {
            set WARP to 4.
        } else if targetNode:ETA > 120 {
            set WARP to 2.
        } else {
            set WARP to 0.
        }

        set faceVec to safeLookDirUp(dvVec, SHIP:UP:FOREVECTOR).
        wait 0.2.
    }

    set WARP to 0.
    set faceVec to safeLookDirUp(dvVec, SHIP:UP:FOREVECTOR).
    if getAlignmentAngle(targetNode) >= 2 {
        PRINT "Re-aligning after warp...".
        SET alignStart TO TIME:SECONDS.
        UNTIL getAlignmentAngle(targetNode) < 1 OR (TIME:SECONDS - alignStart > 60) OR not hasnode {
            WAIT 0.5.
        }
    }
    wait 0.5.
    WAIT UNTIL targetNode:ETA <= startEta OR not hasnode.
    lock throttle to 1.

    local burnStart to TIME:SECONDS.
    local doneBurn to false.

    until doneBurn {
        if not execNodeExists(targetNode) {
            set doneBurn to true.
        } else {
            local remDv to targetNode:DELTAV:MAG.

            if remDv < 0.5 {
                set doneBurn to true.
            } else if TIME:SECONDS - burnStart > burnTime * 1.5 {
                set doneBurn to true.
            } else {
                wait 0.1.
            }
        }
    }

    set SHIP:CONTROL:PILOTMAINTHROTTLE to 0.
    unlock throttle.
    unlock steering.
    set SAS to false.

    remove targetNode.
    wait 0.5.

    return true.
}

execMain().