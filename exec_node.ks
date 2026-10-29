// exec_node.ks
RUN utils.
SAS OFF.

FUNCTION getAlignmentAngle { PARAMETER nd.
    LOCAL shipVec TO SHIP:FACING:FOREVECTOR.
    IF nd:DELTAV:MAG < 0.01 { RETURN 0. }
    LOCAL targetVec TO nd:DELTAV:NORMALIZED.
    LOCAL dotVal TO VDOT(shipVec, targetVec).
    SET dotVal TO clamp(dotVal, -1, 1).
    RETURN ARCCOS(dotVal).
}

// CLEARSCREEN.
PRINT "=== Maneuver Node Execution ===".

IF not hasnode {
    PRINT "No maneuver nodes found! Exiting.".
    SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
    EXIT.
}

PRINT "Node found. Required dV: " + ROUND(nextnode:DELTAV:MAG, 2) + " m/s".

SET accel TO MAX(0.1, SHIP:MAXTHRUST / SHIP:MASS).
SET burnTime TO nextnode:DELTAV:MAG / accel.
SET startBurnETA TO (burnTime / 2) + 5.
PRINT "Estimated burn time: " + ROUND(burnTime, 1) + "s".
PRINT "Starting burn at T-" + ROUND(startBurnETA, 1) + "s".

// Разворот ДО варпа
LOCK STEERING TO nextnode:DELTAV.
PRINT "Aligning to burn vector...".

SET alignStart TO TIME:SECONDS.
UNTIL getAlignmentAngle(nextnode) < 2 OR (TIME:SECONDS - alignStart > 120) OR not hasnode {
    WAIT 0.5.
}

IF not hasnode {
    PRINT "Node disappeared during alignment. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

IF getAlignmentAngle(nextnode) >= 2 {
    PRINT "WARNING: Align timeout (angle=" + ROUND(getAlignmentAngle(nextnode), 1) + "deg). Proceeding.".
} ELSE {
    PRINT "Aligned (angle=" + ROUND(getAlignmentAngle(nextnode), 2) + "deg).".
}

// Варп только после выравнивания
IF nextnode:ETA > startBurnETA + 30 {
    PRINT "Warping to burn start...".
    SET WARP TO 3.
    WAIT UNTIL nextnode:ETA <= startBurnETA + 30 OR not hasnode.
    SET WARP TO 0.
    IF not hasnode {
        PRINT "Node disappeared during warp. Exiting.".
        UNLOCK STEERING.
        EXIT.
    }
    IF getAlignmentAngle(nextnode) >= 2 {
        PRINT "Re-aligning after warp...".
        SET alignStart TO TIME:SECONDS.
        UNTIL getAlignmentAngle(nextnode) < 2 OR (TIME:SECONDS - alignStart > 60) OR not hasnode {
            WAIT 0.5.
        }
    }
}

// Финальная проверка
IF not hasnode {
    PRINT "Node disappeared. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

SET alignStart TO TIME:SECONDS.
UNTIL getAlignmentAngle(nextnode) < 2 OR (TIME:SECONDS - alignStart > 30) OR not hasnode {
    WAIT 0.1.
}

IF not hasnode {
    PRINT "Node disappeared. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

IF getAlignmentAngle(nextnode) >= 2 {
    PRINT "WARNING: Final angle=" + ROUND(getAlignmentAngle(nextnode), 1) + "deg. Proceeding.".
}

WAIT UNTIL nextnode:ETA <= startBurnETA OR not hasnode.
IF not hasnode {
    PRINT "Node disappeared. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

PRINT "Burning...".
UNTIL not hasnode OR nextnode:DELTAV:MAG <= 0.5 {
    IF nextnode:DELTAV:MAG > 10 {
        SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 1.
    } ELSE IF nextnode:DELTAV:MAG > 2 {
        SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.3.
    } ELSE {
        SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.1.
    }
    WAIT 0.05.
}

SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
IF hasnode {
    REMOVE nextnode.
    PRINT "Node executed successfully!".
}

UNLOCK STEERING.
SAS ON.