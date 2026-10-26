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

IF ALLNODES:LENGTH = 0 {
    PRINT "No maneuver nodes found! Exiting.".
    SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
    EXIT.
}

SET activeNode TO ALLNODES[0].

IF activeNode:DELTAV:MAG < 0.5 {
    PRINT "Node dV negligible (" + ROUND(activeNode:DELTAV:MAG, 2) + " m/s). Removing.".
    REMOVE activeNode.
    EXIT.
}

PRINT "Node found. Required dV: " + ROUND(activeNode:DELTAV:MAG, 2) + " m/s".

SET accel TO MAX(0.1, SHIP:MAXTHRUST / SHIP:MASS).
SET burnTime TO activeNode:DELTAV:MAG / accel.
SET startBurnETA TO (burnTime / 2) + 5.
PRINT "Estimated burn time: " + ROUND(burnTime, 1) + "s".
PRINT "Starting burn at T-" + ROUND(startBurnETA, 1) + "s".

// Разворот ДО варпа
LOCK STEERING TO activeNode:DELTAV.
PRINT "Aligning to burn vector...".

SET alignStart TO TIME:SECONDS.
UNTIL getAlignmentAngle(activeNode) < 2 OR (TIME:SECONDS - alignStart > 120) OR ALLNODES:LENGTH = 0 {
    WAIT 0.5.
}

IF ALLNODES:LENGTH = 0 {
    PRINT "Node disappeared during alignment. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

IF getAlignmentAngle(activeNode) >= 2 {
    PRINT "WARNING: Align timeout (angle=" + ROUND(getAlignmentAngle(activeNode), 1) + "deg). Proceeding.".
} ELSE {
    PRINT "Aligned (angle=" + ROUND(getAlignmentAngle(activeNode), 2) + "deg).".
}

// Варп только после выравнивания
IF activeNode:ETA > startBurnETA + 30 {
    PRINT "Warping to burn start...".
    SET WARP TO 3.
    WAIT UNTIL activeNode:ETA <= startBurnETA + 30 OR ALLNODES:LENGTH = 0.
    SET WARP TO 0.
    IF ALLNODES:LENGTH = 0 {
        PRINT "Node disappeared during warp. Exiting.".
        UNLOCK STEERING.
        EXIT.
    }
    IF getAlignmentAngle(activeNode) >= 2 {
        PRINT "Re-aligning after warp...".
        SET alignStart TO TIME:SECONDS.
        UNTIL getAlignmentAngle(activeNode) < 2 OR (TIME:SECONDS - alignStart > 60) OR ALLNODES:LENGTH = 0 {
            WAIT 0.5.
        }
    }
}

// Финальная проверка
IF ALLNODES:LENGTH = 0 {
    PRINT "Node disappeared. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

SET alignStart TO TIME:SECONDS.
UNTIL getAlignmentAngle(activeNode) < 2 OR (TIME:SECONDS - alignStart > 30) OR ALLNODES:LENGTH = 0 {
    WAIT 0.1.
}

IF ALLNODES:LENGTH = 0 {
    PRINT "Node disappeared. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

IF getAlignmentAngle(activeNode) >= 2 {
    PRINT "WARNING: Final angle=" + ROUND(getAlignmentAngle(activeNode), 1) + "deg. Proceeding.".
}

WAIT UNTIL activeNode:ETA <= startBurnETA OR ALLNODES:LENGTH = 0.
IF ALLNODES:LENGTH = 0 {
    PRINT "Node disappeared. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

PRINT "Burning...".
UNTIL ALLNODES:LENGTH = 0 OR activeNode:DELTAV:MAG <= 0.5 {
    IF activeNode:DELTAV:MAG > 10 {
        SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 1.
    } ELSE IF activeNode:DELTAV:MAG > 2 {
        SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.3.
    } ELSE {
        SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.1.
    }
    WAIT 0.05.
}

SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
IF ALLNODES:LENGTH > 0 {
    REMOVE activeNode.
    PRINT "Node executed successfully!".
}

UNLOCK STEERING.
SAS ON.