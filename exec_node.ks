// exec_node.ks
// Выполняет маневренную ноду: разворот ДО варпа, SAS, проверка курса
RUN utils.

FUNCTION getAlignmentAngle {
    LOCAL shipVec TO SHIP:FACING:FOREVECTOR.
    IF n:DELTAV:MAG < 0.01 { RETURN 0. }
    LOCAL targetVec TO n:DELTAV:NORMALIZED.
    LOCAL dot TO VDOT(shipVec, targetVec).
    SET dot TO clamp(dot, -1, 1).
    RETURN ARCCOS(dot).
}

CLEARSCREEN.
PRINT "=== Maneuver Node Execution ===".

IF ALLNODES:LENGTH = 0 {
    PRINT "No maneuver nodes found! Exiting.".
    SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
    EXIT.
}

SET n TO ALLNODES[0].

IF n:DELTAV:MAG < 0.5 {
    PRINT "Node dV negligible (" + ROUND(n:DELTAV:MAG, 2) + " m/s). Removing.".
    REMOVE n.
    EXIT.
}

PRINT "Node found. Required dV: " + ROUND(n:DELTAV:MAG, 2) + " m/s".

// Расчет времени горения на основе TWR
SET accel TO MAX(0.1, SHIP:MAXTHRUST / SHIP:MASS).
SET burnTime TO n:DELTAV:MAG / accel.
SET startBurnETA TO (burnTime / 2) + 5.

PRINT "Estimated burn time: " + ROUND(burnTime, 1) + "s".
PRINT "Starting burn at T-" + ROUND(startBurnETA, 1) + "s".

// ШАГ 1: СРАЗУ поворачиваем ракету на вектор ноды (ДО варпа!)
LOCK STEERING TO n:DELTAV.
PRINT "Aligning to burn vector...".

// Ждем выравнивания (угол < 2°) с таймаутом 120 секунд
SET alignStart TO TIME:SECONDS.
UNTIL getAlignmentAngle() < 2 OR (TIME:SECONDS - alignStart > 120) OR ALLNODES:LENGTH = 0 {
    WAIT 0.5.
}

IF ALLNODES:LENGTH = 0 {
    PRINT "Node disappeared during alignment. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

IF getAlignmentAngle() >= 2 {
    PRINT "WARNING: Align timeout (angle=" + ROUND(getAlignmentAngle(), 1) + "deg). Proceeding.".
} ELSE {
    PRINT "Aligned (angle=" + ROUND(getAlignmentAngle(), 2) + "deg).".
}

// ШАГ 2: Варпим только после того, как ракета на курсе
IF n:ETA > startBurnETA + 30 {
    PRINT "Warping to burn start...".
    SET WARP TO 3.
    WAIT UNTIL n:ETA <= startBurnETA + 30 OR ALLNODES:LENGTH = 0.
    SET WARP TO 0.
    
    IF ALLNODES:LENGTH = 0 {
        PRINT "Node disappeared during warp. Exiting.".
        UNLOCK STEERING.
        EXIT.
    }
    
    // После варпа проверяем курс (мог сбиться)
    IF getAlignmentAngle() >= 2 {
        PRINT "Re-aligning after warp (angle=" + ROUND(getAlignmentAngle(), 1) + "deg)...".
        SET alignStart TO TIME:SECONDS.
        UNTIL getAlignmentAngle() < 2 OR (TIME:SECONDS - alignStart > 60) OR ALLNODES:LENGTH = 0 {
            WAIT 0.5.
        }
    }
}

// ШАГ 3: Финальная проверка курса перед горением
IF ALLNODES:LENGTH = 0 {
    PRINT "Node disappeared. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

SET alignStart TO TIME:SECONDS.
UNTIL getAlignmentAngle() < 2 OR (TIME:SECONDS - alignStart > 30) OR ALLNODES:LENGTH = 0 {
    WAIT 0.1.
}

IF ALLNODES:LENGTH = 0 {
    PRINT "Node disappeared. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

IF getAlignmentAngle() >= 2 {
    PRINT "WARNING: Final angle=" + ROUND(getAlignmentAngle(), 1) + "deg. Proceeding.".
}

// ШАГ 4: Ждем момента начала горения
WAIT UNTIL n:ETA <= startBurnETA OR ALLNODES:LENGTH = 0.

IF ALLNODES:LENGTH = 0 {
    PRINT "Node disappeared. Exiting.".
    UNLOCK STEERING.
    EXIT.
}

PRINT "Burning...".

// ШАГ 5: Горение с динамическим дросселированием
UNTIL ALLNODES:LENGTH = 0 OR n:DELTAV:MAG <= 0.5 {
    IF n:DELTAV:MAG > 10 {
        SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 1.
    } ELSE IF n:DELTAV:MAG > 2 {
        SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.3.
    } ELSE {
        SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.1.
    }
    WAIT 0.05.
}

SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

IF ALLNODES:LENGTH > 0 {
    REMOVE n.
    PRINT "Node executed successfully!".
}

UNLOCK STEERING.
// Включаем SAS для стабилизации
SET SAS TO TRUE.
