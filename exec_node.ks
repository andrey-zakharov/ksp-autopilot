// exec_node.ks
// Выполняет следующую маневренную ноду с учетом TWR (split the burn)
// Использует ALLNODES и динамически дросселирует двигатель

CLEARSCREEN.
PRINT "=== Maneuver Node Execution ===".

IF ALLNODES:LENGTH = 0 {
    PRINT "No maneuver nodes found! Exiting.".
    SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
    EXIT.
}

SET n TO ALLNODES[0].
PRINT "Node found. Required dV: " + ROUND(n:DELTAV:MAG, 2) + " m/s".

// Расчет времени горения на основе TWR
// SHIP:MAXTHRUST в kN, SHIP:MASS в тоннах -> ускорение в м/с^2
SET accel TO MAX(0.1, SHIP:MAXTHRUST / SHIP:MASS).
SET burnTime TO n:DELTAV:MAG / accel.
// Начинаем горение за половину времени + запас на стабилизацию SAS
SET startBurnETA TO (burnTime / 2) + 5.

PRINT "Estimated burn time: " + ROUND(burnTime, 1) + "s".
PRINT "Starting burn at T-" + ROUND(startBurnETA, 1) + "s".

// Ускорение времени, если до старта горения далеко
IF n:ETA > startBurnETA + 30 {
    PRINT "Warping to burn start...".
    SET WARP TO 3.
    WAIT UNTIL n:ETA <= startBurnETA + 30 OR ALLNODES:LENGTH = 0.
    SET WARP TO 0.
}

// Поворачиваем корабль по вектору ноды
LOCK STEERING TO n:DELTAV.
WAIT 1. // Даем SAS время стабилизироваться

// Начинаем горение, когда ETA <= startBurnETA
WAIT UNTIL n:ETA <= startBurnETA OR ALLNODES:LENGTH = 0.

IF ALLNODES:LENGTH = 0 {
    PRINT "Node disappeared (maybe completed manually?). Exiting.".
    UNLOCK STEERING.
    EXIT.
}

PRINT "Burning...".

// Держим газ, пока остается требуемый dV
UNTIL ALLNODES:LENGTH = 0 OR n:DELTAV:MAG <= 0.5 {
    // Динамическое дросселирование в конце для точности
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

// Удаляем выполненную ноду
IF ALLNODES:LENGTH > 0 {
    REMOVE n.
    PRINT "Node executed successfully!".
}

UNLOCK STEERING.
LOCK STEERING TO PROGRADE.