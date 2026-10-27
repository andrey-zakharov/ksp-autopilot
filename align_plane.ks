// align_plane.ks — совмещение плоскостей. Рамка восстанавливается по элементам орбиты,
// калибруясь по достоверной нормали цели. Не использует SHIP:POSITION/VELOCITY.
PARAMETER tolerance IS 0.05.
PARAMETER steerTol IS 2.
PARAMETER maxIterations IS 3.
PARAMETER warpLevel IS 3.
PARAMETER minNodeEta IS 30.
PARAMETER clearOldNodes IS TRUE.

SET activeNode TO 0.
SET throttleCmd TO 0.

FUNCTION clamp { PARAMETER x. PARAMETER lo. PARAMETER hi.
    IF x < lo { RETURN lo. } IF x > hi { RETURN hi. } RETURN x. }
FUNCTION normDeg { PARAMETER a. LOCAL x TO a.
    UNTIL x >= 0 { SET x TO x + 360. } UNTIL x < 360 { SET x TO x - 360. } RETURN x. }
FUNCTION trueToMean { PARAMETER nuDeg. PARAMETER ecc.
    LOCAL k TO SQRT(ABS(1 - ecc) / (1 + ecc)).
    LOCAL eAn TO 2 * ARCTAN2(k * SIN(nuDeg / 2), COS(nuDeg / 2)).
    RETURN eAn - (180 / CONSTANT:PI) * ecc * SIN(eAn). }
// нормаль орбиты из наклонения и LAN в мировой рамке
FUNCTION reconNormal { PARAMETER inc. PARAMETER lan. PARAMETER ref. PARAMETER nv.
    LOCAL eqDir TO ANGLEAXIS(lan - 90, nv) * ref.
    RETURN ((COS(inc) * nv) + (SIN(inc) * eqDir)):NORMALIZED. }
FUNCTION steerErrorDeg {
    LOCAL want TO activeNode:DELTAV.
    IF want:MAG < 0.0001 { RETURN 0. }
    RETURN VANG(SHIP:FACING:FOREVECTOR, want). }

FUNCTION findNode { PARAMETER nTgt. PARAMETER pAxis. PARAMETER qAxis.
    PARAMETER nuNow. PARAMETER ecc. PARAMETER period. PARAMETER minEta.
    LOCAL A TO VDOT(pAxis, nTgt). LOCAL B TO VDOT(qAxis, nTgt).
    IF ABS(A) < 0.0000001 AND ABS(B) < 0.0000001 { RETURN List(0). }
    LOCAL nu1 TO normDeg(ARCTAN2(-A, B)). LOCAL nu2 TO normDeg(nu1 + 180).
    LOCAL mNow TO trueToMean(nuNow, ecc).
    LOCAL best TO List(). LOCAL bestEta TO -1. LOCAL anyB TO List(). LOCAL anyEta TO -1.
    LOCAL k TO 0. LOCAL nuN TO 0. LOCAL dm TO 0. LOCAL nodeEta TO 0. LOCAL item TO List().
    UNTIL k >= 2 {
        IF k = 0 { SET nuN TO nu1. } ELSE { SET nuN TO nu2. }
        SET dm TO normDeg(trueToMean(nuN, ecc) - mNow).
        SET nodeEta TO dm / 360 * period.
        SET item TO List(nodeEta, nuN).
        IF anyEta < 0 OR nodeEta < anyEta { SET anyEta TO nodeEta. SET anyB TO item. }
        IF nodeEta >= minEta AND (bestEta < 0 OR nodeEta < bestEta) { SET bestEta TO nodeEta. SET best TO item. }
        SET k TO k + 1.
    }
    IF best:LENGTH = 2 { RETURN best. }
    RETURN anyB. }

FUNCTION executeNode { PARAMETER steerTolDeg. PARAMETER warpLvl.
    SAS OFF.
    LOCK STEERING TO activeNode:DELTAV.
    LOCAL maxAcc TO MAX(0.01, SHIP:MAXTHRUST / SHIP:MASS).
    LOCAL burnTime TO activeNode:DELTAV:MAG / maxAcc.
    LOCAL leadTime TO (burnTime / 2) + 5.
    LOCAL t0 TO 0. LOCAL dv0 TO V(0,0,0). LOCAL base TO 0. LOCAL err TO 0.
    PRINT "Горение ~" + ROUND(burnTime,1) + " с, старт за T-" + ROUND(leadTime,1) + " с".
    SET t0 TO TIME:SECONDS.
    UNTIL steerErrorDeg() < steerTolDeg OR (TIME:SECONDS - t0) > 180 OR NOT HASNODE { WAIT 0.2. }
    IF NOT HASNODE { RETURN FALSE. }
    IF activeNode:ETA > leadTime + 60 {
        SET WARP TO warpLvl.
        WAIT UNTIL activeNode:ETA <= (leadTime + 60) OR NOT HASNODE.
        SET WARP TO 0. WAIT 0.5.
        IF NOT HASNODE { RETURN FALSE. }
        SET t0 TO TIME:SECONDS.
        UNTIL steerErrorDeg() < steerTolDeg OR (TIME:SECONDS - t0) > 60 OR NOT HASNODE { WAIT 0.2. }
    }
    IF NOT HASNODE { RETURN FALSE. }
    WAIT UNTIL activeNode:ETA <= leadTime OR NOT HASNODE.
    IF NOT HASNODE { RETURN FALSE. }
    SET dv0 TO activeNode:DELTAV.
    LOCK THROTTLE TO throttleCmd.
    UNTIL (NOT HASNODE) OR activeNode:DELTAV:MAG < 0.1 OR VDOT(dv0, activeNode:DELTAV) < 0 {
        SET maxAcc TO MAX(0.01, SHIP:MAXTHRUST / SHIP:MASS).
        SET err TO steerErrorDeg().
        SET base TO MIN(1, activeNode:DELTAV:MAG / maxAcc).
        IF err > 60 { SET throttleCmd TO 0. } ELSE { SET throttleCmd TO base * MAX(0.15, COS(err)). }
        WAIT 0.
    }
    SET throttleCmd TO 0. WAIT 0.2.
    UNLOCK THROTTLE.
    SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
    UNLOCK STEERING.
    RETURN TRUE. }

// ==========================================================================
CLEARSCREEN.
PRINT "=== СОВМЕЩЕНИЕ ПЛОСКОСТИ ===".
IF NOT HASTARGET { PRINT "Нет цели (клавиша Y).". EXIT. }
IF SHIP:MAXTHRUST <= 0.01 { PRINT "Нет тяги.". EXIT. }
IF SHIP:ORBIT:ECCENTRICITY >= 1 { PRINT "Орбита не замкнута.". EXIT. }
IF NOT (TARGET:ORBIT:BODY:NAME = SHIP:ORBIT:BODY:NAME) { PRINT "Разные SOI.". EXIT. }
IF clearOldNodes { UNTIL ALLNODES:LENGTH = 0 { REMOVE ALLNODES[0]. } }

// ---- калибровка опорного направления по достоверной нормали цели (один раз) ----
SET northVec TO SHIP:BODY:ANGULARVEL:NORMALIZED.
SET nTgt TO VCRS(TARGET:POSITION, TARGET:VELOCITY:ORBIT):NORMALIZED.
SET i_t TO TARGET:ORBIT:INCLINATION.
SET lan_t TO TARGET:ORBIT:LAN.
SET nTgtEq TO nTgt - (VDOT(nTgt, northVec) * northVec).
SET refDir TO ANGLEAXIS(90 - lan_t, northVec) * nTgtEq:NORMALIZED.
PRINT "Калибровка цели: " + ROUND(VANG(reconNormal(i_t, lan_t, refDir, northVec), nTgt), 3) + " deg".

SET iter TO 0. SET done TO FALSE. SET prevRelInc TO 999.

UNTIL done {
    SET i_s TO SHIP:ORBIT:INCLINATION.
    SET lan_s TO SHIP:ORBIT:LAN.
    SET nShip TO reconNormal(i_s, lan_s, refDir, northVec).
    SET iRel TO VANG(nShip, nTgt).

    CLEARSCREEN.
    PRINT "=== СОВМЕЩЕНИЕ ПЛОСКОСТИ ===".
    PRINT "Отн. наклонение: " + ROUND(iRel,4) + " deg    Итерация " + (iter+1) + "/" + maxIterations.

    IF iRel <= tolerance { PRINT "Плоскости совпадают. ГОТОВО.". SET done TO TRUE. }
    ELSE IF iRel > 179 { PRINT "Ретроградный случай — вручную.". SET done TO TRUE. }
    ELSE IF iter >= maxIterations { PRINT "Лимит итераций.". SET done TO TRUE. }
    ELSE {
        // ---- перицентральный базис корабля из элементов (мировая рамка) ----
        SET ecc TO SHIP:ORBIT:ECCENTRICITY.
        SET sma TO SHIP:ORBIT:SEMIMAJORAXIS.
        SET mu TO SHIP:ORBIT:BODY:MU.
        SET period TO SHIP:ORBIT:PERIOD.
        SET nuNow TO SHIP:ORBIT:TRUEANOMALY.
        SET argp TO SHIP:ORBIT:ARGUMENTOFPERIAPSIS.
        SET nodeDir_s TO (ANGLEAXIS(lan_s, northVec) * refDir):NORMALIZED.
        // направление движения в восходящем узле: угловой момент вдоль nShip
        SET e2 TO VCRS(nShip, nodeDir_s):NORMALIZED.
        IF VDOT(VCRS(nodeDir_s, e2), nShip) < 0 { SET e2 TO (-1) * e2. }
        SET pAxis TO ((COS(argp) * nodeDir_s) + (SIN(argp) * e2)):NORMALIZED.
        SET qAxis TO ((-SIN(argp) * nodeDir_s) + (COS(argp) * e2)):NORMALIZED.

        SET res TO findNode(nTgt, pAxis, qAxis, nuNow, ecc, period, minNodeEta).
        IF res:LENGTH < 2 { PRINT "Узел не найден.". SET done TO TRUE. }
        ELSE {
            SET nodeEta TO res[0]. SET nuNode TO res[1].
            SET dVec TO ((COS(nuNode) * pAxis) + (SIN(nuNode) * qAxis)):NORMALIZED.
            SET hMag TO SQRT(mu * sma * (1 - ecc*ecc)).
            SET vNode TO (mu / hMag) * ((-SIN(nuNode) * pAxis) + ((ecc + COS(nuNode)) * qAxis)).
            SET nodeType TO "DN". IF VDOT(vNode, nTgt) > 0 { SET nodeType TO "AN". }

            // ---- разворот плоскости вокруг линии узлов (сохраняет апоцентр) ----
            SET b1 TO VCRS(nShip, dVec):NORMALIZED.
            SET b2 TO VCRS(nTgt, dVec):NORMALIZED.
            SET vRad TO VDOT(vNode, dVec).
            SET vTan TO VDOT(vNode, b1).
            SET dvVec TO ((vRad * dVec) + (vTan * b2)) - vNode.
            PRINT "Узел " + nodeType + " через " + ROUND(nodeEta,1) + " с, dV=" + ROUND(dvVec:MAG,2) + " м/с".

            // ---- нода + калибровка её осей ----
            SET activeNode TO NODE(TimeSpan(nodeEta), 0, 1, 0).
            ADD activeNode. WAIT 0.2.
            SET activeNode:PROGRADE TO 0. SET activeNode:RADIALOUT TO 0.
            SET activeNode:NORMAL TO 1. WAIT 0.1.
            SET normAxis TO activeNode:DELTAV:NORMALIZED.
            SET activeNode:NORMAL TO 0. SET activeNode:PROGRADE TO 1. WAIT 0.1.
            SET proAxis TO activeNode:DELTAV:NORMALIZED.
            SET activeNode:PROGRADE TO 0. SET activeNode:RADIALOUT TO 1. WAIT 0.1.
            SET radAxis TO activeNode:DELTAV:NORMALIZED.
            SET activeNode:PROGRADE TO VDOT(dvVec, proAxis).
            SET activeNode:NORMAL TO VDOT(dvVec, normAxis).
            SET activeNode:RADIALOUT TO VDOT(dvVec, radAxis).
            WAIT 0.2.

            SET rbAng TO VANG(dvVec, activeNode:DELTAV).
            PRINT "Контроль ноды: отклонение " + ROUND(rbAng,2) + " deg".
            // ---- ПРОГНОЗ направления разворота и автокоррекция ----
            SET nodeUT TO TIME:SECONDS + activeNode:ETA.
            SET vAfter TO VELOCITYAT(SHIP, nodeUT):ORBIT.          // скорость ПОСЛЕ манёвра (нода учитывается)
            SET nAfter TO VCRS(dVec, vAfter):NORMALIZED.           // нормаль орбиты после манёвра
            SET iRelAfter TO VANG(nAfter, nTgt).
            PRINT "Прогноз: отн. наклонение станет " + ROUND(iRelAfter, 3) + " deg (сейчас " + ROUND(iRel, 3) + ")".

            IF iRelAfter > iRel {
                PRINT "Разворот идёт в зеркальную плоскость — переворачиваю нормаль".
                SET activeNode:NORMAL TO -activeNode:NORMAL.
                WAIT 0.2.
                // контрольная перепроверка
                SET vAfter2 TO VELOCITYAT(SHIP, TIME:SECONDS + activeNode:ETA):ORBIT.
                SET nAfter2 TO VCRS(dVec, vAfter2):NORMALIZED.
                PRINT "После переворота: отн. наклонение станет " + ROUND(VANG(nAfter2, nTgt), 3) + " deg".
            }
            IF rbAng > 5 { PRINT "Нода легла криво — отмена.". REMOVE activeNode. SET done TO TRUE. }
            ELSE {
                SET ok TO executeNode(steerTol, warpLevel).
                UNTIL ALLNODES:LENGTH = 0 { REMOVE ALLNODES[0]. }
                SET prevRelInc TO iRel.
                SET iter TO iter + 1.
                IF NOT ok { PRINT "Манёвр прерван.". SET done TO TRUE. } ELSE { WAIT 2. }
            }
        }
    }
}

// ---- итог ----
SET nShipF TO reconNormal(SHIP:ORBIT:INCLINATION, SHIP:ORBIT:LAN, refDir, northVec).
SET iRelF TO VANG(nShipF, nTgt).
CLEARSCREEN.
PRINT "=== РЕЗУЛЬТАТ ===".
PRINT "Было: " + ROUND(prevRelInc,4) + " deg   Стало: " + ROUND(iRelF,4) + " deg".
PRINT "Апоцентр: " + ROUND(SHIP:ORBIT:APOAPSIS) + " м".
IF iRelF <= tolerance { PRINT "ПЛОСКОСТИ СОВМЕЩЕНЫ". }
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0. UNLOCK THROTTLE. UNLOCK STEERING.