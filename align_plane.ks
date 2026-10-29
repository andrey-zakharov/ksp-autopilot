@LAZYGLOBAL OFF.

// ==========================================================================
// align_plane.ks — совмещение плоскостей орбит без изменения Ap/Pe
// Использует exec_node.ks
//
// Главное отличие от предыдущей версии:
// прогноз делается аналитически от скорости ДО ноды + DELTAV ноды,
// а не через POSITIONAT/VELOCITYAT точно в момент ноды.
// ==========================================================================

PARAMETER tolDeg IS 0.05.
PARAMETER maxIters IS 3.
PARAMETER minTimeSec IS 30.
PARAMETER clearOld IS TRUE.
PARAMETER coarseSamples IS 96.
PARAMETER bisectIters IS 24.
PARAMETER minBurnDv IS 0.5.

// Допуски качества ноды
PARAMETER maxNodeMisalignDeg IS 8.
PARAMETER maxSpeedErr IS 2.0.
PARAMETER maxApPeDrift IS 10000.

RUN utils.

// ==========================================================================
// Вспомогательные функции
// ==========================================================================

FUNCTION sameSign { PARAMETER valA. PARAMETER valB.
    IF valA >= 0 AND valB >= 0 { RETURN TRUE. }
    IF valA <= 0 AND valB <= 0 { RETURN TRUE. }
    RETURN FALSE.
}

FUNCTION orbitNormal { PARAMETER posArg. PARAMETER velArg.
    LOCAL hVec IS VCRS(posArg, velArg).

    IF hVec:MAG < 0.000001 {
        RETURN V(0, 1, 0).
    }

    RETURN hVec:NORMALIZED.
}

// ==========================================================================
// Поиск времени до узла пересечения плоскостей
// ==========================================================================

FUNCTION findNodeTime { PARAMETER planeNrm. PARAMETER minT. PARAMETER periodSec. PARAMETER samples. PARAMETER bisections.

    LOCAL curUT IS TIME:SECONDS.
    LOCAL stepSec IS MAX(1, periodSec / samples).

    LOCAL tA IS minT.
    LOCAL posA IS POSITIONAT(SHIP, curUT + tA).
    LOCAL projA IS VDOT(posA, planeNrm).

    LOCAL bestT IS tA.
    LOCAL bestAbs IS ABS(projA).

    LOCAL tB IS tA + stepSec.
    LOCAL limitT IS minT + periodSec + stepSec.

    LOCAL posB IS V(0, 0, 0).
    LOCAL projB IS 0.

    LOCAL lowT IS 0.
    LOCAL highT IS 0.
    LOCAL lowProj IS 0.
    LOCAL midT IS 0.
    LOCAL posM IS V(0, 0, 0).
    LOCAL projM IS 0.
    LOCAL bisectIdx IS 0.

    IF projA = 0 {
        RETURN tA.
    }

    UNTIL tB > limitT {

        SET posB TO POSITIONAT(SHIP, curUT + tB).
        SET projB TO VDOT(posB, planeNrm).

        IF ABS(projB) < bestAbs {
            SET bestAbs TO ABS(projB).
            SET bestT TO tB.
        }

        IF projB = 0 {
            RETURN tB.
        }

        IF NOT sameSign(projA, projB) {

            SET lowT TO tB - stepSec.
            SET highT TO tB.
            SET lowProj TO projA.
            SET bisectIdx TO 0.

            UNTIL bisectIdx >= bisections OR (highT - lowT) < 0.25 {

                SET midT TO (lowT + highT) / 2.
                SET posM TO POSITIONAT(SHIP, curUT + midT).
                SET projM TO VDOT(posM, planeNrm).

                IF projM = 0 {
                    SET lowT TO midT.
                    SET highT TO midT.
                } ELSE {
                    IF NOT sameSign(lowProj, projM) {
                        SET highT TO midT.
                    } ELSE {
                        SET lowT TO midT.
                        SET lowProj TO projM.
                    }
                }

                SET bisectIdx TO bisectIdx + 1.
            }

            RETURN (lowT + highT) / 2.
        }

        SET tA TO tB.
        SET projA TO projB.
        SET tB TO tB + stepSec.
    }

    RETURN bestT.
}

// ==========================================================================
// Кандидат на dV: поворот скорости вокруг радиус-вектора на +/- угол.
// Возвращает List(dvVec, errDeg).
// ==========================================================================

FUNCTION rotatedPlaneDv { PARAMETER rVec. PARAMETER vVec. PARAMETER tgtNrm. PARAMETER dirSign.

    LOCAL rHat IS rVec:NORMALIZED.
    LOCAL hVec IS orbitNormal(rVec, vVec).
    LOCAL angDeg IS VANG(hVec, tgtNrm).

    LOCAL vRot IS ANGLEAXIS(dirSign * angDeg, rHat) * vVec.
    LOCAL dvVec IS vRot - vVec.

    LOCAL hRot IS orbitNormal(rVec, vRot).
    LOCAL errDeg IS VANG(hRot, tgtNrm).

    RETURN List(dvVec, errDeg).
}

// ==========================================================================
// Локальное состояние
// ==========================================================================

LOCAL fatalFlag IS FALSE.
LOCAL doneFlag IS FALSE.
LOCAL iterCount IS 0.

LOCAL targetNrm IS V(0, 0, 0).
LOCAL shipNrm IS V(0, 0, 0).
LOCAL relInc IS 0.

LOCAL startAp IS 0.
LOCAL startPe IS 0.
LOCAL endAp IS 0.
LOCAL endPe IS 0.
LOCAL finalInc IS 0.

LOCAL nodeTimeSec IS 0.
LOCAL orbPeriod IS 0.

LOCAL rPre IS V(0, 0, 0).
LOCAL vPre IS V(0, 0, 0).
LOCAL hPre IS V(0, 0, 0).
LOCAL tNrmLocal IS V(0, 0, 0).
LOCAL alphaDeg IS 0.
LOCAL refSpeed IS 0.

LOCAL planNode IS 0.

LOCAL normAxis IS V(0, 0, 0).
LOCAL proAxis IS V(0, 0, 0).
LOCAL radAxis IS V(0, 0, 0).

LOCAL candInfo IS List(V(0, 0, 0), 0).
LOCAL candDv IS V(0, 0, 0).
LOCAL candErr IS 0.

LOCAL tmpPro IS 0.
LOCAL tmpNorm IS 0.
LOCAL tmpRad IS 0.
LOCAL tmpNodeDv IS V(0, 0, 0).
LOCAL tmpMis IS 0.
LOCAL predV IS V(0, 0, 0).
LOCAL predH IS V(0, 0, 0).
LOCAL tmpPredErr IS 0.
LOCAL predSpeed IS 0.
LOCAL speedErr IS 0.

LOCAL bestErrDeg IS 999.
LOCAL bestPrograde IS 0.
LOCAL bestNormal IS 0.
LOCAL bestMisalign IS 999.
LOCAL bestSpeedErr IS 999.
LOCAL hasBest IS FALSE.

LOCAL signIdx IS 0.
LOCAL signDir IS 0.

LOCAL nodeDv IS V(0, 0, 0).
LOCAL dvMag IS 0.

LOCAL postOrb IS 0.
LOCAL postAp IS 0.
LOCAL postPe IS 0.
LOCAL preAp IS 0.
LOCAL prePe IS 0.
LOCAL driftApPlan IS 0.
LOCAL driftPePlan IS 0.

// ==========================================================================
// Старт
// ==========================================================================

CLEARSCREEN.

PRINT "=== PLANE ALIGN ===".
PRINT "analytic forecast, Ap/Pe preserve".
PRINT "".

IF NOT HASTARGET {
    PRINT "NO TARGET".
    SET fatalFlag TO TRUE.
}

IF NOT fatalFlag {
    IF SHIP:MAXTHRUST <= 0.01 {
        PRINT "NO THRUST".
        SET fatalFlag TO TRUE.
    }
}

IF NOT fatalFlag {
    IF SHIP:ORBIT:ECCENTRICITY >= 1 {
        PRINT "OPEN ORBIT".
        SET fatalFlag TO TRUE.
    }
}

IF NOT fatalFlag {
    IF TARGET:ORBIT:BODY:NAME <> SHIP:ORBIT:BODY:NAME {
        PRINT "DIFF SOI".
        SET fatalFlag TO TRUE.
    }
}

IF NOT fatalFlag {
    SET targetNrm TO orbitNormal(TARGET:POSITION, TARGET:VELOCITY:ORBIT).
    SET startAp TO SHIP:ORBIT:APOAPSIS.
    SET startPe TO SHIP:ORBIT:PERIAPSIS.

    PRINT "Ap0/Pe0: " + ROUND(startAp) + " / " + ROUND(startPe) + " m".
    PRINT "".
}

UNTIL doneFlag OR fatalFlag {

    SET shipNrm TO orbitNormal(SHIP:POSITION, SHIP:VELOCITY:ORBIT).

    // Нормали плоскости можно брать с любым знаком.
    // Ориентируем нормаль цели так, чтобы угол был <= 90 deg.
    IF VANG(shipNrm, targetNrm) > 90 {
        SET targetNrm TO (-1) * targetNrm.
    }

    SET relInc TO VANG(shipNrm, targetNrm).

    PRINT "IT " + (iterCount + 1) + "/" + maxIters +
          " inc=" + ROUND(relInc, 4) +
          " Ap/Pe=" + ROUND(SHIP:ORBIT:APOAPSIS) +
          "/" + ROUND(SHIP:ORBIT:PERIAPSIS).

    IF relInc <= tolDeg {
        PRINT "ALIGNED".
        SET doneFlag TO TRUE.
    }

    IF NOT doneFlag {
        IF iterCount >= maxIters {
            PRINT "LIMIT".
            SET doneFlag TO TRUE.
        }
    }

    // ----------------------------------------------------------------------
    // Поиск времени узла
    // ----------------------------------------------------------------------

    IF NOT doneFlag {

        IF clearOld {
            UNTIL NOT HASNODE {
                REMOVE NEXTNODE.
            }
            WAIT 0.
        }

        SET orbPeriod TO MAX(60, SHIP:ORBIT:PERIOD).
        SET nodeTimeSec TO findNodeTime(targetNrm, minTimeSec, orbPeriod, coarseSamples, bisectIters).

        // NaN не равен сам себе
        IF nodeTimeSec <> nodeTimeSec {
            PRINT "BAD TIME".
            SET doneFlag TO TRUE.
        } ELSE {
            PRINT "node time=" + ROUND(nodeTimeSec, 1) + " s".
        }
    }

    // ----------------------------------------------------------------------
    // Состояние ДО ноды в момент узла
    // Это ключевое: берем pre-burn r/v, а не post-burn из POSITIONAT/VELOCITYAT.
    // ----------------------------------------------------------------------

    IF NOT doneFlag {

        SET rPre TO POSITIONAT(SHIP, TIME:SECONDS + nodeTimeSec).
        SET vPre TO VELOCITYAT(SHIP, TIME:SECONDS + nodeTimeSec):ORBIT.

        IF rPre:MAG < 1 OR vPre:MAG < 0.01 {
            PRINT "BAD PRE-NODE STATE".
            SET doneFlag TO TRUE.
        } ELSE {

            SET hPre TO orbitNormal(rPre, vPre).
            SET tNrmLocal TO targetNrm.

            IF VANG(hPre, tNrmLocal) > 90 {
                SET tNrmLocal TO (-1) * tNrmLocal.
            }

            SET alphaDeg TO VANG(hPre, tNrmLocal).
            SET refSpeed TO vPre:MAG.

            PRINT "pre-node alpha=" + ROUND(alphaDeg, 4) +
                  " deg, v=" + ROUND(refSpeed, 1) + " m/s".

            IF alphaDeg <= tolDeg {
                PRINT "ALIGNED AT NODE".
                SET doneFlag TO TRUE.
            }
        }
    }

    // ----------------------------------------------------------------------
    // Создаем ноду и калибруем её оси
    // ----------------------------------------------------------------------

    IF NOT doneFlag {

        SET planNode TO NODE(TimeSpan(nodeTimeSec), 0, 0, 0).
        ADD planNode.
        WAIT 0.5.

        SET planNode:RADIALOUT TO 0.
        SET planNode:NORMAL TO 1.
        SET planNode:PROGRADE TO 0.
        WAIT 0.1.
        SET normAxis TO planNode:DELTAV:NORMALIZED.

        SET planNode:NORMAL TO 0.
        SET planNode:PROGRADE TO 1.
        SET planNode:RADIALOUT TO 0.
        WAIT 0.1.
        SET proAxis TO planNode:DELTAV:NORMALIZED.

        SET planNode:PROGRADE TO 0.
        SET planNode:RADIALOUT TO 1.
        SET planNode:NORMAL TO 0.
        WAIT 0.1.
        SET radAxis TO planNode:DELTAV:NORMALIZED.

        IF normAxis:MAG < 0.000001 OR proAxis:MAG < 0.000001 OR radAxis:MAG < 0.000001 {
            PRINT "BAD NODE AXES".
            REMOVE planNode.
            SET doneFlag TO TRUE.
        }
    }

    // ----------------------------------------------------------------------
    // Перебираем два знака поворота и выбираем лучший по аналитическому прогнозу
    // ----------------------------------------------------------------------

    IF NOT doneFlag {

        SET bestErrDeg TO 999.
        SET bestPrograde TO 0.
        SET bestNormal TO 0.
        SET bestMisalign TO 999.
        SET bestSpeedErr TO 999.
        SET hasBest TO FALSE.

        SET signIdx TO 0.

        UNTIL signIdx >= 2 {

            IF signIdx = 0 {
                SET signDir TO 1.
            } ELSE {
                SET signDir TO -1.
            }

            SET candInfo TO rotatedPlaneDv(rPre, vPre, tNrmLocal, signDir).
            SET candDv TO candInfo[0].
            SET candErr TO candInfo[1].

            SET tmpPro TO VDOT(candDv, proAxis).
            SET tmpNorm TO VDOT(candDv, normAxis).
            SET tmpRad TO VDOT(candDv, radAxis).

            // Радиальную составляющую обнуляем, чтобы не менять Ap/Pe.
            SET planNode:RADIALOUT TO 0.
            SET planNode:NORMAL TO tmpNorm.
            SET planNode:PROGRADE TO tmpPro.
            WAIT 0.1.

            SET tmpNodeDv TO planNode:DELTAV.

            IF tmpNodeDv:MAG < 0.0001 {
                SET tmpMis TO 999.
            } ELSE {
                SET tmpMis TO VANG(candDv, tmpNodeDv).
            }

            // Аналитический прогноз: скорость ДО ноды + фактический DELTAV ноды.
            SET predV TO vPre + tmpNodeDv.
            SET predH TO orbitNormal(rPre, predV).

            IF predH:MAG < 0.000001 {
                SET tmpPredErr TO 999.
            } ELSE {
                SET tmpPredErr TO VANG(predH, tNrmLocal).
            }

            SET predSpeed TO predV:MAG.
            SET speedErr TO ABS(predSpeed - refSpeed).

            PRINT "cand sign=" + signDir +
                  " dVn=" + ROUND(tmpNorm, 3) +
                  " dVp=" + ROUND(tmpPro, 3) +
                  " dVr=" + ROUND(tmpRad, 3) +
                  " mis=" + ROUND(tmpMis, 2) +
                  " predErr=" + ROUND(tmpPredErr, 4) +
                  " spdErr=" + ROUND(speedErr, 3).

            IF tmpMis <= maxNodeMisalignDeg AND
               speedErr <= maxSpeedErr AND
               tmpPredErr < bestErrDeg {

                SET bestErrDeg TO tmpPredErr.
                SET bestPrograde TO tmpPro.
                SET bestNormal TO tmpNorm.
                SET bestMisalign TO tmpMis.
                SET bestSpeedErr TO speedErr.
                SET hasBest TO TRUE.
            }

            SET signIdx TO signIdx + 1.
        }

        IF NOT hasBest {
            PRINT "NO GOOD CANDIDATE".
            REMOVE planNode.
            SET doneFlag TO TRUE.
        } ELSE IF bestErrDeg >= alphaDeg {
            PRINT "BEST CANDIDATE DOES NOT IMPROVE: best=" + ROUND(bestErrDeg, 4) +
                  " alpha=" + ROUND(alphaDeg, 4).
            REMOVE planNode.
            SET doneFlag TO TRUE.
        }
    }

    // ----------------------------------------------------------------------
    // Выставляем лучшую ноду и проверяем итоговый прогноз
    // ----------------------------------------------------------------------

    IF NOT doneFlag {

        SET planNode:RADIALOUT TO 0.
        SET planNode:NORMAL TO bestNormal.
        SET planNode:PROGRADE TO bestPrograde.
        WAIT 0.2.

        SET nodeDv TO planNode:DELTAV.
        SET dvMag TO nodeDv:MAG.

        PRINT "FINAL node: time=" + ROUND(nodeTimeSec, 1) +
              " dVn=" + ROUND(bestNormal, 3) +
              " dVp=" + ROUND(bestPrograde, 3) +
              " |dV|=" + ROUND(dvMag, 3) +
              " mis=" + ROUND(bestMisalign, 2) +
              " predErr=" + ROUND(bestErrDeg, 4) +
              " spdErr=" + ROUND(bestSpeedErr, 3).

        IF dvMag < minBurnDv {
            PRINT "SMALL DV".
            REMOVE planNode.
            SET doneFlag TO TRUE.
        }
    }

    // ----------------------------------------------------------------------
    // Проверка Ap/Pe через встроенный прогноз ноды
    // ----------------------------------------------------------------------

    IF NOT doneFlag {

        SET postOrb TO planNode:ORBIT.
        SET postAp TO postOrb:APOAPSIS.
        SET postPe TO postOrb:PERIAPSIS.

        SET preAp TO SHIP:ORBIT:APOAPSIS.
        SET prePe TO SHIP:ORBIT:PERIAPSIS.

        SET driftApPlan TO postAp - preAp.
        SET driftPePlan TO postPe - prePe.

        PRINT "plan Ap/Pe drift: " + ROUND(driftApPlan) + " / " + ROUND(driftPePlan) + " m".

        IF ABS(driftApPlan) > maxApPeDrift OR ABS(driftPePlan) > maxApPeDrift {
            PRINT "ABORT: too large Ap/Pe drift".
            REMOVE planNode.
            SET doneFlag TO TRUE.
        }
    }

    // ----------------------------------------------------------------------
    // Выполнение через exec_node.ks
    // ----------------------------------------------------------------------

    IF NOT doneFlag {

        UNLOCK THROTTLE.
        UNLOCK STEERING.
        SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

        PRINT "RUN exec_node".
        RUN exec_node.

        SET SAS TO FALSE.

        // На случай, если exec_node что-то не добрал.
        UNTIL NOT HASNODE {
            REMOVE NEXTNODE.
        }

        SET iterCount TO iterCount + 1.
        WAIT 1.
    }

    PRINT "".
}

// ==========================================================================
// Итог
// ==========================================================================

IF startAp > 0 AND NOT fatalFlag {

    SET shipNrm TO orbitNormal(SHIP:POSITION, SHIP:VELOCITY:ORBIT).

    IF VANG(shipNrm, targetNrm) > 90 {
        SET targetNrm TO (-1) * targetNrm.
    }

    SET finalInc TO VANG(shipNrm, targetNrm).

    SET endAp TO SHIP:ORBIT:APOAPSIS.
    SET endPe TO SHIP:ORBIT:PERIAPSIS.

    LOCAL driftAp IS endAp - startAp.
    LOCAL driftPe IS endPe - startPe.

    PRINT "=== RESULT ===".
    PRINT "inc=" + ROUND(finalInc, 4).
    PRINT "Ap " + ROUND(startAp) + " -> " + ROUND(endAp) +
          " drift=" + ROUND(driftAp).
    PRINT "Pe " + ROUND(startPe) + " -> " + ROUND(endPe) +
          " drift=" + ROUND(driftPe).

    IF ABS(driftAp) > 1000 OR ABS(driftPe) > 1000 {
        PRINT "WARN DRIFT".
    }

    IF finalInc <= tolDeg {
        PRINT "OK".
    } ELSE {
        PRINT "NOT FULL".
    }

} ELSE {
    PRINT "NOT RUN".
}

SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.