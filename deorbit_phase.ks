// ==========================================================================
// deorbit_phase.ks
// Фаза 1: смена наклонения, чтобы орбита пролегала над целью
// Фаза 2: гашение горизонтальной скорости на орбите над целью
// Читает глобальные переменные из land.ks: siteLat, siteLng, siteTerrain
// ==========================================================================

PARAMETER horizStartAlt IS 10000.
PARAMETER targetTolM IS 200.
PARAMETER coarseDiv IS 96.
PARAMETER fineDiv IS 8.
PARAMETER searchWindows IS 1.
PARAMETER minBurnDv IS 1.0.
PARAMETER maxBurnDv IS 3000.
PARAMETER doCorrection IS TRUE.

// Агрессивная коррекция
PARAMETER corrMaxAcc IS 6.0.        // м/с^2, раньше было 2.0
PARAMETER corrMaxVel IS 120.0.      // м/с, раньше было 30
PARAMETER corrExitDist IS 250.0.    // м, раньше было 50
PARAMETER corrMinAlt IS 6000.0.     // м, раньше было 2000
PARAMETER corrExitVel IS 8.0.       // м/с, порог остаточной горизонтали
PARAMETER corrMaxTiltDeg IS 45.0.   // градусы, раньше было жёстко 25
PARAMETER corrMaxThrottle IS 0.90.  // раньше было жёстко 0.5
PARAMETER corrKp IS 0.8.            // коэффициент реакции на ошибку скорости
PARAMETER corrSafeVVert IS -45.0.   // безопасная вертикальная скорость, м/с

PARAMETER execLeadSec IS 5.
PARAMETER depNodeBias IS 0.
PARAMETER depLeadIter IS 2.

RUN utils.

SET depMinDv TO minBurnDv.
SET depMaxDv TO maxBurnDv.
SET depBestDvVec TO V(0, 0, 0).

// ==========================================================================
// Геометрия цели
// ==========================================================================

FUNCTION depTargetVec { PARAMETER latDeg. PARAMETER lngDeg. PARAMETER terrM. PARAMETER deltaSec.
    LOCAL geoPoint TO BODY:GEOPOSITIONLATLNG(latDeg, lngDeg).
    LOCAL baseVec TO geoPoint:POSITION - BODY:POSITION.
    IF NOT vecIsOk(baseVec) OR baseVec:MAG < 1 { RETURN V(0, 0, 0). }
    LOCAL dirVec TO baseVec:NORMALIZED.
    LOCAL angVelVec TO BODY:ANGULARVEL.
    IF vecIsOk(angVelVec) AND angVelVec:MAG > 0.00000001 {
        LOCAL rotAngleDeg TO angVelVec:MAG * deltaSec * (180 / CONSTANT:PI).
        SET rotAngleDeg TO normDeg360(rotAngleDeg).
        SET dirVec TO ANGLEAXIS(rotAngleDeg, angVelVec:NORMALIZED) * dirVec.
    }
    RETURN (BODY:RADIUS + terrM) * dirVec.
}

// ==========================================================================
// Нормаль целевой плоскости (для фазы 1)
// ==========================================================================

FUNCTION depTargetNormal { PARAMETER latDeg. PARAMETER lngDeg. PARAMETER terrM.
    LOCAL tgtVec TO depTargetVec(latDeg, lngDeg, terrM, 0).
    IF NOT vecIsOk(tgtVec) OR tgtVec:MAG < 1 { RETURN V(0, 0, 0). }
    LOCAL tgtHat TO tgtVec:NORMALIZED.
    LOCAL spinAxis TO BODY:ANGULARVEL.
    IF NOT vecIsOk(spinAxis) OR spinAxis:MAG < 0.00000001 { RETURN V(0, 1, 0). }
    SET spinAxis TO spinAxis:NORMALIZED.
    LOCAL spinProj TO spinAxis - (VDOT(spinAxis, tgtHat) * tgtHat).
    IF NOT vecIsOk(spinProj) OR spinProj:MAG < 0.000001 { RETURN V(0, 0, 0). }
    RETURN spinProj:NORMALIZED.
}

// ==========================================================================
// ФАЗА 1: смена наклонения (без изменений)
// ==========================================================================

FUNCTION depInclinationDv { PARAMETER tSec. PARAMETER tgtNormal.
    LOCAL curUT TO TIME:SECONDS.
    LOCAL velAtT TO VELOCITYAT(SHIP, TIMESTAMP(curUT + tSec)):ORBIT.
    LOCAL posAtT TO bodyPosAt(tSec).
    IF NOT vecIsOk(posAtT) OR NOT vecIsOk(velAtT) { RETURN List(FALSE, V(0,0,0), 0). }
    IF posAtT:MAG < 1 OR velAtT:MAG < 1 { RETURN List(FALSE, V(0,0,0), 0). }

    LOCAL radialHat TO posAtT:NORMALIZED.
    LOCAL vMag TO velAtT:MAG.
    LOCAL oldHVec TO VCRS(posAtT, velAtT).
    LOCAL oldNormal TO V(0, 1, 0).
    IF vecIsOk(oldHVec) AND oldHVec:MAG > 0.000001 { SET oldNormal TO oldHVec:NORMALIZED. }
    LOCAL tiltAngle TO VANG(oldNormal, tgtNormal).

    LOCAL vRad TO VDOT(velAtT, radialHat).
    LOCAL vTanSq TO vMag * vMag - vRad * vRad.
    IF vTanSq < 0 { SET vTanSq TO 0. }
    LOCAL vTan TO SQRT(vTanSq).
    LOCAL tgtDir TO VCRS(tgtNormal, radialHat).
    LOCAL velNew TO V(0, 0, 0).
    LOCAL methodUsed TO 0.

    IF vecIsOk(tgtDir) AND tgtDir:MAG > 0.000001 {
        SET tgtDir TO tgtDir:NORMALIZED.
        LOCAL curTanDir TO velAtT - vRad * radialHat.
        IF vecIsOk(curTanDir) AND curTanDir:MAG > 0.000001 {
            SET curTanDir TO curTanDir:NORMALIZED.
            IF VDOT(tgtDir, curTanDir) < 0 { SET tgtDir TO (-1) * tgtDir. }
        }
        SET velNew TO vRad * radialHat + vTan * tgtDir.
        SET methodUsed TO 1.
    }

    LOCAL checkAng TO 999.
    IF methodUsed = 1 AND vecIsOk(velNew) {
        LOCAL newHVec TO VCRS(posAtT, velNew).
        IF vecIsOk(newHVec) AND newHVec:MAG > 0.000001 {
            SET checkAng TO VANG(newHVec:NORMALIZED, tgtNormal).
        }
    }

    IF checkAng > 1.0 {
        LOCAL crossLine TO VCRS(oldNormal, tgtNormal).
        IF vecIsOk(crossLine) AND crossLine:MAG > 0.000001 {
            SET crossLine TO crossLine:NORMALIZED.
            LOCAL velRotPlus TO ANGLEAXIS(tiltAngle, crossLine) * velAtT.
            LOCAL velRotMinus TO ANGLEAXIS((-1) * tiltAngle, crossLine) * velAtT.
            LOCAL hPlus TO VCRS(posAtT, velRotPlus).
            LOCAL errPlus TO 999.
            IF vecIsOk(hPlus) AND hPlus:MAG > 0.000001 { SET errPlus TO VANG(hPlus:NORMALIZED, tgtNormal). }
            LOCAL hMinus TO VCRS(posAtT, velRotMinus).
            LOCAL errMinus TO 999.
            IF vecIsOk(hMinus) AND hMinus:MAG > 0.000001 { SET errMinus TO VANG(hMinus:NORMALIZED, tgtNormal). }
            IF errPlus < errMinus {
                SET velNew TO velRotPlus. SET checkAng TO errPlus.
            } ELSE {
                SET velNew TO velRotMinus. SET checkAng TO errMinus.
            }
            SET methodUsed TO 2.
        }
    }

    PRINT "  depInclinationDv: угол=" + ROUND(tiltAngle, 3) + " checkAng=" + ROUND(checkAng, 4) + " method=" + methodUsed.
    IF NOT vecIsOk(velNew) OR velNew:MAG < 1 { RETURN List(FALSE, V(0,0,0), 0). }

    LOCAL dvResult TO velNew - velAtT.
    IF NOT vecIsOk(dvResult) { RETURN List(FALSE, V(0,0,0), 0). }
    PRINT "  dV=" + ROUND(dvResult:MAG, 2) + " м/с".
    RETURN List(TRUE, dvResult, tiltAngle).
}

FUNCTION depPhaseOne {
    LOCAL needInc TO ABS(siteLat).
    LOCAL curInc TO SHIP:ORBIT:INCLINATION.
    PRINT "Фаза 1: нужно наклонение " + ROUND(needInc, 2) + " (сейчас " + ROUND(curInc, 2) + ").".
    IF curInc >= needInc - 0.15 {
        PRINT "Фаза 1: наклонение уже подходит, пропуск.".
        RETURN TRUE.
    }

    LOCAL tgtNormal TO depTargetNormal(siteLat, siteLng, siteTerrain).
    IF NOT vecIsOk(tgtNormal) OR tgtNormal:MAG < 0.000001 { RETURN FALSE. }

    LOCAL depMass TO MAX(0.001, SHIP:MASS).
    LOCAL depPeriod TO MAX(60, SHIP:ORBIT:PERIOD).
    LOCAL minTime TO MAX(30, estimateTurnTime(90, depMass) + 10).
    LOCAL crossTime TO findPlaneCrossTime(tgtNormal, minTime, depPeriod).
    IF NOT isFiniteVal(crossTime) OR crossTime < 5 { RETURN FALSE. }

    LOCAL incResult TO depInclinationDv(crossTime, tgtNormal).
    IF NOT incResult[0] { RETURN FALSE. }
    LOCAL incDvVec TO incResult[1].
    LOCAL incDvMag TO incDvVec:MAG.
    PRINT "Фаза 1: узел через " + ROUND(crossTime, 1) + " с, dV=" + ROUND(incDvMag, 1) + " м/с".
    IF incDvMag < depMinDv { RETURN TRUE. }
    IF incDvMag > depMaxDv { RETURN TRUE. }

    LOCAL incVel TO VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + crossTime)):ORBIT.
    LOCAL incPos TO bodyPosAt(crossTime).
    IF NOT vecIsOk(incPos) OR NOT vecIsOk(incVel) { RETURN FALSE. }

    LOCAL predictedVel TO incVel + incDvVec.
    LOCAL predictedNormal TO VCRS(incPos, predictedVel).
    IF NOT vecIsOk(predictedNormal) OR predictedNormal:MAG < 0.000001 { RETURN FALSE. }
    SET predictedNormal TO predictedNormal:NORMALIZED.
    LOCAL predictedErr TO VANG(predictedNormal, tgtNormal).
    PRINT "Фаза 1: прогноз отклонения=" + ROUND(predictedErr, 4) + " deg".
    IF predictedErr > 1.0 { RETURN FALSE. }

    SET depBestDvVec TO incDvVec.
    LOCAL incNode TO NODE(TimeSpan(crossTime), 0, 1, 0).
    ADD incNode. WAIT 0.3.
    IF NOT HASNODE { RETURN FALSE. }

    LOCAL axesList TO calibrateNodeAxes().
    LOCAL realRadial TO axesList[0].
    LOCAL realNormal TO axesList[1].
    LOCAL realPrograde TO axesList[2].
    IF NOT vecIsOk(realNormal) OR realNormal:MAG < 0.5 { REMOVE NEXTNODE. RETURN FALSE. }

    SET NEXTNODE:RADIALOUT TO VDOT(incDvVec, realRadial).
    SET NEXTNODE:NORMAL TO VDOT(incDvVec, realNormal).
    SET NEXTNODE:PROGRADE TO VDOT(incDvVec, realPrograde).
    WAIT 0.2.

    LOCAL calibErr TO VANG(NEXTNODE:DELTAV, incDvVec).
    PRINT "Фаза 1: отклонение ноды=" + ROUND(calibErr, 3) + " deg".
    IF calibErr > 5.0 { REMOVE NEXTNODE. RETURN FALSE. }

    PRINT "Фаза 1: выполняю манёвр.".
    UNLOCK THROTTLE. UNLOCK STEERING.
    RUN exec_node.
    SET SAS TO FALSE.
    UNTIL NOT HASNODE { REMOVE NEXTNODE. }
    WAIT 1.
    PRINT "Фаза 1: завершена. Новое inc=" + ROUND(SHIP:ORBIT:INCLINATION, 3) + ".".
    RETURN TRUE.
}

// Знаковый угол от корабля к цели вдоль орбиты.
// Положительный = цель впереди по движению.
// Отрицательный = цель позади.
FUNCTION depSignedSepAt { PARAMETER tSec.
    LOCAL depPos TO bodyPosAt(tSec).
    LOCAL depVel TO VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + tSec)):ORBIT.

    IF NOT vecIsOk(depPos) OR NOT vecIsOk(depVel) {
        RETURN 999.
    }
    IF depPos:MAG < 1 OR depVel:MAG < 0.01 {
        RETURN 999.
    }

    LOCAL depHVec TO VCRS(depPos, depVel).
    IF NOT vecIsOk(depHVec) OR depHVec:MAG < 0.000001 {
        RETURN 999.
    }
    SET depHVec TO depHVec:NORMALIZED.

    LOCAL depTgtVec TO depTargetVec(siteLat, siteLng, siteTerrain, tSec).
    IF NOT vecIsOk(depTgtVec) OR depTgtVec:MAG < 1 {
        RETURN 999.
    }

    LOCAL depSHat TO depPos:NORMALIZED.
    LOCAL depTHat TO depTgtVec:NORMALIZED.

    LOCAL depAng TO VANG(depSHat, depTHat).
    LOCAL depAxis TO VCRS(depSHat, depTHat).

    // Если вектор от корабля к цели направлен против орбитального момента,
    // значит цель позади по движению.
    IF vecIsOk(depAxis) AND VDOT(depAxis, depHVec) < 0 {
        SET depAng TO (-1) * depAng.
    }

    // Приводим к [-180, 180)
    IF depAng > 180 {
        SET depAng TO depAng - 360.
    }
    IF depAng <= -180 {
        SET depAng TO depAng + 360.
    }

    RETURN depAng.
}

// Ищет ближайшее будущее время, когда знаковое расстояние до цели
// равно leadAngleDeg.
// leadAngleDeg = 0   -> момент пролёта над целью.
// leadAngleDeg > 0   -> цель впереди корабля на этот угол.
FUNCTION depFindCrossingTime { PARAMETER leadAngleDeg IS 0.
    LOCAL depPeriod TO MAX(60, SHIP:ORBIT:PERIOD).
    LOCAL depMass TO MAX(0.001, SHIP:MASS).
    LOCAL depMinTime TO MAX(30, estimateTurnTime(180, depMass) + 10).
    LOCAL depStep TO MAX(2, depPeriod / MAX(24, coarseDiv)).
    LOCAL depScanEnd TO depMinTime + depPeriod * MAX(1, searchWindows).

    LOCAL depTA TO depMinTime.
    LOCAL depSA TO depSignedSepAt(depTA).
    LOCAL depFA TO depSA - leadAngleDeg.

    LOCAL depBestT TO -1.
    LOCAL depBestE TO 999.
    LOCAL depCnt TO 0.

    // Начальная точка
    IF depSA < 900 {
        IF leadAngleDeg <= 0.2 OR depSA > 0 {
            LOCAL depEA TO ABS(depFA).
            IF depEA < depBestE {
                SET depBestE TO depEA.
                SET depBestT TO depTA.
            }
        }
    }

    UNTIL depTA >= depScanEnd {
        LOCAL depTB TO depTA + depStep.
        IF depTB > depScanEnd {
            SET depTB TO depScanEnd.
        }

        LOCAL depSB TO depSignedSepAt(depTB).
        LOCAL depFB TO depSB - leadAngleDeg.

        IF depSA < 900 AND depSB < 900 {
            // Ищем переход: было больше целевого угла, стало меньше/равно.
            // Для lead > 0 это момент, когда цель ещё впереди, но уже подходит.
            IF depFA > 0 AND depFB <= 0 {
                LOCAL depLoT TO depTA.
                LOCAL depHiT TO depTB.
                LOCAL depIdx TO 0.

                UNTIL depIdx >= 22 OR (depHiT - depLoT) < 0.5 {
                    LOCAL depMidT TO (depLoT + depHiT) / 2.
                    LOCAL depSM TO depSignedSepAt(depMidT).
                    LOCAL depFM TO depSM - leadAngleDeg.

                    IF depSM >= 900 {
                        SET depHiT TO depMidT.
                    } ELSE IF depFM > 0 {
                        SET depLoT TO depMidT.
                    } ELSE {
                        SET depHiT TO depMidT.
                    }

                    SET depIdx TO depIdx + 1.
                }

                RETURN (depLoT + depHiT) / 2.
            }

            // Запасной лучший кандидат.
            // Для lead > 0 принимаем только случаи, когда цель реально впереди.
            IF leadAngleDeg > 0.2 {
                IF depSB > 0 {
                    LOCAL depEB TO ABS(depFB).
                    IF depEB < depBestE {
                        SET depBestE TO depEB.
                        SET depBestT TO depTB.
                    }
                }
            } ELSE {
                LOCAL depEB TO ABS(depFB).
                IF depEB < depBestE {
                    SET depBestE TO depEB.
                    SET depBestT TO depTB.
                }
            }
        }

        SET depTA TO depTB.
        SET depSA TO depSB.
        SET depFA TO depFB.

        SET depCnt TO depCnt + 1.
        IF depCnt >= 8 {
            SET depCnt TO 0.
            WAIT 0.
        }
    }

    RETURN depBestT.
}

// ==========================================================================
// ФАЗА 2: гашение горизонтальной скорости так, чтобы конец торможения
// был над целью.
// ==========================================================================
FUNCTION depPhaseTwo {
    // Сначала грубо ищем момент пролёта над целью без опережения.
    LOCAL depCrossTime TO depFindCrossingTime(0).
    IF NOT isFiniteVal(depCrossTime) OR depCrossTime < 30 {
        PRINT "Фаза 2: момент пролёта над целью не найден.".
        RETURN FALSE.
    }

    LOCAL depMass TO MAX(0.001, SHIP:MASS).
    LOCAL depAccFull TO MAX(0.1, SHIP:MAXTHRUST / depMass).

    // Первичная оценка в момент пролёта над целью.
    LOCAL depPosC TO bodyPosAt(depCrossTime).
    LOCAL depVelC TO VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + depCrossTime)):ORBIT.

    IF NOT vecIsOk(depPosC) OR NOT vecIsOk(depVelC) {
        PRINT "Фаза 2: не удалось получить состояние в момент пролёта.".
        RETURN FALSE.
    }
    IF depPosC:MAG < 1 OR depVelC:MAG < 1 {
        PRINT "Фаза 2: некорректное состояние в момент пролёта.".
        RETURN FALSE.
    }

    LOCAL depUpC TO depPosC:NORMALIZED.
    LOCAL depVRadC TO VDOT(depVelC, depUpC).
    LOCAL depVHorizC TO depVelC - depVRadC * depUpC.
    LOCAL depDvMag TO depVHorizC:MAG.

    IF depDvMag < depMinDv {
        PRINT "Фаза 2: горизонтальная скорость уже мала.".
        RETURN TRUE.
    }
    IF depDvMag > depMaxDv {
        PRINT "Фаза 2: dV слишком велик (" + ROUND(depDvMag, 1) + "), пропуск.".
        RETURN TRUE.
    }

    LOCAL depBurnTime TO depDvMag / depAccFull.
    LOCAL depLeadDeg TO (depVHorizC:MAG / depPosC:MAG) * depBurnTime / 2 * (180 / CONSTANT:PI).

    LOCAL depStartTime TO depCrossTime.

    // Уточняем точку начала торможения.
    // За время горения средняя горизонтальная скорость примерно вдвое меньше,
    // поэтому надо начинать раньше на угол leadDeg.
    LOCAL depIter TO 0.
    UNTIL depIter >= depLeadIter {
        LOCAL depPosS TO bodyPosAt(depStartTime).
        LOCAL depVelS TO VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + depStartTime)):ORBIT.

        IF NOT vecIsOk(depPosS) OR NOT vecIsOk(depVelS) {
            PRINT "Фаза 2: не удалось получить состояние в точке начала.".
            RETURN FALSE.
        }
        IF depPosS:MAG < 1 OR depVelS:MAG < 1 {
            PRINT "Фаза 2: некорректное состояние в точке начала.".
            RETURN FALSE.
        }

        LOCAL depUpS TO depPosS:NORMALIZED.
        LOCAL depVRadS TO VDOT(depVelS, depUpS).
        LOCAL depVHorizS TO depVelS - depVRadS * depUpS.

        SET depDvMag TO depVHorizS:MAG.

        IF depDvMag < depMinDv {
            PRINT "Фаза 2: горизонтальная скорость в точке начала мала.".
            RETURN TRUE.
        }
        IF depDvMag > depMaxDv {
            PRINT "Фаза 2: dV в точке начала слишком велик (" + ROUND(depDvMag, 1) + "), пропуск.".
            RETURN TRUE.
        }

        SET depBurnTime TO depDvMag / depAccFull.
        SET depLeadDeg TO (depVHorizS:MAG / depPosS:MAG) * depBurnTime / 2 * (180 / CONSTANT:PI).

        SET depStartTime TO depFindCrossingTime(depLeadDeg).

        IF NOT isFiniteVal(depStartTime) OR depStartTime < 30 {
            PRINT "Фаза 2: точка начала торможения не найдена.".
            RETURN FALSE.
        }

        SET depIter TO depIter + 1.
    }

    // Финальное состояние в точке начала торможения.
    LOCAL depPosFinal TO bodyPosAt(depStartTime).
    LOCAL depVelFinal TO VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + depStartTime)):ORBIT.

    IF NOT vecIsOk(depPosFinal) OR NOT vecIsOk(depVelFinal) {
        PRINT "Фаза 2: не удалось получить финальное состояние начала.".
        RETURN FALSE.
    }

    LOCAL depUpFinal TO depPosFinal:NORMALIZED.
    LOCAL depVRadFinal TO VDOT(depVelFinal, depUpFinal).
    LOCAL depVHorizFinal TO depVelFinal - depVRadFinal * depUpFinal.

    SET depDvMag TO depVHorizFinal:MAG.

    IF depDvMag < depMinDv {
        PRINT "Фаза 2: финальный dV слишком мал.".
        RETURN TRUE.
    }
    IF depDvMag > depMaxDv {
        PRINT "Фаза 2: финальный dV слишком велик.".
        RETURN TRUE.
    }

    // Ретроградный по горизонтали импульс.
    LOCAL depNodeDvVec TO (-1) * depVHorizFinal.
    SET depBestDvVec TO depNodeDvVec.

    SET depBurnTime TO depDvMag / depAccFull.

    // exec_node начинает горение за burnTime/2 + execLeadSec до ноды.
    // Чтобы горение началось в depStartTime:
    // nodeEta = startTime + burnTime/2 + execLeadSec
    LOCAL depNodeEta TO depStartTime + depBurnTime / 2 + execLeadSec + depNodeBias.
    IF depNodeEta < 30 {
        SET depNodeEta TO 30.
    }

    PRINT "Фаза 2: cross=" + ROUND(depCrossTime, 1) +
          " start=" + ROUND(depStartTime, 1) +
          " lead=" + ROUND(depLeadDeg, 3) + " deg".
    PRINT "Фаза 2: vH=" + ROUND(depDvMag, 1) +
          " dV=" + ROUND(depDvMag, 1) +
          " burn=" + ROUND(depBurnTime, 1) +
          " node=" + ROUND(depNodeEta, 1).

    // Простой прогноз: после импульса горизонтальная скорость должна быть почти нулевой.
    LOCAL depPredVel TO depVelFinal + depNodeDvVec.
    LOCAL depPredVRad TO VDOT(depPredVel, depUpFinal).
    LOCAL depPredVHoriz TO depPredVel - depPredVRad * depUpFinal.
    PRINT "Фаза 2: прогноз остаточной горизонтали=" + ROUND(depPredVHoriz:MAG, 3) + " м/с".

    // Создаём ноду с калибровкой осей.
    LOCAL depDeorbitNode TO NODE(TimeSpan(depNodeEta), 0, 1, 0).
    ADD depDeorbitNode.
    WAIT 0.3.

    IF NOT HASNODE {
        PRINT "Фаза 2: нода не создана.".
        RETURN FALSE.
    }

    LOCAL depAxesList TO calibrateNodeAxes().
    LOCAL depRealRadial TO depAxesList[0].
    LOCAL depRealNormal TO depAxesList[1].
    LOCAL depRealPrograde TO depAxesList[2].

    IF NOT vecIsOk(depRealNormal) OR depRealNormal:MAG < 0.5 {
        PRINT "Фаза 2: калибровка осей не удалась.".
        REMOVE NEXTNODE.
        RETURN FALSE.
    }

    SET NEXTNODE:RADIALOUT TO VDOT(depNodeDvVec, depRealRadial).
    SET NEXTNODE:NORMAL TO VDOT(depNodeDvVec, depRealNormal).
    SET NEXTNODE:PROGRADE TO VDOT(depNodeDvVec, depRealPrograde).
    WAIT 0.2.

    LOCAL depCalibErr TO VANG(NEXTNODE:DELTAV, depNodeDvVec).
    PRINT "Фаза 2: отклонение ноды=" + ROUND(depCalibErr, 3) + " deg".

    IF depCalibErr > 5.0 {
        PRINT "Фаза 2: нода легла криво, отмена.".
        REMOVE NEXTNODE.
        RETURN FALSE.
    }

    UNLOCK THROTTLE.
    UNLOCK STEERING.
    RUN exec_node.
    SET SAS TO FALSE.
    UNTIL NOT HASNODE {
        REMOVE NEXTNODE.
    }

    PRINT "Фаза 2: гашение горизонтали завершено.".
    RETURN TRUE.
}

// ==========================================================================
// ФАЗА 3: агрессивная коррекция к цели на малой/средней тяге
// ==========================================================================
FUNCTION depPhaseThree {
    LOCAL corrThrottle TO 0.
    LOCAL corrSteer TO SHIP:FACING.

    LOCK THROTTLE TO corrThrottle.
    LOCK STEERING TO corrSteer.
    SET WARP TO 0.

    LOCAL corrDone TO FALSE.
    LOCAL diagTimer TO TIME:SECONDS.
    LOCAL tiltLimitDeg TO clamp(corrMaxTiltDeg, 0, 80).

    UNTIL corrDone {

        LOCAL altNow TO MAX(0, SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT).
        LOCAL shipStatus TO SHIP:STATUS.

        IF shipStatus = "LANDED" OR shipStatus = "SPLASHED" {
            SET corrDone TO TRUE.
        }

        // Главное: не спускаемся слишком низко.
        IF NOT corrDone {
            IF altNow <= corrMinAlt {
                SET corrDone TO TRUE.
            }
        }

        IF NOT corrDone {

            LOCAL upNow TO SHIP:UP:FOREVECTOR.
            LOCAL velSurf TO SHIP:VELOCITY:SURFACE.

            LOCAL vVert TO VDOT(velSurf, upNow).
            LOCAL vHoriz TO velSurf - (vVert * upNow).
            LOCAL vHorizMag TO vHoriz:MAG.

            LOCAL gNow TO safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.001).
            LOCAL massNow TO MAX(0.001, SHIP:MASS).
            LOCAL maxAccNow TO MAX(0.1, SHIP:MAXTHRUST / massNow).

            // Вектор на цель в мировой системе
            LOCAL tgtWorld TO BODY:GEOPOSITIONLATLNG(siteLat, siteLng):POSITION.
            LOCAL shipWorld TO SHIP:POSITION.
            LOCAL toTarget TO tgtWorld - shipWorld.

            LOCAL targetVert TO VDOT(toTarget, upNow).
            LOCAL horizToTarget TO toTarget - (targetVert * upNow).
            LOCAL distToTarget TO horizToTarget:MAG.

            // ----------------------------------------------------------
            // Желаемая горизонтальная скорость
            // ----------------------------------------------------------
            LOCAL desiredHSpeed TO 0.

            IF distToTarget > corrExitDist {
                // Планируем не "до земли", а короткий горизонт 3..20 секунд.
                LOCAL planTime TO distToTarget / MAX(1, corrMaxVel * 0.75).
                SET planTime TO clamp(planTime, 3, 20).

                SET desiredHSpeed TO MIN(corrMaxVel, distToTarget / planTime).

                // Если уже летим быстрее нужного — не разгоняемся ещё сильнее.
                IF vHorizMag > desiredHSpeed * 1.25 {
                    SET desiredHSpeed TO vHorizMag * 0.75.
                }
            }

            LOCAL desiredHVec TO V(0, 0, 0).
            IF distToTarget > 1 AND desiredHSpeed > 0.1 {
                SET desiredHVec TO horizToTarget:NORMALIZED * desiredHSpeed.
            }

            // ----------------------------------------------------------
            // Горизонтальное ускорение
            // ----------------------------------------------------------
            LOCAL hErr TO desiredHVec - vHoriz.
            LOCAL desiredHAcc TO hErr * corrKp.
            LOCAL desiredHAccMag TO desiredHAcc:MAG.

            IF desiredHAccMag > corrMaxAcc {
                SET desiredHAcc TO desiredHAcc:NORMALIZED * corrMaxAcc.
                SET desiredHAccMag TO corrMaxAcc.
            }

            // ----------------------------------------------------------
            // Вертикальная безопасность
            // Не даём фазе 3 разогнать ракету вниз.
            // ----------------------------------------------------------
            LOCAL vertAcc TO 0.
            LOCAL vVertErr TO corrSafeVVert - vVert.

            IF vVertErr > 5 {
                // Падаем слишком быстро — тормозим вверх.
                SET vertAcc TO MIN(corrMaxAcc, vVertErr * 0.35).
            } ELSE IF vVertErr < -15 {
                // Падаем слишком медленно или растём — немного помогаем гравитации.
                SET vertAcc TO MAX(-corrMaxAcc * 0.25, vVertErr * 0.15).
            }

            LOCAL cmdAcc TO desiredHAcc + (vertAcc * upNow).

            // ----------------------------------------------------------
            // Ограничение по наклону.
            // Горизонтальное ускорение не может быть больше g * tan(tilt).
            // ----------------------------------------------------------
            LOCAL horizCmd TO cmdAcc - (VDOT(cmdAcc, upNow) * upNow).
            LOCAL horizCmdMag TO horizCmd:MAG.
            LOCAL maxHorizAcc TO gNow * TAN(tiltLimitDeg).

            IF horizCmdMag > maxHorizAcc AND horizCmdMag > 0.01 {
                SET horizCmd TO horizCmd:NORMALIZED * maxHorizAcc.
                SET cmdAcc TO horizCmd + (vertAcc * upNow).
            }

            // ----------------------------------------------------------
            // Желаемый вектор тяги
            // ----------------------------------------------------------
            LOCAL thrustWant TO (gNow * upNow + cmdAcc).

            IF NOT vecIsOk(thrustWant) OR thrustWant:MAG < 0.01 {
                SET thrustWant TO upNow.
            } ELSE {
                SET thrustWant TO thrustWant:NORMALIZED.
            }

            LOCAL neededAcc TO (gNow * upNow + cmdAcc):MAG.
            SET corrThrottle TO clamp(neededAcc / maxAccNow, 0, corrMaxThrottle).

            // ----------------------------------------------------------
            // Руль
            // ----------------------------------------------------------
            LOCAL tiltDeg TO VANG(thrustWant, upNow).

            IF tiltDeg < 1.0 {
                SET corrSteer TO rotateToward(corrSteer, upNow, 30, 0.1).
            } ELSE {
                SET corrSteer TO LOOKDIRUP(thrustWant, upNow).
            }

            // ----------------------------------------------------------
            // Ранний выход: достаточно близко и не слишком быстро падаем
            // ----------------------------------------------------------
            IF distToTarget <= corrExitDist AND
               vHorizMag <= corrExitVel AND
               vVert >= corrSafeVVert - 15 {
                SET corrDone TO TRUE.
            }

            // Запасной ранний выход: если уже почти у цели, но не идеально
            IF NOT corrDone {
                IF altNow <= corrMinAlt + 1500 AND
                   distToTarget <= corrExitDist * 2.5 AND
                   vHorizMag <= corrExitVel * 2.5 AND
                   vVert >= corrSafeVVert - 25 {
                    SET corrDone TO TRUE.
                }
            }

            // ----------------------------------------------------------
            // Диагностика
            // ----------------------------------------------------------
            IF TIME:SECONDS > diagTimer {
                PRINT "Фаза 3: dist=" + ROUND(distToTarget) +
                      " vH=" + ROUND(vHorizMag, 1) +
                      " vV=" + ROUND(vVert, 1) +
                      " alt=" + ROUND(altNow) +
                      " thr=" + ROUND(corrThrottle * 100) + "%" +
                      " tilt=" + ROUND(tiltDeg, 1).
                SET diagTimer TO TIME:SECONDS + 1.
            }

            WAIT 0.
        }
    }

    SET corrThrottle TO 0.
    WAIT 0.05.

    UNLOCK THROTTLE.
    UNLOCK STEERING.
    SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

    PRINT "Фаза 3: коррекция завершена, передача посадке.".
    RETURN TRUE.
}

// ==========================================================================
// Старт
// ==========================================================================

PRINT "--- Деорбит: Начало ---".
SET WARP TO 0.
IF NOT (DEFINED siteLat) { SET siteLat TO 0. }
IF NOT (DEFINED siteLng) { SET siteLng TO 0. }
IF NOT (DEFINED siteTerrain) { SET siteTerrain TO 0. }
UNTIL NOT HASNODE { REMOVE NEXTNODE. }
WAIT 0.

SET depCanRun TO TRUE.
IF NOT isFiniteVal(siteLat) OR NOT isFiniteVal(siteLng) OR NOT isFiniteVal(siteTerrain) {
    PRINT "Деорбит: плохие координаты цели.".
    SET depCanRun TO FALSE.
}
IF depCanRun {
    IF NOT vecIsOk(SHIP:VELOCITY:ORBIT) OR SHIP:VELOCITY:ORBIT:MAG < 1 {
        PRINT "Деорбит: нет орбитальной скорости.".
        SET depCanRun TO FALSE.
    }
}
IF depCanRun {
    IF NOT isFiniteVal(SHIP:ORBIT:PERIAPSIS) OR SHIP:ORBIT:PERIAPSIS <= 0 {
        PRINT "Деорбит: уже на суборбитальной траектории, деорбит не нужен.".
        SET depCanRun TO FALSE.
    }
}

IF depCanRun { SET depCanRun TO depPhaseOne(). }
IF depCanRun { SET depCanRun TO depPhaseTwo(). }
IF depCanRun AND doCorrection { SET depCanRun TO depPhaseThree(). }
IF depCanRun {
    PRINT "--- Деорбит: Завершен ---".
} ELSE {
    PRINT "--- Деорбит: Пропущен/отменен ---".
}