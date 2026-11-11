@lazyglobal on.
// Фаза 1: смена наклонения, чтобы орбита пролегала над целью
// Фаза 2: гашение горизонтальной скорости на орбите над целью
// Читает глобальные переменные из land.ks: siteLat, siteLng, siteTerrain
// ==========================================================================

run exec_node.lib.ks.
run utils.

parameter horizStartAlt is 10000.
parameter targetTolM is 200.
parameter coarseDiv is 96.
parameter fineDiv is 8.
parameter searchWindows is 2.
parameter minBurnDv is 1.0.
parameter maxBurnDv is 3000.
parameter doCorrection is true.

parameter corrMaxAcc is 6.0.
parameter corrMaxVel is 120.0.
parameter corrExitDist is 250.0.
parameter corrMinAlt is 6000.0.
parameter corrExitVel is 8.0.
parameter corrMaxTiltDeg is 45.0.
parameter corrMaxThrottle is 0.90.
parameter corrKp is 0.8.
parameter corrSafeVVert is -45.0.
parameter depNodeBias is 0.
parameter depLeadIter is 2.

set depMinDv to minBurnDv.
set depMaxDv to maxBurnDv.
set depBestDvVec to V(0, 0, 0).

// ==========================================================================
// Геометрия цели с учётом вращения тела
// ==========================================================================
function depTargetVec { parameter latDeg. parameter lngDeg. parameter terrM. parameter deltaSec.
    local geoPoint to BODY:GEOPOSITIONLATLNG(latDeg, lngDeg).
    local baseVec to geoPoint:POSITION - BODY:POSITION.
    if not vecIsOk(baseVec) or baseVec:MAG < 1 { return V(0, 0, 0). }

    local dirVec to baseVec:NORMALIZED.
    local angVelVec to BODY:ANGULARVEL.
    if vecIsOk(angVelVec) and angVelVec:MAG > 0.00000001 {
        local rotAngleDeg to angVelVec:MAG * deltaSec * (180 / CONSTANT:PI).
        set rotAngleDeg to normDeg360(rotAngleDeg).
        set dirVec to ANGLEAXIS(rotAngleDeg, angVelVec:NORMALIZED) * dirVec.
    }
    return (BODY:RADIUS + terrM) * dirVec.
}

// ==========================================================================
// Нормаль целевой плоскости (для фазы 1)
// ==========================================================================
function depTargetNormal { parameter latDeg. parameter lngDeg. parameter terrM.
    local tgtVec to depTargetVec(latDeg, lngDeg, terrM, 0).
    if not vecIsOk(tgtVec) or tgtVec:MAG < 1 { return V(0, 0, 0). }
    local tgtHat to tgtVec:NORMALIZED.
    local spinAxis to BODY:ANGULARVEL.
    if not vecIsOk(spinAxis) or spinAxis:MAG < 0.00000001 { return V(0, 1, 0). }
    set spinAxis to spinAxis:NORMALIZED.
    local spinProj to spinAxis - (VDOT(spinAxis, tgtHat) * tgtHat).
    if not vecIsOk(spinProj) or spinProj:MAG < 0.000001 { return V(0, 0, 0). }
    return spinProj:NORMALIZED.
}

// ==========================================================================
// Знаковый угол от корабля к цели вдоль орбиты
// Положительный = цель впереди, отрицательный = позади
// ==========================================================================
function depSignedSepAt { parameter tSec.
    local depPos to bodyPosAt(tSec).
    local depVel to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + tSec)):ORBIT.
    if not vecIsOk(depPos) or not vecIsOk(depVel) { return 999. }
    if depPos:MAG < 1 or depVel:MAG < 0.01 { return 999. }

    local depHVec to VCRS(depPos, depVel).
    if not vecIsOk(depHVec) or depHVec:MAG < 0.000001 { return 999. }
    set depHVec to depHVec:NORMALIZED.

    local depTgtVec to depTargetVec(siteLat, siteLng, siteTerrain, tSec).
    if not vecIsOk(depTgtVec) or depTgtVec:MAG < 1 { return 999. }

    local depSHat to depPos:NORMALIZED.
    local depTHat to depTgtVec:NORMALIZED.
    local depAng to VANG(depSHat, depTHat).
    if not isFiniteVal(depAng) { return 999. }

    local depAxis to VCRS(depSHat, depTHat).
    if vecIsOk(depAxis) and VDOT(depAxis, depHVec) < 0 {
        set depAng to (-1) * depAng.
    }
    set depAng to wrap180Deg(depAng).
    return depAng.
}

// ==========================================================================
// Поиск момента, когда знаковое расстояние равно leadAngleDeg.
// ИСПРАВЛЕНИЕ: возвращает -1 если пересечение не найдено.
// ==========================================================================
function depFindCrossingTime { parameter leadAngleDeg.
    local depPeriod to MAX(60, SHIP:ORBIT:PERIOD).
    local depMass to MAX(0.001, SHIP:MASS).
    local depMinTime to MAX(30, estimateTurnTime(180, depMass) + 10).
    local depStep to MAX(2, depPeriod / MAX(24, coarseDiv)).
    local depScanEnd to depMinTime + depPeriod * MAX(1, searchWindows).

    local depTA to depMinTime.
    local depSA to depSignedSepAt(depTA).
    local depFA to depSA - leadAngleDeg.
    local depCnt to 0.

    until depTA >= depScanEnd {
        local depTB to depTA + depStep.
        if depTB > depScanEnd { set depTB to depScanEnd. }

        local depSB to depSignedSepAt(depTB).
        local depFB to depSB - leadAngleDeg.

        if isFiniteVal(depSA) and isFiniteVal(depSB) and
           ABS(depSA) < 900 and ABS(depSB) < 900 {
            // Переход через искомый угол
            if (depFA > 0 and depFB <= 0) or (depFA < 0 and depFB >= 0) {
                // Бисекция
                local depLoT to depTA.
                local depHiT to depTB.
                local depIdx to 0.
                until depIdx >= 22 or (depHiT - depLoT) < 0.5 {
                    local depMidT to (depLoT + depHiT) / 2.
                    local depSM to depSignedSepAt(depMidT).
                    local depFM to depSM - leadAngleDeg.
                    if not isFiniteVal(depFM) or ABS(depSM) >= 900 {
                        set depHiT to depMidT.
                    } else if depFM > 0 {
                        set depLoT to depMidT.
                    } else {
                        set depHiT to depMidT.
                    }
                    set depIdx to depIdx + 1.
                }
                return (depLoT + depHiT) / 2.
            }
        }

        set depTA to depTB.
        set depSA to depSB.
        set depFA to depFB.
        set depCnt to depCnt + 1.
        if depCnt >= 8 { set depCnt to 0. wait 0. }
    }

    // Пересечение не найдено
    return -1.
}

// ==========================================================================
// Оценка времени свободного падения
// ==========================================================================
function depEstimateFallTime { parameter posVec.
    local altNow to MAX(0, posVec:MAG - BODY:RADIUS).
    if altNow < 10 { return 0. }
    local gNow to safeDiv(BODY:MU, posVec:MAG * posVec:MAG, 0.001).
    if gNow <= 0.000001 { return 0. }
    return SQRT(2 * altNow / gNow).
}

// ==========================================================================
// Угловая скорость вращения тела с учётом направления
// ==========================================================================
function depBodyRotRate { parameter posVec. parameter velVec.
    local rotPeriod to BODY:ROTATIONPERIOD.
    if not isFiniteVal(rotPeriod) or rotPeriod <= 1 { return 0. }
    local rotRate to 360 / rotPeriod.
    local hVec to VCRS(posVec, velVec).
    local spinAxis to BODY:ANGULARVEL.
    if vecIsOk(hVec) and vecIsOk(spinAxis) and
       hVec:MAG > 0.000001 and spinAxis:MAG > 0.000001 {
        if VDOT(hVec:NORMALIZED, spinAxis:NORMALIZED) < 0 {
            set rotRate to (-1) * rotRate.
        }
    }
    return rotRate.
}

// ==========================================================================
// ФАЗА 1: смена наклонения
// ==========================================================================
function depInclinationDv { parameter tSec. parameter tgtNormal.
    local velAtT to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + tSec)):ORBIT.
    local posAtT to bodyPosAt(tSec).
    if not vecIsOk(posAtT) or not vecIsOk(velAtT) { return LIST(false, V(0,0,0), 0). }
    if posAtT:MAG < 1 or velAtT:MAG < 1 { return LIST(false, V(0,0,0), 0). }

    local radialHat to posAtT:NORMALIZED.
    local vMag to velAtT:MAG.
    local oldHVec to VCRS(posAtT, velAtT).
    local oldNormal to V(0, 1, 0).
    if vecIsOk(oldHVec) and oldHVec:MAG > 0.000001 { set oldNormal to oldHVec:NORMALIZED. }

    local tiltAngle to VANG(oldNormal, tgtNormal).
    local vRad to VDOT(velAtT, radialHat).
    local vTanSq to vMag * vMag - vRad * vRad.
    if vTanSq < 0 { set vTanSq to 0. }
    local vTan to SQRT(vTanSq).

    local tgtDir to VCRS(tgtNormal, radialHat).
    local velNew to V(0, 0, 0).

    if vecIsOk(tgtDir) and tgtDir:MAG > 0.000001 {
        set tgtDir to tgtDir:NORMALIZED.
        local curTanDir to velAtT - vRad * radialHat.
        if vecIsOk(curTanDir) and curTanDir:MAG > 0.000001 {
            set curTanDir to curTanDir:NORMALIZED.
            if VDOT(tgtDir, curTanDir) < 0 { set tgtDir to (-1) * tgtDir. }
        }
        set velNew to vRad * radialHat + vTan * tgtDir.
    }

    local checkAng to 999.
    if vecIsOk(velNew) {
        local newHVec to VCRS(posAtT, velNew).
        if vecIsOk(newHVec) and newHVec:MAG > 0.000001 {
            set checkAng to VANG(newHVec:NORMALIZED, tgtNormal).
        }
    }

    // Запасной метод: поворот вокруг линии узлов
    if checkAng > 1.0 {
        local crossLine to VCRS(oldNormal, tgtNormal).
        if vecIsOk(crossLine) and crossLine:MAG > 0.000001 {
            set crossLine to crossLine:NORMALIZED.
            local velRotPlus to ANGLEAXIS(tiltAngle, crossLine) * velAtT.
            local velRotMinus to ANGLEAXIS((-1) * tiltAngle, crossLine) * velAtT.
            local hPlus to VCRS(posAtT, velRotPlus).
            local errPlus to 999.
            if vecIsOk(hPlus) and hPlus:MAG > 0.000001 { set errPlus to VANG(hPlus:NORMALIZED, tgtNormal). }
            local hMinus to VCRS(posAtT, velRotMinus).
            local errMinus to 999.
            if vecIsOk(hMinus) and hMinus:MAG > 0.000001 { set errMinus to VANG(hMinus:NORMALIZED, tgtNormal). }
            if errPlus < errMinus {
                set velNew to velRotPlus.
            } else {
                set velNew to velRotMinus.
            }
        }
    }

    if not vecIsOk(velNew) or velNew:MAG < 1 { return LIST(false, V(0,0,0), 0). }
    local dvResult to velNew - velAtT.
    if not vecIsOk(dvResult) { return LIST(false, V(0,0,0), 0). }
    return LIST(true, dvResult, tiltAngle).
}

function depPhaseOne {
    local needInc to ABS(siteLat).
    local curInc to SHIP:ORBIT:INCLINATION.
    print "Фаза 1: нужно наклонение " + ROUND(needInc, 2) +
          " (сейчас " + ROUND(curInc, 2) + ").".

    if curInc >= needInc - 0.15 {
        print "Фаза 1: наклонение уже подходит, пропуск.".
        return true.
    }

    local tgtNormal to depTargetNormal(siteLat, siteLng, siteTerrain).
    if not vecIsOk(tgtNormal) or tgtNormal:MAG < 0.000001 { return false. }

    local depMass to MAX(0.001, SHIP:MASS).
    local depPeriod to MAX(60, SHIP:ORBIT:PERIOD).
    local minTime to MAX(30, estimateTurnTime(90, depMass) + 10).
    local crossTime to findPlaneCrossTime(tgtNormal, minTime, depPeriod).
    if not isFiniteVal(crossTime) or crossTime < 5 { return false. }

    local incResult to depInclinationDv(crossTime, tgtNormal).
    if not incResult[0] { return false. }

    local incDvVec to incResult[1].
    local incDvMag to incDvVec:MAG.
    print "Фаза 1: узел через " + ROUND(crossTime, 1) +
          " с, dV=" + ROUND(incDvMag, 1) + " м/с".

    if incDvMag < depMinDv { return true. }
    if incDvMag > depMaxDv { return true. }

    set depBestDvVec to incDvVec.
    local incNode to NODE(TIMESPAN(crossTime), 0, 1, 0).
    add incNode. wait 0.3.
    if not HASNODE { return false. }

    local axesList to calibrateNodeAxes().
    if not vecIsOk(axesList[1]) or axesList[1]:MAG < 0.5 {
        remove NEXTNODE. return false.
    }

    set NEXTNODE:RADIALOUT to VDOT(incDvVec, axesList[0]).
    set NEXTNODE:NORMAL to VDOT(incDvVec, axesList[1]).
    set NEXTNODE:PROGRADE to VDOT(incDvVec, axesList[2]).
    wait 0.2.

    local calibErr to VANG(NEXTNODE:DELTAV, incDvVec).
    if not isFiniteVal(calibErr) or calibErr > 5.0 {
        remove NEXTNODE. return false.
    }

    print "Фаза 1: выполняю манёвр.".
    unlock THROTTLE. unlock STEERING.
    run exec_node.
    wait 1.
    print "Фаза 1: завершена. Новое inc=" + ROUND(SHIP:ORBIT:INCLINATION, 3) + ".".
    return true.
}

// ==========================================================================
// ФАЗА 2: гашение горизонтальной скорости
// ==========================================================================
function depPhaseTwo {
    // Момент пролёта над целью
    local depCrossTime to depFindCrossingTime(0).
    if not isFiniteVal(depCrossTime) or depCrossTime < 30 {
        print "Фаза 2: момент пролёта над целью не найден.".
        return false.
    }

    local depMass to MAX(0.001, SHIP:MASS).
    local depAccFull to MAX(0.1, SHIP:MAXTHRUST / depMass).

    // Первичная оценка
    local depPosC to bodyPosAt(depCrossTime).
    local depVelC to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + depCrossTime)):ORBIT.
    if not vecIsOk(depPosC) or not vecIsOk(depVelC) { return false. }
    if depPosC:MAG < 1 or depVelC:MAG < 1 { return false. }

    local depUpC to depPosC:NORMALIZED.
    local depVHorizC to depVelC - (VDOT(depVelC, depUpC) * depUpC).
    local depDvMag to depVHorizC:MAG.

    if depDvMag < depMinDv {
        print "Фаза 2: горизонтальная скорость уже мала.".
        return true.
    }
    if depDvMag > depMaxDv {
        print "Фаза 2: dV слишком велик (" + ROUND(depDvMag, 1) + ").".
        return true.
    }

    // ЕДИНОЕ время горения для всех расчётов
    local depBurnTime to sumBurnTime(depDvMag).
    if not isFiniteVal(depBurnTime) or depBurnTime < 1 {
        set depBurnTime to depDvMag / depAccFull.
    }

    // Угол опережения с учётом падения и вращения
    local depTFall to depEstimateFallTime(depPosC).
    local depRotRate to depBodyRotRate(depPosC, depVelC).
    local depLeadDeg to (depVHorizC:MAG / depPosC:MAG) * (depBurnTime / 2) * (180 / CONSTANT:PI)
                        - depRotRate * (depBurnTime + depTFall).

    // ИСПРАВЛЕНИЕ: ограничиваем угол разумным диапазоном
    set depLeadDeg to clamp(depLeadDeg, 0, 90).

    local depStartTime to depCrossTime.
    local depIter to 0.

    until depIter >= depLeadIter {
        local depPosS to bodyPosAt(depStartTime).
        local depVelS to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + depStartTime)):ORBIT.
        if not vecIsOk(depPosS) or not vecIsOk(depVelS) { return false. }
        if depPosS:MAG < 1 or depVelS:MAG < 1 { return false. }

        local depUpS to depPosS:NORMALIZED.
        local depVHorizS to depVelS - (VDOT(depVelS, depUpS) * depUpS).
        set depDvMag to depVHorizS:MAG.

        if depDvMag < depMinDv { return true. }
        if depDvMag > depMaxDv { return true. }

        set depBurnTime to sumBurnTime(depDvMag).
        if not isFiniteVal(depBurnTime) or depBurnTime < 1 {
            set depBurnTime to depDvMag / depAccFull.
        }

        set depTFall to depEstimateFallTime(depPosS).
        set depRotRate to depBodyRotRate(depPosS, depVelS).
        set depLeadDeg to (depVHorizC:MAG / depPosC:MAG) * (depBurnTime / 2) * (180 / CONSTANT:PI)
                            - depRotRate * (depBurnTime + depTFall).
        set depLeadDeg to clamp(depLeadDeg, 0, 90).

        local depNewStart to depFindCrossingTime(depLeadDeg).
        if not isFiniteVal(depNewStart) or depNewStart < 30 {
            // Пересечение не найдено — используем момент пролёта
            print "Фаза 2: опережение не найдено, торможу над целью.".
            set depStartTime to depCrossTime.
            set depLeadDeg to 0.
            set depIter to depLeadIter.
        } else {
            set depStartTime to depNewStart.
        }
        set depIter to depIter + 1.
    }

    // Финальное состояние
    local depPosFinal to bodyPosAt(depStartTime).
    local depVelFinal to VELOCITYAT(SHIP, TIMESTAMP(TIME:SECONDS + depStartTime)):ORBIT.
    if not vecIsOk(depPosFinal) or not vecIsOk(depVelFinal) { return false. }

    local depUpFinal to depPosFinal:NORMALIZED.
    local depVHorizFinal to depVelFinal - (VDOT(depVelFinal, depUpFinal) * depUpFinal).
    set depDvMag to depVHorizFinal:MAG.

    if depDvMag < depMinDv { return true. }
    if depDvMag > depMaxDv { return true. }

    local depNodeDvVec to (-1) * depVHorizFinal.
    set depBestDvVec to depNodeDvVec.

    // Нода в центре импульса
    local depNodeEta to depStartTime + depBurnTime / 2 + depNodeBias.
    set depNodeEta to MAX(30, depNodeEta).

    print "Фаза 2: cross=" + ROUND(depCrossTime, 1) +
          " start=" + ROUND(depStartTime, 1) +
          " lead=" + ROUND(depLeadDeg, 3) + " deg" +
          " tFall=" + ROUND(depTFall, 1) + " с" +
          " rotRate=" + ROUND(depRotRate, 6) + " deg/с".
    print "Фаза 2: vH=" + ROUND(depDvMag, 1) +
          " burn=" + ROUND(depBurnTime, 1) +
          " node=" + ROUND(depNodeEta, 1).

    // Создаём ноду
    local depDeorbitNode to NODE(TIMESPAN(depNodeEta), 0, 1, 0).
    add depDeorbitNode. wait 0.3.
    if not HASNODE { return false. }

    local depAxesList to calibrateNodeAxes().
    if not vecIsOk(depAxesList[1]) or depAxesList[1]:MAG < 0.5 {
        remove NEXTNODE. return false.
    }

    set NEXTNODE:RADIALOUT to VDOT(depNodeDvVec, depAxesList[0]).
    set NEXTNODE:NORMAL to VDOT(depNodeDvVec, depAxesList[1]).
    set NEXTNODE:PROGRADE to VDOT(depNodeDvVec, depAxesList[2]).
    wait 0.2.

    local depCalibErr to VANG(NEXTNODE:DELTAV, depNodeDvVec).
    if not isFiniteVal(depCalibErr) or depCalibErr > 5.0 {
        remove NEXTNODE. return false.
    }

    unlock THROTTLE. unlock STEERING.
    run exec_node.
    print "Фаза 2: гашение горизонтали завершено.".
    return true.
}

function depPhaseThree { parameter maxTime.
    local shipPos to bodyPosAt(0).
    local shipVel to SHIP:VELOCITY:ORBIT.
    if not vecIsOk(shipPos) or not vecIsOk(shipVel) { return true. }
    if shipPos:MAG < 1 or shipVel:MAG < 1 { return true. }

    local upNow to shipPos:NORMALIZED.
    local altNow to shipPos:MAG - BODY:RADIUS.
    local vVert to VDOT(shipVel, upNow).
    local vHoriz to shipVel - (vVert * upNow).

    // Оценка времени до поверхности (баллистика)
    local gNow to safeDiv(BODY:MU, shipPos:MAG * shipPos:MAG, 0.001).
    local tSurf to 9999.
    if vVert < -0.5 {
        local disc to vVert * vVert + 2 * gNow * altNow.
        if isFiniteVal(disc) and disc > 0 {
            set tSurf to (SQRT(disc) - vVert) / MAX(0.001, gNow).
        }
    }

    if not isFiniteVal(tSurf) or tSurf < 5 {
        print "Фаза 3: нет оценки времени до поверхности.".
        return true.
    }

    // Позиция цели с учётом вращения за время падения
    local tgtVec to depTargetVec(siteLat, siteLng, siteTerrain, tSurf).
    if not vecIsOk(tgtVec) or tgtVec:MAG < 1 { return true. }

    // Предсказанная позиция корабля в момент достижения поверхности
    local predPos to shipPos + vHoriz * tSurf.
    local predHoriz to predPos - (VDOT(predPos, upNow) * upNow).
    local tgtHoriz to tgtVec - (VDOT(tgtVec, upNow) * upNow).
    local missHoriz to tgtHoriz - predHoriz.
    local missDist to missHoriz:MAG.

    print "Фаза 3: промах=" + ROUND(missDist) +
          " м, tSurf=" + ROUND(tSurf, 1) +
          " с, alt=" + ROUND(altNow).

    // Промах мал — коррекция не нужна
    if missDist < corrExitDist {
        print "Фаза 3: коррекция не нужна.".
        return true.
    }

    // dV = промах / время до поверхности
    // Чем раньше коррекция, тем меньше dV
    local corrDvMag to missDist / MAX(10, tSurf).

    // Лимит: не более 50 м/с на одной коррекции
    set corrDvMag to MIN(corrDvMag, 50).

    if corrDvMag < minBurnDv {
        print "Фаза 3: коррекция мала (" + ROUND(corrDvMag, 1) + " м/с), пропуск.".
        return true.
    }

    local corrDv to missHoriz:NORMALIZED * corrDvMag.
    print "Фаза 3: коррекция=" + ROUND(corrDvMag, 1) + " м/с".
    if ( maxTime < 30 ) {
        print "Фаза 3: коррекция отменена по времени!".
        return false.
    }

    // Ноду ставим через 30 секунд — минимальная задержка
    local corrNode to NODE(TIMESPAN(30), 0, 1, 0).
    add corrNode. wait 0.3.
    if not HASNODE { return true. }

    local axesList to calibrateNodeAxes().
    set NEXTNODE:RADIALOUT to VDOT(corrDv, axesList[0]).
    set NEXTNODE:NORMAL to VDOT(corrDv, axesList[1]).
    set NEXTNODE:PROGRADE to VDOT(corrDv, axesList[2]).
    wait 0.2.

    unlock THROTTLE. unlock STEERING.
    run exec_node.
    print "Фаза 3: коррекция завершена.".
    return true.
}

function isStillHigh {
    // Пример использования для ПОЛНОЙ остановки относительно поверхности:
    //local speed_to_kill is SHIP:VELOCITY:SURFACE:MAG.
    // 4. Истинная высота до поверхности (высота радара)
    local radar_alt is SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT.
    print "calc bd: " + ROUND(calculate_burn_dist(SHIP:VERTICALSPEED)).
    return VDOT(SHIP:VELOCITY:SURFACE, SHIP:UP:FOREVECTOR) >= 0 or radar_alt > calculate_burn_dist(SHIP:VERTICALSPEED) * 2.
}

// ==========================================================================
// Старт
// ==========================================================================
print "--- Деорбит: Начало ---".
set WARP to 0.

if not (defined siteLat) { set siteLat to 0. }
if not (defined siteLng) { set siteLng to 0. }
if not (defined siteTerrain) { set siteTerrain to 0. }

// Проверяем чужие ноды вместо удаления
if HASNODE {
    print "Деорбит: есть чужая маневровая нода. Удали вручную.".
    exit.
}

set depCanRun to true.

if not isFiniteVal(siteLat) or not isFiniteVal(siteLng) or not isFiniteVal(siteTerrain) {
    print "Деорбит: плохие координаты цели.".
    set depCanRun to false.
}

if depCanRun and (not vecIsOk(SHIP:VELOCITY:ORBIT) or SHIP:VELOCITY:ORBIT:MAG < 1) {
    print "Деорбит: нет орбитальной скорости.".
    set depCanRun to false.
}

if depCanRun and (not isFiniteVal(SHIP:ORBIT:PERIAPSIS) or SHIP:ORBIT:PERIAPSIS <= 0) {
    print "Деорбит: уже на суборбитальной траектории.".
    set depCanRun to false.
}

if depCanRun and not isStillHigh {
    print "Деорбит: падаем! выход.".
    set depCanRun to false.
}

if depCanRun { set depCanRun to depPhaseOne(). }
if depCanRun and not isStillHigh {
    print "Деорбит: падаем! выход.".
    set depCanRun to false.
}
if depCanRun { set depCanRun to depPhaseTwo(). }
if depCanRun and not isStillHigh {
    print "Деорбит: падаем! выход.".
    set depCanRun to false.
}
local bdist is calculate_burn_dist(SHIP:VERTICALSPEED).
local maxtime is get_freefall_time_with_v(SHIP:ALTITUDE, bdist, SHIP:VERTICALSPEED).
print "max time: " + disTime(maxtime).
if depCanRun and doCorrection { set depCanRun to depPhaseThree(maxtime). }

if depCanRun {
    print "--- Деорбит: Завершен ---".
} else {
    print "--- Деорбит: Пропущен/отменен ---".
}