// ==========================================================================
// land.ks — посадка на безатмосферное тело (kOS 1.6)
// ==========================================================================
// Модульная структура:
//   utils.ks              — общие утилиты (clamp, safeDiv, ...)
//   find_landing_site.ks  — выбор точки + поиск площадки
//   suicide_burn.ks       — suicide burn с регуляцией тяги
//   exec_node.ks          — выполнение маневренной ноды
//
// 4 фазы посадки:
//   1. horizontal — гашение горизонтальной скорости
//   2. coast      — падение с промоткой времени до suicide burn
//   3. suicide    — вызов suicide_burn.ks
//   4. final      — финальная мягкая посадка
// ==========================================================================

PARAMETER landLat        IS 0.
PARAMETER landLng        IS 0.
PARAMETER maxSlope       IS 5.
PARAMETER touchdownSpeed IS 2.
PARAMETER baseHorizTol   IS 2.
PARAMETER brakeSafetyMargin IS 2.0.
PARAMETER coastLeadTime  IS 30.
PARAMETER maxTiltDeg     IS 5.    // Максимальный наклон от вертикали

RUN utils.

// ==========================================================================
//  РАСЧЁТ ТОЧКИ ТОРМОЖЕНИЯ (для ноды деорбита)
// ==========================================================================

FUNCTION findBrakingPoint {
    PARAMETER landLat. PARAMETER landLng. PARAMETER landTerrain.
    LOCAL totalLng TO landLng + BODY:ROTATIONANGLE.
    LOCAL latR TO landLat * CONSTANT:PI / 180.
    LOCAL lngR TO totalLng * CONSTANT:PI / 180.
    LOCAL rLand TO BODY:RADIUS + landTerrain.
    LOCAL landVec TO V(rLand*COS(latR)*COS(lngR), rLand*SIN(latR), rLand*COS(latR)*SIN(lngR)).
    LOCAL bestTrueAnom TO SHIP:ORBIT:TRUEANOMALY.
    LOCAL bestAngle TO 360.
    LOCAL bestETA TO 0.
    LOCAL nu TO 0.
    UNTIL nu >= 360 {
        LOCAL posAtNu TO POSITIONAT(SHIP, TIME:SECONDS + SHIP:ORBIT:PERIOD * nu / 360).
        LOCAL velAtNu TO VELOCITYAT(SHIP, TIME:SECONDS + SHIP:ORBIT:PERIOD * nu / 360):ORBIT.
        LOCAL toLand TO landVec - posAtNu.
        LOCAL rHat TO posAtNu:NORMALIZED.
        LOCAL vHoriz TO velAtNu - (VDOT(velAtNu, rHat) * rHat).
        IF vHoriz:MAG > 0.1 {
            LOCAL angle TO VANG(vHoriz, toLand).
            IF angle < bestAngle {
                SET bestAngle TO angle.
                SET bestTrueAnom TO nu.
                SET bestETA TO SHIP:ORBIT:PERIOD * nu / 360.
            }
        }
        SET nu TO nu + 10.
    }
    RETURN List(bestTrueAnom, bestETA, bestAngle).
}

// ==========================================================================
//  ГЛАВНЫЙ БЛОК
// ==========================================================================
SET scriptDone TO FALSE.

CLEARSCREEN.
SET SAS TO FALSE.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
SET throttleSet TO 0.
LOCK THROTTLE TO throttleSet.

UNTIL scriptDone {

    IF BODY:ATM:EXISTS { PRINT "ВНИМАНИЕ: у тела есть атмосфера, расчёт на вакуум.". }
    
    // Проверка типа орбиты
    SET alreadyDescending TO FALSE.
    
    IF SHIP:ORBIT:ECCENTRICITY < 1 {
        PRINT "Орбита замкнута (e=" + ROUND(SHIP:ORBIT:ECCENTRICITY, 3) + ").".
    } ELSE IF SHIP:ORBIT:PERIAPSIS < 0 {
        PRINT "Суборбитальная траектория (Pe=" + ROUND(SHIP:ORBIT:PERIAPSIS) + " м).".
        PRINT "Ракета уже снижается.".
        SET alreadyDescending TO TRUE.
    } ELSE {
        PRINT "ОШИБКА: гиперболическая траектория (Pe=" + ROUND(SHIP:ORBIT:PERIAPSIS) + " м).".
        SET scriptDone TO TRUE.
    }
    
    IF SHIP:MAXTHRUST <= 0.01 {
        PRINT "ОШИБКА: нет тяги.".
        SET scriptDone TO TRUE.
    }
    IF scriptDone { BREAK. }

    SET g TO BODY:MU / (BODY:RADIUS^2).
    SET maxAcc TO SHIP:MAXTHRUST / SHIP:MASS.
    SET twr TO maxAcc / g.
    SET aNet TO MAX(0.5, maxAcc - g).
    PRINT "TWR: " + ROUND(twr, 2) + "   aNet: " + ROUND(aNet, 2) + " м/с2".

    // ================= ЭТАП 1: ВЫБОР МЕСТА =================
    // Вызываем отдельный скрипт find_landing_site.ks
    // Он устанавливает глобальные переменные: landLat, landLng, landTerrain, landSlope, landGeo
    PRINT "=== Этап 1: выбор места посадки ===".
    SET initLat TO landLat.
    SET initLng TO landLng.
    RUN find_landing_site(initLat, initLng, maxSlope).
    // Результат уже в переменных landLat, landLng, landTerrain, landSlope
    WAIT 2.

    // ================= ЭТАП 2: РАСЧЁТ ТОЧКИ ТОРМОЖЕНИЯ =================
    IF NOT alreadyDescending {
        PRINT "=== Этап 2: расчёт точки торможения ===".
        SET brakeInfo TO findBrakingPoint(landLat, landLng, landTerrain).
        SET brakeTrueAnom TO brakeInfo[0].
        SET brakeETA TO brakeInfo[1].
        SET brakeAngle TO brakeInfo[2].

        PRINT "Лучшая точка торможения: истинная аномалия " + ROUND(brakeTrueAnom, 1) + "°".
        PRINT "ETA до точки: " + ROUND(brakeETA) + " с, угол: " + ROUND(brakeAngle, 1) + "°".

        SET posAtBrake TO POSITIONAT(SHIP, TIME:SECONDS + brakeETA).
        SET velAtBrake TO VELOCITYAT(SHIP, TIME:SECONDS + brakeETA):ORBIT.
        SET rHat TO posAtBrake:NORMALIZED.
        SET vHoriz TO velAtBrake - (VDOT(velAtBrake, rHat) * rHat).
        SET dvBrake TO vHoriz:MAG.

        PRINT "Горизонтальная скорость: " + ROUND(vHoriz:MAG, 1) + " м/с, ΔV=" + ROUND(dvBrake, 1) + " м/с".

        SET brakeNode TO NODE(TimeSpan(brakeETA), 0, 0, -dvBrake).
        ADD brakeNode.
        PRINT "Нода создана. ETA: " + ROUND(brakeNode:ETA) + " с, dV: " + ROUND(brakeNode:DELTAV:MAG, 1) + " м/с".

        IF ALLNODES:LENGTH > 0 {
            PRINT "Выполняю манёвр торможения...".
            UNLOCK THROTTLE.
            RUN exec_node.
            LOCK THROTTLE TO throttleSet.
        } ELSE {
            PRINT "ОШИБКА: нода исчезла.".
            SET scriptDone TO TRUE.
        }

        IF scriptDone { BREAK. }
    } ELSE {
        PRINT "=== Этап 2: пропущен (ракета уже снижается) ===".
    }

    // ================= ЭТАП 3: ПОСАДКА (4 ФАЗЫ) =================
    PRINT "=== Этап 3: посадка ===".

    UNLOCK STEERING.
    SET SAS TO TRUE.
    SET throttleSet TO 0.

    PRINT "Ждём начала снижения...".
    WAIT UNTIL SHIP:VERTICALSPEED < -5.

    PRINT "Начинаю посадку: AGL=" +
          ROUND(SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT) + " м, vVert=" +
          ROUND(SHIP:VERTICALSPEED,1) + " м/с, статус=" + SHIP:STATUS.

    IF SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" {
        PRINT "Ракета уже на поверхности.".
        SET scriptDone TO TRUE.
    } ELSE {
        SET SAS TO FALSE.
        SET noThrustSince TO -1.
        SET nextDiagTimer TO 0.
        SET landingPhase TO "horizontal".
        SET lastTwr TO 0.
        SET currentWarp TO 0.
        SET inSuicideBurn TO FALSE.

        UNTIL SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" {
            SET altAGL TO SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT.
            IF altAGL < 0 { SET altAGL TO 0. }

            SET upDir TO SHIP:UP:FOREVECTOR.
            SET vSurf TO SHIP:VELOCITY:SURFACE.
            SET vVert TO VDOT(vSurf, upDir).
            SET vHoriz TO vSurf - (vVert * upDir).
            SET vHorizMag TO vHoriz:MAG.
            SET vVertAbs TO ABS(vVert).

            SET gNow TO safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.01).

            SET thrustNow TO SHIP:AVAILABLETHRUST.
            IF thrustNow <= 0.01 { SET thrustNow TO SHIP:MAXTHRUST. }
            SET massNow TO MAX(0.001, SHIP:MASS).
            SET aThrust TO MAX(0.1, thrustNow / massNow).
            SET aNet TO MAX(0.1, aThrust - gNow).
            SET twrNow TO aThrust / MAX(0.01, gNow).

            IF thrustNow > 0.01 {
                SET noThrustSince TO -1.
            } ELSE {
                IF noThrustSince < 0 { SET noThrustSince TO TIME:SECONDS. }
                IF (TIME:SECONDS - noThrustSince) > 5 {
                    PRINT "ОШИБКА: нет тяги 5 секунд.".
                    SET throttleSet TO 0.
                    BREAK.
                }
            }

            IF ABS(twrNow - lastTwr) > 0.5 {
                PRINT "  [TWR] " + ROUND(lastTwr, 2) + " → " + ROUND(twrNow, 2) +
                      " (масса " + ROUND(massNow) + " кг, тяга " + ROUND(thrustNow) + " кН)".
                SET lastTwr TO twrNow.
            }

            SET horizTol TO MIN(15, baseHorizTol + twrNow * 0.3).

            SET hBrake TO (vVertAbs*vVertAbs + 2*gNow*altAGL - touchdownSpeed*touchdownSpeed) / (2 * aThrust).
            IF hBrake < 0 { SET hBrake TO 0. }
            SET hBrakeSafe TO hBrake * brakeSafetyMargin.

            SET deltaH TO altAGL - hBrake.
            IF deltaH < 0 { SET deltaH TO 0. }
            IF vVertAbs > 1 {
                SET tFall TO deltaH / vVertAbs.
            } ELSE {
                SET tFall TO 9999.
            }

            SET vSafe TO SQRT(2 * aNet * MAX(1, altAGL)).
            SET mustBrake TO (altAGL <= hBrakeSafe) OR (vVertAbs > vSafe AND altAGL > 100).

            // =====================================================================
            //  ФАЗА 1: ГОРИЗОНТАЛЬНОЕ ТОРМОЖЕНИЕ
            //  Целимся строго против vHoriz (горизонтально)
            // =====================================================================
            IF vHorizMag > 5 {
                SET landingPhase TO "horizontal".
                SET inSuicideBurn TO FALSE.
                LOCK THROTTLE TO throttleSet.
                SET SAS TO FALSE.

                IF currentWarp > 0 {
                    SET WARP TO 0.
                    WAIT 0.5.
                    SET currentWarp TO 0.
                }

                IF vHorizMag > 0.5 {
                    SET antiHorizDir TO (-1) * vHoriz:NORMALIZED.
                    LOCK STEERING TO LOOKDIRUP(antiHorizDir, upDir).
                } ELSE {
                    LOCK STEERING TO LOOKDIRUP(upDir, SHIP:FACING:TOPVECTOR).
                }
                SET throttleSet TO 1.

            // =====================================================================
            //  ФАЗА 2: COAST (ПАДЕНИЕ С ПРОМОТКОЙ ВРЕМЕНИ)
            //  Ракета вертикальна, тяга 0%
            // =====================================================================
            } ELSE IF NOT mustBrake AND altAGL > 100 {
                SET landingPhase TO "coast".
                SET inSuicideBurn TO FALSE.
                SET SAS TO FALSE.
                
                // Ракета вертикальна
                LOCK STEERING TO LOOKDIRUP(upDir, SHIP:FACING:TOPVECTOR).
                UNLOCK THROTTLE.
                SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
                SET throttleSet TO 0.

                // Варпим, если до suicide burn далеко
                IF tFall > coastLeadTime AND altAGL > 5000 {
                    SET targetWarp TO 0.
                    IF tFall > coastLeadTime * 8 { SET targetWarp TO 4. }
                    ELSE IF tFall > coastLeadTime * 4 { SET targetWarp TO 3. }
                    ELSE IF tFall > coastLeadTime * 2 { SET targetWarp TO 2. }
                    ELSE { SET targetWarp TO 1. }

                    IF targetWarp <> currentWarp {
                        SET WARP TO targetWarp.
                        SET currentWarp TO targetWarp.
                        WAIT 0.5.
                    }

                    WAIT 1.
                } ELSE {
                    IF currentWarp > 0 {
                        SET WARP TO 0.
                        WAIT 0.5.
                        SET currentWarp TO 0.
                    }
                    LOCK THROTTLE TO throttleSet.
                    WAIT 0.1.
                }

            // =====================================================================
            //  ФАЗА 3: SUICIDE BURN (вызов suicide_burn.ks)
            //  Ракета почти вертикальна с наклоном <= 5°
            // =====================================================================
            } ELSE IF mustBrake AND altAGL > 100 {
                SET landingPhase TO "suicide".
                PRINT "  Переход к suicide burn...".
                
                // Вызываем отдельный скрипт suicide_burn.ks
                // Он использует глобальные переменные и устанавливает:
                //   suicideDone = TRUE когда altAGL <= 100
                //   suicideFailed = TRUE если ракета улетает вверх
                RUN suicide_burn.
                
                // Проверяем результат
                IF suicideFailed {
                    // Ракета улетела вверх — возвращаемся в coast
                    PRINT "  Suicide burn прерван, возвращаюсь в coast".
                    SET landingPhase TO "coast".
                    SET inSuicideBurn TO FALSE.
                    UNLOCK THROTTLE.
                    UNLOCK STEERING.
                    SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
                    SET SAS TO TRUE.
                    SET throttleSet TO 0.
                } ELSE IF suicideDone {
                    // Suicide burn завершён — переходим к финальной посадке
                    SET landingPhase TO "final".
                }

            // =====================================================================
            //  ФАЗА 4: ФИНАЛЬНАЯ ПОСАДКА
            //  altAGL <= 100 м, ракета почти вертикальна
            // =====================================================================
            } ELSE IF altAGL <= 100 {
                SET landingPhase TO "final".
                SET inSuicideBurn TO FALSE.
                LOCK THROTTLE TO throttleSet.
                SET SAS TO FALSE.

                IF currentWarp > 0 {
                    SET WARP TO 0.
                    WAIT 0.5.
                    SET currentWarp TO 0.
                }

                IF twrNow < 1.1 {
                    LOCK STEERING TO LOOKDIRUP(upDir, SHIP:FACING:TOPVECTOR).
                    SET throttleSet TO 0.
                } ELSE {
                    // Целевая скорость: от -40 м/с на 100 м до -touchdownSpeed на 0 м
                    SET vTarget TO -touchdownSpeed - (altAGL / 100) * (40 - touchdownSpeed).
                    SET vTargetAbs TO ABS(vTarget).

                    // Направление: вертикально с наклоном <= 5° против vHoriz
                    SET maxTiltRad TO maxTiltDeg * CONSTANT:PI / 180.
                    IF vHorizMag > 0.5 {
                        SET antiHorizDir TO (-1) * vHoriz:NORMALIZED.
                        SET tiltFactor TO MIN(maxTiltRad, vHorizMag / 30).
                        SET targetDir TO (COS(tiltFactor) * upDir + SIN(tiltFactor) * antiHorizDir):NORMALIZED.
                    } ELSE {
                        SET targetDir TO upDir.
                    }
                    LOCK STEERING TO LOOKDIRUP(targetDir, upDir).

                    // Тяга по формуле
                    SET aNeeded TO (vVertAbs*vVertAbs - vTargetAbs*vTargetAbs) / (2 * MAX(1, altAGL)) + gNow.
                    SET minThrottle TO safeDiv(gNow, aThrust, 0.1).
                    SET throttleSet TO clamp(aNeeded / MAX(0.1, aThrust), minThrottle, 1).
                }
            }

            SET throttleSet TO clamp(throttleSet, 0, 1).

            // Диагностика каждые 2 секунды
            IF TIME:SECONDS >= nextDiagTimer {
                SET shipHeading TO SHIP:FACING:YAW.
                SET shipPitch TO SHIP:FACING:PITCH.
                
                IF landingPhase = "coast" {
                    SET actualWarp TO WARP.
                    PRINT "  Фаза: " + landingPhase +
                          " | vVert=" + ROUND(vVert, 1) +
                          " | vHoriz=" + ROUND(vHorizMag, 1) +
                          " | altAGL=" + ROUND(altAGL) +
                          " | hBrake=" + ROUND(hBrake) +
                          " | vSafe=" + ROUND(vSafe, 1) +
                          " | tFall=" + ROUND(tFall) + "с" +
                          " | warp=" + currentWarp + "x" +
                          " (real=" + actualWarp + "x)" +
                          " | hdg=" + ROUND(shipHeading, 0) + "°" +
                          " | pit=" + ROUND(shipPitch, 0) + "°" +
                          " | thr=0%".
                } ELSE {
                    PRINT "  Фаза: " + landingPhase +
                          " | vVert=" + ROUND(vVert, 1) +
                          " | vHoriz=" + ROUND(vHorizMag, 1) +
                          " | tol=" + ROUND(horizTol, 1) +
                          " | altAGL=" + ROUND(altAGL) +
                          " | hBrake=" + ROUND(hBrake) +
                          " | vSafe=" + ROUND(vSafe, 1) +
                          " | TWR=" + ROUND(twrNow, 2) +
                          " | hdg=" + ROUND(shipHeading, 0) + "°" +
                          " | pit=" + ROUND(shipPitch, 0) + "°" +
                          " | thr=" + ROUND(throttleSet * 100) + "%".
                }
                SET nextDiagTimer TO TIME:SECONDS + 2.
            }

            WAIT 0.
        }
    }

    SET scriptDone TO TRUE.
}

SET throttleSet TO 0.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
UNLOCK THROTTLE.
UNLOCK STEERING.
IF currentWarp > 0 { SET WARP TO 0. }
SET SAS TO TRUE.
PRINT "=== ПОСАДКА ЗАВЕРШЕНА ===".
PRINT "Статус: " + SHIP:STATUS.
PRINT "Скорость касания: " + ROUND(SHIP:VELOCITY:SURFACE:MAG, 2) + " м/с".
PRINT "Позиция: " + ROUND(SHIP:GEOPOSITION:LAT,5) + ", " + ROUND(SHIP:GEOPOSITION:LNG,5).