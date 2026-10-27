// ==========================================================================
// land.ks — посадка на безатмосферное тело (kOS 1.6)
// ==========================================================================
PARAMETER landLat IS 0.
PARAMETER landLng IS 0.
PARAMETER maxSlope IS 5.

PARAMETER touchdownSpeed IS 0.      
PARAMETER finalAlt IS 100.          
PARAMETER finalApproachSpeed IS 10. 

// Параметры фазы HORIZONTAL
PARAMETER horizTol IS 1.           // Порог выхода из фазы (м/с)

// Параметры SUICIDE
PARAMETER suicideTiltDeg IS 8.      
PARAMETER brakeSafetyMargin IS 1.1. 
PARAMETER brakeLeadTime IS 0.5.     
PARAMETER terrainMargin IS 5.       
PARAMETER groundClearance IS 3.     // Запас высоты для завершения суицида (метры)

// Общие параметры
PARAMETER maxSlewDeg IS 10.         
PARAMETER maxIgniteTiltDeg IS 15.   
PARAMETER coastLeadTime IS 20.      
PARAMETER maxWarpAim IS 5.          
PARAMETER maxWarpRate IS 2.         

RUN utils.

// ==========================================================================
//  СОСТОЯНИЕ
// ==========================================================================
SET throttleCmd TO 0.
SET steerFacing TO SHIP:FACING.
SET currentWarp TO 0.

SET lastSec TO TIME:SECONDS.
SET dtSec TO 0.02.

SET altAGL TO 0.
SET upVector TO SHIP:UP:FOREVECTOR.
SET velSurf TO SHIP:VELOCITY:SURFACE.
SET velVert TO 0.
SET velHoriz TO V(0,0,0).
SET velHorizMag TO 0.
SET velVertAbs TO 0.
SET orbHorizMag TO 0.

SET gravAcc TO 0.001.
SET thrustAvail TO 0.
SET shipMass TO 1.
SET thrustAcc TO 0.01.
SET netAcc TO 0.05.
SET twrNow TO 0.

SET brakeAlt TO -1.
SET timeToBrake TO 9999.
SET faceRate TO 0.
SET prevFaceVec TO SHIP:FACING:FOREVECTOR.
SET noThrustSince TO 0.

SET siteLat TO landLat.
SET siteLng TO landLng.
SET siteTerrain TO 0.
SET siteSource TO "MANUAL".
SET autoLeadSec TO 0.
SET autoBrakeSec TO 0.
SET nextDiagTimer TO 0.

// ==========================================================================
//  ФУНКЦИИ РАСЧЁТА
// ==========================================================================

FUNCTION surfaceWorldVec { PARAMETER latDeg. PARAMETER lngDeg. PARAMETER terrainM. PARAMETER futureSec.
    LOCAL rotRate TO 360 / MAX(1, BODY:ROTATIONPERIOD).
    LOCAL totalLng TO lngDeg + BODY:ROTATIONANGLE + rotRate * futureSec.
    LOCAL radVal TO BODY:RADIUS + terrainM.

    RETURN V(
        radVal * COS(latDeg) * COS(totalLng),
        radVal * SIN(latDeg),
        radVal * COS(latDeg) * SIN(totalLng)
    ).
}

FUNCTION timeToFall { PARAMETER altNow. PARAMETER altEnd. PARAMETER downSpeed. PARAMETER grav.
    LOCAL distM TO MAX(0, altNow - altEnd).
    IF distM <= 0 { RETURN 0. }
    LOCAL gSafe TO MAX(0.0001, grav).
    LOCAL vSafe TO MAX(0, downSpeed).
    RETURN (SQRT(vSafe * vSafe + 2 * gSafe * distM) - vSafe) / gSafe.
}

FUNCTION flightState {
    LOCAL nowSec TO TIME:SECONDS.
    LOCAL rawDt TO nowSec - lastSec.

    SET lastSec TO nowSec.
    SET dtSec TO clamp(rawDt, 0.01, 0.25).

    SET altAGL TO MAX(0, SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT).
    SET upVector TO SHIP:UP:FOREVECTOR.

    SET velSurf TO SHIP:VELOCITY:SURFACE.
    SET velVert TO VDOT(velSurf, upVector).
    SET velHoriz TO velSurf - (velVert * upVector).
    SET velHorizMag TO velHoriz:MAG.
    SET velVertAbs TO ABS(velVert).

    LOCAL orbVelVec TO SHIP:VELOCITY:ORBIT.
    LOCAL orbHorizVec TO orbVelVec - (VDOT(orbVelVec, upVector) * upVector).
    SET orbHorizMag TO orbHorizVec:MAG.

    SET gravAcc TO safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.001).

    SET thrustAvail TO SHIP:AVAILABLETHRUST.
    IF thrustAvail <= 0.01 { SET thrustAvail TO SHIP:MAXTHRUST. }

    SET shipMass TO MAX(0.001, SHIP:MASS).
    SET thrustAcc TO MAX(0.01, thrustAvail / shipMass).
    SET twrNow TO thrustAcc / MAX(0.001, gravAcc).
    SET netAcc TO MAX(0.05, thrustAcc - gravAcc).

    SET faceRate TO VANG(SHIP:FACING:FOREVECTOR, prevFaceVec) / MAX(0.05, rawDt).
    SET prevFaceVec TO SHIP:FACING:FOREVECTOR.

    SET brakeAlt TO -1.
    SET timeToBrake TO 9999.

    IF velVert < 0 {
        LOCAL brakeDist TO (velVertAbs * velVertAbs - touchdownSpeed * touchdownSpeed) / (2 * netAcc).
        IF brakeDist < 0 { SET brakeDist TO 0. }
        SET brakeAlt TO brakeDist * brakeSafetyMargin + velVertAbs * brakeLeadTime + terrainMargin.
        SET timeToBrake TO timeToFall(altAGL, brakeAlt, velVertAbs, gravAcc).
    }

    IF thrustAvail <= 0.01 {
        IF noThrustSince = 0 { SET noThrustSince TO nowSec. }
    } ELSE {
        SET noThrustSince TO 0.
    }
}

FUNCTION findBrakingPoint { PARAMETER targetLat. PARAMETER targetLng. PARAMETER targetTerrain.
    LOCAL bestEta TO 10.
    LOCAL bestDv TO 0.
    LOCAL bestErr TO 360.
    LOCAL scanEta TO 10.
    LOCAL periodSec TO MAX(60, SHIP:ORBIT:PERIOD).

    UNTIL scanEta > periodSec {
        LOCAL futurePos TO POSITIONAT(SHIP, TIME:SECONDS + scanEta).
        LOCAL futureVel TO VELOCITYAT(SHIP, TIME:SECONDS + scanEta):ORBIT.
        LOCAL futureUp TO futurePos:NORMALIZED.
        LOCAL futureHoriz TO futureVel - (VDOT(futureVel, futureUp) * futureUp).
        LOCAL futureHorizMag TO futureHoriz:MAG.

        IF futureHorizMag > 0.1 {
            LOCAL sitePos TO surfaceWorldVec(targetLat, targetLng, targetTerrain, scanEta).
            LOCAL toSite TO sitePos - futurePos.
            LOCAL errDeg TO VANG(futureHoriz, toSite).

            IF errDeg <= 8 {
                RETURN List(scanEta, futureHorizMag, errDeg).
            }

            IF errDeg < bestErr {
                SET bestErr TO errDeg.
                SET bestEta TO scanEta.
                SET bestDv TO futureHorizMag.
            } ELSE IF scanEta > bestEta + 30 AND errDeg > bestErr + 5 {
                SET scanEta TO periodSec + 1.
            }
        }
        SET scanEta TO scanEta + 10.
    }
    RETURN List(bestEta, bestDv, bestErr).
}

function printState {
    IF TIME:SECONDS < nextDiagTimer {
        return.
    }
    SET shipHeading TO SHIP:FACING:YAW.
    SET shipPitch TO SHIP:FACING:PITCH.
    PRINT
    " | vV=" + ROUND(velVert, 1) +
    " | vH=" + ROUND(velHorizMag, 1) +
    " | tol=" + ROUND(horizTol, 1) +
    " | AGL=" + ROUND(altAGL) +
    " | bAlt=" + ROUND(brakeAlt) +
    " | ttb=" + ROUND(timeToBrake, 1) +
    " | TWR=" + ROUND(twrNow, 2) +
    " | hdg=" + ROUND(shipHeading, 0) + "°" +
    " | pit=" + ROUND(shipPitch, 0) + "°" +
    " | thr=" + ROUND(throttleCmd * 100) + "%".
    SET nextDiagTimer TO TIME:SECONDS + 2.
}

// ==========================================================================
//  СТАРТ
// ==========================================================================
CLEARSCREEN.
SET SAS TO FALSE.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.

SET throttleCmd TO 0.
SET steerFacing TO SHIP:FACING.

LOCK THROTTLE TO throttleCmd.
LOCK STEERING TO steerFacing.

flightState().

PRINT "=== ПОСАДКА ===".
PRINT "Статус: " + SHIP:STATUS +
      " AGL=" + ROUND(altAGL) +
      " vV=" + ROUND(velVert,1) +
      " vH=" + ROUND(velHorizMag,1) +
      " pit=" + ROUND(SHIP:FACING:PITCH,1).

IF SHIP:ORBIT:ECCENTRICITY >= 1 AND SHIP:ORBIT:PERIAPSIS > 0 {
    PRINT "ОШИБКА: гиперболический пролёт. Нужна тормозная нода до запуска.".
    EXIT.
}

RUN find_landing_site(landLat, landLng, maxSlope).

SET siteLat TO landingLat.
SET siteLng TO landingLng.
SET siteTerrain TO landingTerrain.

PRINT "Цель: " + ROUND(siteLat,5) + "," + ROUND(siteLng,5) +
      " рельеф=" + ROUND(siteTerrain).

WAIT 1.

// ==========================================================================
//  ЭТАП ДЕОРБИТА
// ==========================================================================
LOCK isLanded TO SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED".

UNTIL isLanded OR
      SHIP:ORBIT:PERIAPSIS <= 0 OR
      orbHorizMag < 5 OR
      (altAGL < 10000 AND velVert < -5) {

    flightState().
    
    // Удаляем старые ноды
    UNTIL ALLNODES:LENGTH = 0 { REMOVE ALLNODES[0]. }
    WAIT 0.

    // Простая логика: гасим ВСЮ орбитальную скорость ретроградно.
    // Время задержки берем достаточное для разворота ракеты (TWR высокий -> разворот быстрый).
    // 10 секунд обычно достаточно даже для крупных кораблей с низким TWR.
    SET nodeEta TO 10.
    
    // Если есть авто-лидирование из find_landing_site, можно использовать его, 
    // но ограничим сверху, чтобы не ждать вечность.
    IF siteSource = "AUTO" AND autoLeadSec > 0 {
        SET nodeEta TO CLAMP(autoLeadSec, 10, 60).
    }

    // Текущая орбитальная скорость
    SET nodeVel TO SHIP:VELOCITY:ORBIT.
    SET nodeDv TO nodeVel:MAG.

    PRINT "Деорбит: ETA=" + ROUND(nodeEta) + " с, dV=" + ROUND(nodeDv, 1) + " м/с".

    IF nodeDv >= 1 {
        // NODE(time, radial, normal, prograde)
        // prograde = -dV => торможение против движения
        SET maneuverNode TO NODE(TimeSpan(nodeEta), 0, 0, -nodeDv).
        
        // Сразу добавляем ноду в план полета.
        // До ADD нельзя обращаться к свойствам вроде DELTAV.
        ADD maneuverNode.
        WAIT 0.2.

        UNLOCK THROTTLE.
        UNLOCK STEERING.

        RUN exec_node.

        SET SAS TO FALSE.
        SET steerFacing TO SHIP:FACING.
        SET prevFaceVec TO steerFacing:FOREVECTOR.

        LOCK THROTTLE TO throttleCmd.
        LOCK STEERING TO steerFacing.
        
        WAIT 1.
    } ELSE {
        PRINT "Горизонтальная скорость мала (<1 м/с). Деорбит не нужен.".
        BREAK.
    }
    
    flightState().
}
UNTIL isLanded {
    flightState().

    // ==========================================================
    //  HORIZONTAL: Активное гашение горизонтали
    // ==========================================================
    UNTIL isLanded OR
          velHorizMag <= horizTol OR
          (velVert < 0 AND altAGL <= brakeAlt) OR
          altAGL <= finalAlt {

        flightState().

        // 1. Расчет максимального допустимого угла наклона
        LOCAL tiltRad TO 0.
        
        IF thrustAcc > gravAcc {
            SET tiltRad TO ARCCOS(clamp(gravAcc / thrustAcc, 0, 1)).
        } ELSE {
            SET tiltRad TO 0.
        }

        // Строим целевой вектор
        IF velHorizMag > 0.1 {
            SET antiHoriz TO (-1) * velHoriz:NORMALIZED.
            SET desiredDir TO (COS(tiltRad) * upVector + SIN(tiltRad) * antiHoriz):NORMALIZED.
        } ELSE {
            SET desiredDir TO upVector.
        }

        // 2. Управление тягой с предсказанием
        LOCAL baseThr TO 1.0.
        LOCAL horizDecelMax TO thrustAcc * SIN(tiltRad).
        LOCAL predictedDrop TO horizDecelMax * dtSec.
        
        IF predictedDrop > velHorizMag {
            LOCAL reqAccel TO velHorizMag / MAX(0.001, dtSec).
            SET baseThr TO clamp(reqAccel / MAX(0.01, horizDecelMax), 0, 1).
        }

        SET throttleCmd TO baseThr.

        // Применяем гейт по ошибке ориентации
        SET throttleCmd TO thrustGate(throttleCmd, desiredDir, maxIgniteTiltDeg).
        SET throttleCmd TO clamp(throttleCmd, 0, 1).

        // Плавный разворот
        SET steerFacing TO rotateToward(steerFacing, desiredDir, maxSlewDeg, dtSec).

        IF currentWarp > 0 {
            SET WARP TO 0.
            SET currentWarp TO 0.
            WAIT 0.2.
        }
        printState().
    }

    // ==========================================================
    //  COAST / FREEFALL
    // ==========================================================
    UNTIL isLanded OR
          (velVert < 0 AND altAGL <= brakeAlt) OR
          altAGL <= finalAlt {

        flightState().

        SET desiredDir TO upVector.
        SET throttleCmd TO 0.

        SET steerFacing TO rotateToward(steerFacing, desiredDir, maxSlewDeg, dtSec).

        SET aimNow TO VANG(SHIP:FACING:FOREVECTOR, upVector).
        SET wantWarp TO 0.

        IF throttleCmd < 0.01 AND
           velVert < -1 AND
           aimNow <= maxWarpAim AND
           faceRate <= maxWarpRate AND
           altAGL > MAX(3000, finalAlt * 30) AND
           timeToBrake > coastLeadTime * 2 {

            IF timeToBrake > coastLeadTime * 10 {
                SET wantWarp TO 4.
            } ELSE IF timeToBrake > coastLeadTime * 5 {
                SET wantWarp TO 3.
            } ELSE IF timeToBrake > coastLeadTime * 3 {
                SET wantWarp TO 2.
            } ELSE {
                SET wantWarp TO 1.
            }
        }

        IF brakeAlt > 0 AND altAGL <= brakeAlt * 1.5 {
            SET wantWarp TO 0.
        }

        IF wantWarp <> currentWarp {
            SET WARP TO wantWarp.
            SET currentWarp TO wantWarp.
            WAIT 0.2.
        }
    }

    // ==========================================================
    //  SUICIDE BURN
    // ==========================================================
    IF not(isLanded) AND
       velVert < 0 AND
       altAGL > finalAlt AND
       altAGL <= brakeAlt {

        IF currentWarp > 0 {
            SET WARP TO 0.
            SET currentWarp TO 0.
            WAIT 0.2.
        }

        SET GEAR TO TRUE.

        PRINT "Suicide: alt=" + ROUND(altAGL) +
              " vV=" + ROUND(velVert,1) +
              " vH=" + ROUND(velHorizMag,1).

        UNLOCK STEERING.
        UNLOCK THROTTLE.

        // Передаем все необходимые параметры
        RUN suicide_burn(
            finalAlt,
            finalApproachSpeed,
            suicideTiltDeg,
            touchdownSpeed,
            maxIgniteTiltDeg,
            maxSlewDeg,
            groundClearance
        ).

        SET SAS TO FALSE.
        SET steerFacing TO SHIP:FACING.
        SET prevFaceVec TO steerFacing:FOREVECTOR.
        SET throttleCmd TO 0.

        LOCK THROTTLE TO throttleCmd.
        LOCK STEERING TO steerFacing.
    }

    // ==========================================================
    //  FINAL
    // ==========================================================
    UNTIL isLanded OR altAGL > finalAlt {
        flightState().

        SET GEAR TO TRUE.
        SET desiredDir TO upVector.

        IF twrNow < 1.05 {
            SET throttleCmd TO 0.
        } ELSE {
            SET targetMag TO touchdownSpeed +
                (altAGL / MAX(1, finalAlt)) * (finalApproachSpeed - touchdownSpeed).

            SET targetVel TO -targetMag.
            SET netCmd TO 1.5 * (targetVel - velVert).

            IF velVert < targetVel {
                SET energyNet TO (velVertAbs * velVertAbs - targetMag * targetMag) /
                                 (2 * MAX(1, altAGL)).
                SET netCmd TO MAX(netCmd, energyNet).
            }

            SET thrustVertAcc TO gravAcc + netCmd.

            IF velVert > 0 AND altAGL < 30 {
                SET thrustVertAcc TO MIN(thrustVertAcc, gravAcc * 0.35).
            }

            IF thrustVertAcc < 0 {
                SET thrustVertAcc TO 0.
            }

            SET throttleCmd TO clamp(thrustVertAcc / MAX(0.1, thrustAcc), 0, 1).
        }

        SET throttleCmd TO thrustGate(throttleCmd, desiredDir, maxIgniteTiltDeg).
        SET throttleCmd TO clamp(throttleCmd, 0, 1).

        SET steerFacing TO rotateToward(steerFacing, desiredDir, maxSlewDeg, dtSec).

        IF currentWarp > 0 {
            SET WARP TO 0.
            SET currentWarp TO 0.
            WAIT 0.2.
        }
    }
}

SET throttleCmd TO 0.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
UNLOCK THROTTLE.
UNLOCK STEERING.
SET WARP TO 0.
SET SAS TO TRUE.
SET GEAR TO TRUE.
PRINT "=== ПОСАДКА ЗАВЕРШЕНА ===".
PRINT "Статус: " + SHIP:STATUS.
PRINT "Позиция: " + ROUND(SHIP:GEOPOSITION:LAT,5) + ", " + ROUND(SHIP:GEOPOSITION:LNG,5).
PRINT "Скорость: " + ROUND(SHIP:VELOCITY:SURFACE:MAG,2) + " м/с".