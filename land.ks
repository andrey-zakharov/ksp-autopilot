//@LAZYGLOBAL OFF.

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
PARAMETER horizDecelTarget IS 5.0.   // Целевое замедление горизонтали (м/с^2)
PARAMETER horizKp IS 0.8.            // Коэффициент P для регулятора
PARAMETER horizKd IS 0.2.            // Коэффициент D для регулятора

// Параметры SUICIDE
PARAMETER suicideTiltDeg IS 8.      // Макс. наклон в суициде
PARAMETER brakeSafetyMargin IS 1.1. // Запас высоты начала суицида
PARAMETER brakeLeadTime IS 0.5.     // Время реакции/разворота
PARAMETER terrainMargin IS 5.       
PARAMETER groundClearance IS 3.      // Запас высоты для завершения суицида (метры)

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

// Переменные для PD-регулятора горизонтали
SET prevHorizErr TO 0.
SET currentHorizAccel TO 0.

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

FUNCTION checkStage {
    // TBD
}

function printState {
    // Диагностика каждые 2 секунды
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

LOCK isLanded TO SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED".

UNTIL isLanded OR
      SHIP:ORBIT:PERIAPSIS <= 0 OR
      orbHorizMag < 5 OR
      (altAGL < 10000 AND velVert < -5) {

    flightState().
    checkStage().

    UNTIL ALLNODES:LENGTH = 0 {
        REMOVE ALLNODES[0].
    }
    WAIT 0.

    SET nodeEta TO 30. // TBD calc time to rotate rocket

    IF siteSource = "AUTO" AND autoLeadSec > 0 {
        SET nodeEta TO MAX(10, autoLeadSec - autoBrakeSec / 2).
    } ELSE {
        SET brakeInfo TO findBrakingPoint(siteLat, siteLng, siteTerrain).
        SET nodeEta TO brakeInfo[0].
    }

    SET nodeVel TO VELOCITYAT(SHIP, TIME:SECONDS + nodeEta):ORBIT.
    SET nodeDv TO nodeVel:MAG.

    PRINT "Деорбит: ETA=" + ROUND(nodeEta) + " dV=" + ROUND(nodeDv,1).

    IF nodeDv >= 1 {
        SET maneuverNode TO NODE(TimeSpan(nodeEta), 0, 0, -nodeDv).
        ADD maneuverNode.
        WAIT 1.

        UNLOCK THROTTLE.
        UNLOCK STEERING.

        RUN exec_node.

        SET SAS TO FALSE.
        SET steerFacing TO SHIP:FACING.
        SET prevFaceVec TO steerFacing:FOREVECTOR.

        LOCK THROTTLE TO throttleCmd.
        LOCK STEERING TO steerFacing.

        WAIT 0.5.
    }
}

UNTIL isLanded {

    flightState().

    checkStage().

    // ==========================================================
    //  HORIZONTAL: Активное гашение горизонтали с PD-регулятором
    // ==========================================================
    UNTIL isLanded OR
          velHorizMag <= horizTol OR
          (velVert < 0 AND altAGL <= brakeAlt) OR
          altAGL <= finalAlt {

        flightState().
        // 1. Расчет максимального допустимого угла наклона
        // Чтобы не улететь вверх при 100% газе, вертикальная составляющая тяги должна быть <= g.
        // T_acc * cos(theta) <= g  =>  theta_max = arccos(g / T_acc)
        
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

        // 2. PD-Регулятор тяги
        // Цель: поддерживать постоянное замедление horizDecelTarget, пока скорость велика.
        // По мере снижения скорости, целевое замедление может уменьшаться, но лучше держать постоянным до самого конца,
        // чтобы избежать "дрожания" газа. 
        
        // Текущая горизонтальная скорость как мера прогресса
        // Если скорость большая, хотим тормозить интенсивно.
        // Используем простую логику: Target Accel = Constant Deceleration Rate.
        // Но чтобы не перелететь через ноль, масштабируем целевое ускорение от текущей скорости.
        
        // Чем ближе к нулю, тем меньше должно быть ускорение, иначе будет overshoot.
        // Однако PD регулятор сам справится с этим, если задать правильный набор коэффициентов.
        // Здесь зададим целевое ускорение равным константе, пока скорость > threshold, 
        // и линейно падающим до 0 при достижении порога.
        
        LOCAL targetDecel TO horizDecelTarget.
        
        // Смягчение вблизи нуля
        IF velHorizMag < 2.0 {
             SET targetDecel TO horizDecelTarget * (velHorizMag / 2.0).
        }

        // Измеряем текущее горизонтальное ускорение (производная скорости)
        // В kOS нет прямого доступа к acc, поэтому оцениваем через изменение скорости за тик?
        // Нет, лучше использовать разницу скоростей между кадрами.
        // Но у нас есть только текущая скорость. 
        // Упрощенный подход: считаем, что текущее ускорение равно тому, что было задано в прошлом кадре?
        // Или используем P-only контроллер для начала, так как D требует точных данных об ускорении.
        
        // Давайте используем гибридный подход:
        // Требуемая тяга для создания targetDecel:
        // F_thrust_horizontal_component = Mass * targetDecel
        // Throttle = (Mass * targetDecel) / (MaxThrust * Sin(Tilt)) ... грубо
        
        // Более надежный способ для kOS:
        // Рассчитать необходимую долю тяги для достижения целевого замедления.
        // a_needed = targetDecel
        // T_max_horiz = ThrustAcc * Sin(tiltRad)
        // Ratio = a_needed / T_max_horiz
        
        LOCAL maxHorizAccel TO thrustAcc * SIN(tiltRad).
        LOCAL requiredRatio TO 0.
        
        IF maxHorizAccel > 0.01 {
            SET requiredRatio TO clamp(targetDecel / maxHorizAccel, 0, 1).
        } ELSE {
            SET requiredRatio TO 0.
        }
        
        // Применяем демпфирование (D-часть) основанное на изменении ошибки скорости
        // Ошибка скорости = Current Speed - Target Speed (который стремится к 0)
        // Но проще: следим за тем, чтобы скорость не уходила в отрицательную зону (перелет).
        // Поскольку мы целенаправленно гасим положительную скорость, перелет невозможен физически,
        // если тяга не превышает необходимость. 
        
        // Главный источник рывков — это резкое включение/выключение или скачок угла.
        // Ограничим скорость изменения тяги (slew rate limit).
        
        LOCAL deltaThr TO requiredRatio - throttleCmd.
        LOCAL maxDelta TO 0.1 * dtSec * 50. // Максимум 5 единиц тяги в секунду (приблизительно)
        
        IF deltaThr > maxDelta { SET deltaThr TO maxDelta. }
        IF deltaThr < -maxDelta { SET deltaThr TO -maxDelta. }
        
        SET throttleCmd TO throttleCmd + deltaThr.
        SET throttleCmd TO clamp(throttleCmd, 0, 1).

        // Гейт ориентации
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
    //  Двигатель выключен, ракета вертикально, варп только безопасно.
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

        // Передаем groundClearance в suicide_burn
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
    //  Строго вертикально, мягкая посадка.
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