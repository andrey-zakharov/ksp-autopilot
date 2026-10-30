// ==========================================================================
// land.ks — Главный скрипт посадки
// ==========================================================================

PARAMETER landLat IS 0.
PARAMETER landLng IS 0.
PARAMETER maxSlope IS 5.

// Параметры фаз
PARAMETER touchdownSpeed IS 0.
PARAMETER finalAlt IS 100.
PARAMETER finalApproachSpeed IS 10.
PARAMETER horizStartAlt IS 20000.
PARAMETER brakeSafetyMargin IS 1.25.
PARAMETER brakeLeadTime IS 1.0.
PARAMETER terrainMargin IS 5.
PARAMETER groundClearance IS 30. 
PARAMETER maxIgniteTiltDeg IS 15.
PARAMETER suicideTiltDeg IS 8.

// Общие параметры
PARAMETER maxSlewDeg IS 10.      
PARAMETER coastLeadTime IS 20.
PARAMETER maxWarpAim IS 6.
PARAMETER maxWarpRate IS 2.


RUN utils.

// ==========================================================================
// ГЛОБАЛЬНОЕ СОСТОЯНИЕ ДЛЯ ОБМЕНА МЕЖДУ СКРИПТАМИ
// ==========================================================================
SET throttleCmd TO 0.
SET steerFacing TO SHIP:FACING.
SET currentWarp TO 0.
SET lastGlobalSec TO TIME:SECONDS. 

// Переменные состояния полета
SET altAGL TO 0.
SET velVert TO 0.
SET velHorizMag TO 0.
SET gravAcc TO 0.001.
SET thrustAcc TO 0.01.
SET twrNow TO 1.
SET brakeAlt TO -1.
SET timeToBrake TO 9999.
SET faceRate TO 0.
SET prevFaceVec TO SHIP:FACING:FOREVECTOR.

// Координаты площадки
SET siteLat TO landLat.
SET siteLng TO landLng.
SET siteTerrain TO 0.
SET nextDiagTimer TO 0.

// ==========================================================================
// ФУНКЦИИ МОНИТОРИНГА
// ==========================================================================

FUNCTION flightState {
    LOCAL nowSec TO TIME:SECONDS.
    LOCAL rawDt TO nowSec - lastGlobalSec.
    
    SET altAGL TO MAX(0, SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT).
    SET upVector TO SHIP:UP:FOREVECTOR.
    
    SET velSurf TO SHIP:VELOCITY:SURFACE.
    SET velVert TO VDOT(velSurf, upVector).
    SET velHoriz TO velSurf - (velVert * upVector).
    SET velHorizMag TO velHoriz:MAG.
    SET velVertAbs TO ABS(velVert).

    SET gravAcc TO safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.001).
    
    SET thrustAvail TO SHIP:AVAILABLETHRUST.
    IF thrustAvail <= 0.01 { SET thrustAvail TO SHIP:MAXTHRUST. }
    
    SET shipMass TO MAX(0.001, SHIP:MASS).
    SET thrustAcc TO MAX(0.01, thrustAvail / shipMass).
    SET twrNow TO thrustAcc / MAX(0.001, gravAcc).
    
    SET faceRate TO VANG(SHIP:FACING:FOREVECTOR, prevFaceVec) / MAX(0.05, rawDt).
    SET prevFaceVec TO SHIP:FACING:FOREVECTOR.

    SET brakeAlt TO -1.
    SET timeToBrake TO 9999.

    // Дерейтим тягу: реальное торможение может быть хуже идеального расчета.
    // Особенно важно при TWR около 2.0-2.5.
    LOCAL netAcc IS MAX(0.05, thrustAcc * 0.90 - gravAcc).

    IF velVert < 0 {
        LOCAL brakeDist IS (velVertAbs * velVertAbs - touchdownSpeed * touchdownSpeed) /
                           (2 * netAcc).
        IF brakeDist < 0 {
            SET brakeDist TO 0.
        }

        // Обязательно добавляем groundClearance, потому что suicide_burn тормозит не до нуля,
        // а до высоты clearance.
        SET brakeAlt TO brakeDist * brakeSafetyMargin +
                        velVertAbs * brakeLeadTime +
                        terrainMargin +
                        groundClearance.

        SET timeToBrake TO (SQRT(velVertAbs * velVertAbs + 2 * gravAcc * MAX(0, altAGL - brakeAlt)) - velVertAbs) / MAX(0.0001, gravAcc).
    }
}

FUNCTION printDiag {
    PRINT "| AGL:" + ROUND(altAGL) + 
          " vV:" + ROUND(velVert,1) + 
          " vH:" + ROUND(velHorizMag,1) + 
          " TWR:" + ROUND(twrNow,2) + 
          " Thr:" + ROUND(throttleCmd*100) + "%" +
          " Warp:" + currentWarp.
}

// ==========================================================================
// СТАРТ
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
PRINT "Статус: " + SHIP:STATUS + " AGL=" + ROUND(altAGL).

// 1. Площадка: если есть цель — она уже размещена игроком, поиск не нужен
IF HASTARGET {
    SET siteLat TO TARGET:GEOPOSITION:LAT.
    SET siteLng TO TARGET:GEOPOSITION:LNG.
    SET siteTerrain TO BODY:GEOPOSITIONLATLNG(siteLat, siteLng):TERRAINHEIGHT.
    PRINT "Цель: " + TARGET:NAME + " " + ROUND(siteLat,5) + "," + ROUND(siteLng,5).
} ELSE {
    RUN find_landing_site(landLat, landLng, maxSlope).
    SET siteLat TO landingLat.
    SET siteLng TO landingLng.
    SET siteTerrain TO landingTerrain.
    PRINT "Площадка: " + ROUND(siteLat,5) + "," + ROUND(siteLng,5).
}
PRINT "Рельеф цели: " + ROUND(siteTerrain) + " м".
WAIT 1.

LOCK isLanded TO SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED".

// 2. Деорбит (если нужны)
IF SHIP:ORBIT:PERIAPSIS > 0 AND SHIP:VELOCITY:ORBIT:MAG > 5 {
    //RUN deorbit_phase(horizStartAlt).
    RUN deorbit_phase(horizStartAlt, 100, 48, 8, 1, 1.0, 3000).
    
    UNLOCK THROTTLE.
    UNLOCK STEERING.
    SET SAS TO FALSE.
    SET steerFacing TO SHIP:FACING.
    SET prevFaceVec TO steerFacing:FOREVECTOR.
    LOCK THROTTLE TO throttleCmd.
    LOCK STEERING TO steerFacing.
    WAIT 1.
}

// 3. Основной цикл посадки
UNTIL isLanded {
    
    flightState().
    
    // Синхронизируем время для суб-скриптов
    SET lastGlobalSec TO TIME:SECONDS. 
    
    // --- Фаза HORIZONTAL ---
    // Пока есть значимая горизонталь и мы высоко
    UNTIL isLanded OR
        velHorizMag <= 1.0 OR
        altAGL > horizStartAlt OR
        (velVert < 0 AND altAGL <= brakeAlt) OR
        altAGL <= finalAlt {
        
        flightState().
        RUN horizontal_brake. 
        
        IF currentWarp > 0 {
            SET WARP TO 0.
            SET currentWarp TO 0.
            WAIT 0.2.
        }
        
        // Диагностика
        IF TIME:SECONDS > nextDiagTimer {
            printDiag().
            SET nextDiagTimer TO TIME:SECONDS + 2.
        }
        
        WAIT 0.
    }

    // --- Фаза COAST ---
    UNTIL isLanded OR
          (velVert < 0 AND altAGL <= brakeAlt) OR
          altAGL <= finalAlt OR
          (velHorizMag > 1.0 AND altAGL <= horizStartAlt) {
        
        flightState().
        SET throttleCmd TO 0.
        // В Coast ракета всегда стремится к вертикали плавно
        SET steerFacing TO rotateToward(steerFacing, SHIP:UP:FOREVECTOR, maxSlewDeg, 0.1).
        LOCK STEERING TO steerFacing.
        
        LOCAL aimErr TO VANG(SHIP:FACING:FOREVECTOR, SHIP:UP:FOREVECTOR).
        LOCAL wantWarp TO 0.
        
        IF throttleCmd < 0.01 AND velVert < -1 AND aimErr <= maxWarpAim AND faceRate <= maxWarpRate AND timeToBrake > coastLeadTime * 2 {
            IF timeToBrake > coastLeadTime * 10 { SET wantWarp TO 4. }
            ELSE IF timeToBrake > coastLeadTime * 5 { SET wantWarp TO 3. }
            ELSE IF timeToBrake > coastLeadTime * 3 { SET wantWarp TO 2. }
            ELSE { SET wantWarp TO 1. }
        }
        
        IF wantWarp <> currentWarp {
            SET WARP TO wantWarp.
            SET currentWarp TO wantWarp.
            WAIT 0.2.
        }
        
        IF TIME:SECONDS > nextDiagTimer {
            printDiag().
            SET nextDiagTimer TO TIME:SECONDS + 2.
        }
        
        WAIT 0.
    }

    // --- Фаза SUICIDE ---
    IF NOT isLanded AND velVert < 0 AND altAGL <= brakeAlt AND altAGL > finalAlt {
        IF currentWarp > 0 {
            SET WARP TO 0.
            SET currentWarp TO 0.
            WAIT 0.2.
        }
        SET GEAR TO TRUE.
        PRINT "SUICIDE START".
        
        UNLOCK STEERING.
        UNLOCK THROTTLE.
        
        // suicide_burn.ks также должен читать глобалы или иметь свои константы
        // Здесь оставляем вызов с параметрами, так как suicide_burn может быть общим модулем
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

    // --- Фаза FINAL ---
    UNTIL isLanded OR altAGL > finalAlt {
        flightState().
        SET GEAR TO TRUE.
        SET throttleCmd TO 0. 
        
        IF twrNow < 1.05 {
            SET throttleCmd TO 0.
        } ELSE {
            // Простой P-контроллер для финальной посадки
            LOCAL targetMag TO touchdownSpeed + (altAGL / MAX(1, finalAlt)) * (finalApproachSpeed - touchdownSpeed).
            LOCAL targetVel TO -targetMag.
            LOCAL err TO targetVel - velVert.
            LOCAL accCmd TO gravAcc + 1.5 * err.
            
            IF velVert > 0 AND altAGL < 30 { SET accCmd TO MIN(accCmd, gravAcc * 0.35). }
            IF accCmd < 0 { SET accCmd TO 0. }
            
            SET throttleCmd TO clamp(accCmd / MAX(0.1, thrustAcc), 0, 1).
        }
        
        // В Final ракета всегда стремится к вертикали плавно
        SET steerFacing TO rotateToward(steerFacing, SHIP:UP:FOREVECTOR, maxSlewDeg, 0.1).
        LOCK STEERING TO steerFacing.
        
        IF TIME:SECONDS > nextDiagTimer {
            printDiag().
            SET nextDiagTimer TO TIME:SECONDS + 2.
        }
        
        WAIT 0.
    }
}

// Завершение
SET throttleCmd TO 0.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
UNLOCK THROTTLE.
UNLOCK STEERING.
SET WARP TO 0.
SET SAS TO TRUE.
SET GEAR TO TRUE.
PRINT "=== ГОТОВО ===".