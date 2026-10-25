// ==========================================================================
// suicide_burn.ks — suicide burn с регуляцией тяги и наклоном <= 5°
// ==========================================================================
// Использует глобальные переменные из land.ks:
//   vVert, vHoriz, vHorizMag, altAGL, upDir, gNow, aThrust, vSafe,
//   maxTiltRad, currentWarp, throttleSet, landingPhase, inSuicideBurn
//
// Устанавливает:
//   suicideDone = TRUE когда altAGL <= 100
//   suicideFailed = TRUE если ракета улетает вверх (vVert > 0 и altAGL > 1500)
// ==========================================================================

SET suicideDone TO FALSE.
SET suicideFailed TO FALSE.
SET smoothTargetDir TO V(0, 1, 0).
SET dirInitialized TO FALSE.

UNTIL suicideDone OR suicideFailed {
    // Пересчитываем параметры каждый кадр
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
    
    SET vSafe TO SQRT(2 * aNet * MAX(1, altAGL)).
    
    // Проверка завершения
    IF altAGL <= 100 {
        SET suicideDone TO TRUE.
        BREAK.
    }
    
    // Проверка улёта вверх
    IF vVert > 0 AND altAGL > 1500 {
        PRINT "  [!] Ракета улетает вверх (vVert=" + ROUND(vVert,1) + "), перехожу в coast".
        SET suicideFailed TO TRUE.
        BREAK.
    }
    
    // Установка фазы
    SET landingPhase TO "suicide".
    SET inSuicideBurn TO TRUE.
    
    // Управление тягой
    LOCK THROTTLE TO throttleSet.
    SET SAS TO FALSE.
    
    // Выход из варпа если был
    IF currentWarp > 0 {
        SET WARP TO 0.
        WAIT 0.5.
        SET currentWarp TO 0.
    }
    
    // Направление тяги: вертикально с наклоном <= 5° против vHoriz
    SET maxTiltRad TO 5 * CONSTANT:PI / 180.  // Максимум 5°
    
    IF vHorizMag > 0.5 {
        SET antiHorizDir TO (-1) * vHoriz:NORMALIZED.
        // Наклон пропорционален vHoriz, но не больше 5°
        SET tiltFactor TO MIN(maxTiltRad, vHorizMag / 30).
        SET rawTargetDir TO (COS(tiltFactor) * upDir + SIN(tiltFactor) * antiHorizDir):NORMALIZED.
    } ELSE {
        SET rawTargetDir TO upDir.
    }
    
    // Сглаживание направления
    IF NOT dirInitialized {
        SET smoothTargetDir TO rawTargetDir.
        SET dirInitialized TO TRUE.
    } ELSE {
        SET alpha TO 0.1.
        SET smoothTargetDir TO ((1 - alpha) * smoothTargetDir + alpha * rawTargetDir):NORMALIZED.
    }
    
    LOCK STEERING TO LOOKDIRUP(smoothTargetDir, upDir).
    
    // Регуляция тяги в зависимости от vVert
    IF vVert > 0 {
        // Ракета поднимается — выключаем тягу
        SET throttleSet TO 0.
    } ELSE IF vVert > -5 {
        // Ракета почти остановилась — минимальная тяга (компенсация g)
        SET throttleSet TO safeDiv(gNow, aThrust, 0.5).
    } ELSE {
        // Ракета падает — тормозим до vSafe
        SET vTargetSuicide TO MIN(vSafe, 20).
        SET aNeeded TO (vVertAbs*vVertAbs - vTargetSuicide*vTargetSuicide) / (2 * MAX(1, altAGL)) + gNow.
        SET minThrottle TO safeDiv(gNow, aThrust, 0.1).
        SET throttleSet TO clamp(aNeeded / MAX(0.1, aThrust), minThrottle, 1).
    }
    
    SET throttleSet TO clamp(throttleSet, 0, 1).
    
    WAIT 0.
}