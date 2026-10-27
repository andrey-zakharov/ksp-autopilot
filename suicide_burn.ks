// ==========================================================================
// suicide_burn.ks — финальное торможение перед посадкой
// ==========================================================================
PARAMETER exitAlt IS 100.
PARAMETER exitSpeed IS 10.
PARAMETER maxTiltDeg IS 8.
PARAMETER touchdownSpeed IS 0.
PARAMETER maxIgniteTiltDeg IS 15.
PARAMETER slewDeg IS 10.
PARAMETER clearance IS 3. // Безопасный запас от земли (метры)

RUN utils.

SET sbThrottle TO 0.
SET sbSteer TO SHIP:FACING.
SET sbDiagTimer TO 0.

LOCK THROTTLE TO sbThrottle.
LOCK STEERING TO sbSteer.

SET sbLastSec TO TIME:SECONDS.
SET sbAlt TO MAX(0, SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT).
SET sbVVert TO VDOT(SHIP:VELOCITY:SURFACE, SHIP:UP:FOREVECTOR).

UNTIL SHIP:STATUS = "LANDED" OR
      SHIP:STATUS = "SPLASHED" OR
      sbAlt <= clearance OR 
      (sbAlt <= exitAlt AND ABS(sbVVert) <= MAX(10, exitSpeed * 2)) {

    SET sbNow TO TIME:SECONDS.
    SET sbDt TO clamp(sbNow - sbLastSec, 0.01, 0.25).
    SET sbLastSec TO sbNow.

    SET sbAlt TO MAX(0, SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT).
    SET sbUp TO SHIP:UP:FOREVECTOR.

    SET sbVel TO SHIP:VELOCITY:SURFACE.
    SET sbVVert TO VDOT(sbVel, sbUp).
    SET sbVHoriz TO sbVel - (sbVVert * sbUp).
    SET sbVHorizMag TO sbVHoriz:MAG.
    SET sbVVertAbs TO ABS(sbVVert).

    SET sbThrust TO SHIP:AVAILABLETHRUST.
    IF sbThrust <= 0.01 { SET sbThrust TO SHIP:MAXTHRUST. }

    SET sbMass TO MAX(0.001, SHIP:MASS).
    SET sbAcc TO MAX(0.01, sbThrust / sbMass).
    SET sbG TO safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.01).

    SET sbTiltDeg TO 0.
    
    // Небольшой наклон только для гашения остаточной горизонтали
    IF sbVHorizMag > 0.1 {
        SET sbTiltDeg TO MIN(maxTiltDeg, sbVHorizMag * 1.0).
        SET sbAnti TO (-1) * sbVHoriz:NORMALIZED.
        SET sbDesired TO (COS(sbTiltDeg) * sbUp + SIN(sbTiltDeg) * sbAnti):NORMALIZED.
    } ELSE {
        SET sbDesired TO sbUp.
    }

    SET sbVertFrac TO MAX(0.1, COS(sbTiltDeg)).
    SET sbNetAvail TO MAX(0.05, sbAcc * sbVertFrac - sbG).

    // Эффективная высота для торможения (до точки касания шасси)
    LOCAL effectiveAlt TO MAX(0.1, sbAlt - clearance).
    
    // Максимальная скорость, которую можно погасить за оставшуюся дистанцию
    LOCAL maxAllowedSpeed TO SQRT(MAX(0.01, 2 * sbNetAvail * effectiveAlt)).
    
    SET sbThrottle TO 0.

    // Логика управления тягой
    IF sbVVertAbs > maxAllowedSpeed {
        // Если скорость слишком велика для остановки на этой высоте — полное торможение
        SET sbThrottle TO 1.0.
    } ELSE {
        // Иначе плавное торможение к нулю на высоте clearance
        // Требуемое замедление для остановки за remaining distance
        LOCAL reqDecel TO (sbVVertAbs * sbVVertAbs) / (2 * MAX(0.1, effectiveAlt)).
        
        // Добавляем коэффициент запаса (1.2), чтобы гарантированно остановиться раньше
        LOCAL targetDecel TO reqDecel * 1.2.
        
        // Переводим требуемое замедление в ускорение двигателя
        // a_thrust_vert = g + targetDecel
        LOCAL thrustVertAcc TO sbG + targetDecel.
        
        // Ограничиваем тягу
        SET sbThrottle TO clamp(thrustVertAcc / MAX(0.1, sbAcc * sbVertFrac), 0, 1).
    }

    // Защита от взлета вверх у самой поверхности
    IF sbVVert > 0 AND sbAlt < MAX(50, exitAlt * 1.5) {
        SET sbThrottle TO MIN(sbThrottle, 0.5).
    }

    // Гейт ориентации
    SET sbThrottle TO thrustGate(sbThrottle, sbDesired, maxIgniteTiltDeg).
    SET sbThrottle TO clamp(sbThrottle, 0, 1).

    // Плавный разворот
    SET sbSteer TO rotateToward(sbSteer, sbDesired, slewDeg, sbDt).

    // Диагностика
    IF sbNow >= sbDiagTimer {
        PRINT "SB alt=" + ROUND(sbAlt) +
              " clr=" + ROUND(clearance) +
              " effH=" + ROUND(effectiveAlt) +
              " vV=" + ROUND(sbVVert,1) +
              " maxV=" + ROUND(maxAllowedSpeed,1) +
              " vH=" + ROUND(sbVHorizMag,1) +
              " tilt=" + ROUND(VANG(SHIP:FACING:FOREVECTOR, sbUp),1) +
              " thr=" + ROUND(sbThrottle * 100) + "%".
        SET sbDiagTimer TO sbNow + 2.
    }

    WAIT 0.
}

SET sbThrottle TO 0.
WAIT 0.05.

UNLOCK THROTTLE.
UNLOCK STEERING.
SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.