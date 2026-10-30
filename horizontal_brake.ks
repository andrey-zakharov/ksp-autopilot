// ==========================================================================
// horizontal_brake.ks — Один шаг гашения горизонтали
// Вход: none (читает SHIP и глобальные lastGlobalSec, throttleCmd, steerFacing)
// Выход: обновленные throttleCmd, steerFacing
// ==========================================================================

RUN utils.

PARAMETER horizTol IS 1.0.// Порог остановки (м/с)
PARAMETER fastTurnThreshold IS 5.0.       // Скорость, выше которой крутимся быстро

// Читаем состояние корабля напрямую
LOCAL upVec TO SHIP:UP:FOREVECTOR.
LOCAL velSurf TO SHIP:VELOCITY:SURFACE.
LOCAL vVert TO VDOT(velSurf, upVec).
LOCAL vHorizVec TO velSurf - (vVert * upVec).
LOCAL vHorizMag TO vHorizVec:MAG.

LOCAL gNow TO safeDiv(BODY:MU, (BODY:RADIUS + SHIP:ALTITUDE)^2, 0.001).
LOCAL massNow TO MAX(0.001, SHIP:MASS).
LOCAL thrustAvail TO SHIP:AVAILABLETHRUST.
IF thrustAvail <= 0.01 { SET thrustAvail TO SHIP:MAXTHRUST. }
LOCAL aThrust TO MAX(0.01, thrustAvail / massNow).

// Время шага
LOCAL nowSec TO TIME:SECONDS.
LOCAL rawDt TO nowSec - lastGlobalSec.
SET lastGlobalSec TO nowSec. // ОБНОВЛЯЕМ ГЛОБАЛЬНУЮ ПЕРЕМЕННУЮ
LOCAL dtSec TO clamp(rawDt, 0.01, 0.25).

// 1. Предельный угол наклона (Hover Limit)
// T_acc * cos(theta) <= g  =>  theta <= arccos(g / T_acc)
LOCAL tiltRadMax TO 0.
IF aThrust > gNow {
    SET tiltRadMax TO ARCCOS(clamp(gNow / aThrust, 0, 1)).
} ELSE {
    SET tiltRadMax TO 0.
}

// 2. Желаемый угол
LOCAL desiredTiltRad TO 0.
IF vHorizMag > 0.1 {
    // Эвристика: ATAN(v/a_ref). Чем больше скорость, тем больше наклон.
    // Ограничиваем лимитом ховера.
    LOCAL heuristicRad TO ARCTAN(vHorizMag / 2.0). 
    SET desiredTiltRad TO MIN(heuristicRad, tiltRadMax).
}

// 3. Целевой вектор
LOCAL antiHorizDir TO V(0,0,0).
IF vHorizMag > 0.01 {
    SET antiHorizDir TO (-1) * vHorizVec:NORMALIZED.
} ELSE {
    SET antiHorizDir TO upVec.
}

LOCAL desiredDir TO (COS(desiredTiltRad) * upVec + SIN(desiredTiltRad) * antiHorizDir):NORMALIZED.

// 4. Ориентация
IF vHorizMag > fastTurnThreshold {
    // Быстрый поворот
    LOCK STEERING TO LOOKDIRUP(desiredDir, upVec).
} ELSE {
    // Медленный возврат к вертикали
    SET steerFacing TO rotateToward(steerFacing, desiredDir, 2.0, dtSec).
    LOCK STEERING TO steerFacing.
}

// 5. Тяга (Predictive Braking)
LOCAL maxHorizAccel TO aThrust * SIN(desiredTiltRad).
LOCAL requiredHorizAccel TO vHorizMag / MAX(0.01, dtSec).

LOCAL throttleRatio TO 0.
IF maxHorizAccel > 0.01 {
    SET throttleRatio TO clamp(requiredHorizAccel / maxHorizAccel, 0, 1).
} ELSE {
    SET throttleRatio TO 0.
}

// Плавное снижение тяги у нуля
IF vHorizMag < horizTol * 2 {
    SET throttleRatio TO throttleRatio * (vHorizMag / MAX(0.1, horizTol * 2)).
}

// Гейт ориентации
LOCAL finalThrottle TO thrustGate(throttleRatio, desiredDir, 15).
SET finalThrottle TO clamp(finalThrottle, 0, 1).

// Обновляем глобальную тягу
SET throttleCmd TO finalThrottle.