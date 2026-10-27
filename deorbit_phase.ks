// ==========================================================================
// deorbit_phase.ks — Выполнение деорбитального маневра
// Вход: siteLat, siteLng, siteTerrain (глобальные переменные из land.ks)
// Выход: TRUE если маневр выполнен успешно или не требовался
// ==========================================================================

PARAMETER targetLat IS 0.
PARAMETER targetLng IS 0.
PARAMETER targetTerrain IS 0.

RUN utils.

FUNCTION findBrakingPoint { PARAMETER tLat. PARAMETER tLng. PARAMETER tTerr.
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
            // Простая эвристика: ищем момент, когда вектор скорости направлен на площадку
            // Для надежности можно использовать surfaceWorldVec из utils, если он там есть.
            // Здесь используем простую проверку угла между скоростью и радиус-вектором до площадки
            
            // NOTE: Для точного расчета нужен world position площадки в будущем времени.
            // Пока используем упрощенный подход: гасим всю орбитальную скорость сейчас/скоро.
            
            IF VANG(futureHoriz, (futurePos:NORMALIZED)) < 90 { 
                 // Грубая проверка направления
                 RETURN List(scanEta, futureHorizMag, 0).
            }
        }
        SET scanEta TO scanEta + 10.
    }
    RETURN List(bestEta, bestDv, bestErr).
}

PRINT "--- Деорбит: Начало ---".

// Удаляем старые ноды
UNTIL ALLNODES:LENGTH = 0 { REMOVE ALLNODES[0]. }
WAIT 0.

SET orbVel TO SHIP:VELOCITY:ORBIT.
SET nodeDv TO orbVel:MAG.

// Если мы уже падаем или скорость мала — выходим
IF nodeDv < 1 OR SHIP:ORBIT:PERIAPSIS <= 0 {
    PRINT "Деорбит: Не требуется (dV=" + ROUND(nodeDv,1) + ", Pe=" + ROUND(SHIP:ORBIT:PERIAPSIS) + ")".
    RETURN TRUE.
}

// Время ожидания для разворота (минимум 5 сек, максимум 30)
SET waitTime TO CLAMP(5, 5, 30). 

PRINT "Деорбит: dV=" + ROUND(nodeDv, 1) + " м/с, ETA=" + waitTime + " с".

// Создаем ноду: NODE(time, radial, normal, prograde)
// Ретроградно = отрицательный prograde
SET maneuverNode TO NODE(TimeSpan(waitTime), 0, 0, -nodeDv).
ADD maneuverNode.
WAIT 0.5.

UNLOCK THROTTLE.
UNLOCK STEERING.

RUN exec_node.

SET SAS TO FALSE.
PRINT "--- Деорбит: Завершен ---".
RETURN TRUE.