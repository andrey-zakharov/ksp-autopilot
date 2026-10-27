// ==========================================================================
// deorbit_phase.ks — Деорбитальный маневр
// Вход: none (читает SHIP напрямую)
// Выход: none (управление возвращается после выполнения ноды)
// ==========================================================================

RUN utils.

PRINT "--- Деорбит: Начало ---".

// Удаляем старые ноды
UNTIL ALLNODES:LENGTH = 0 { REMOVE ALLNODES[0]. }
WAIT 0.

SET orbVel TO SHIP:VELOCITY:ORBIT.
SET nodeDv TO orbVel:MAG.

// Если мы уже падаем или скорость мала — выходим
IF nodeDv < 1 OR SHIP:ORBIT:PERIAPSIS <= 0 {
    PRINT "Деорбит: Не требуется (dV=" + ROUND(nodeDv,1) + ", Pe=" + ROUND(SHIP:ORBIT:PERIAPSIS) + ")".
    EXIT.
}

// Оценка времени разворота
LOCAL turnAngle TO 180.
LOCAL massNow TO SHIP:MASS.
LOCAL estTurnTime TO estimateTurnTime(turnAngle, massNow).

// Время ожидания = время разворота + запас на стабилизацию
// exec_node сам будет ждать выравнивания, но нода должна быть поставлена 
// с учетом того, что ракета еще не смотрит туда.
SET waitTime TO MAX(10, estTurnTime + 5).

PRINT "Деорбит: dV=" + ROUND(nodeDv, 1) + " м/с, ETA=" + ROUND(waitTime,1) + " с".

// Создаем ноду: NODE(time, radial, normal, prograde)
// Ретроградно = отрицательный prograde
SET maneuverNode TO NODE(TimeSpan(waitTime), 0, 0, -nodeDv).
ADD maneuverNode.
WAIT 0.5.

UNLOCK THROTTLE.
UNLOCK STEERING.

// Выполняем ноду
RUN exec_node.

SET SAS TO FALSE.
PRINT "--- Деорбит: Завершен ---".