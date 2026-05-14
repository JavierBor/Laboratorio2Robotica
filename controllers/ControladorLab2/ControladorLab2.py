from controller import Robot

# Configuración básica
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 1. Inicializar Motores
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# 2. Inicializar Sensores de Distancia
ps = []
sensors_names = ['ps0', 'ps7', 'ps5', 'ps2']
for name in sensors_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    ps.append(sensor)

# 3. Inicializar Encoders
left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# --- Variables de Odometría ---
RADIO_RUEDA = 0.0205 
robot.step(timestep)
pos_izq_anterior = left_encoder.getValue()
pos_der_anterior = right_encoder.getValue()

# --- Variables del Filtro Simple (COMENTADO) ---
# TAMANO_VENTANA = 5
# ventana_frontal = []

# --- NUEVO: Variables del Filtro de Kalman ---
# Valores iniciales
d_k = 0.0       # Distancia estimada inicial (\hat{d}_{k-1})
P_k = 1.0       # Covarianza inicial (incertidumbre de nuestra estimación)
Q = 0.01        # Ruido del proceso (incertidumbre de los encoders/deslizamiento)
R = 50.0        # Ruido de medición (varianza del sensor IR, ajustable)

# Bucle principal
while robot.step(timestep) != -1:
    # A. Leer Sensores de Distancia
    lectura_ps0 = ps[0].getValue()
    lectura_ps7 = ps[1].getValue()
    lectura_izq = ps[2].getValue()
    lectura_der = ps[3].getValue()
    
    # Medición cruda frontal (z_k)
    z_k = (lectura_ps0 + lectura_ps7) / 2.0
    
    # --- FILTRO SIMPLE (COMENTADO) ---
    # ventana_frontal.append(z_k)
    # if len(ventana_frontal) > TAMANO_VENTANA:
    #     ventana_frontal.pop(0)
    # dist_medida_filtrada = sum(ventana_frontal) / len(ventana_frontal)
    
    # B. Leer Encoders y Calcular Avance Lineal
    pos_izq_actual = left_encoder.getValue()
    pos_der_actual = right_encoder.getValue()

    delta_theta_izq = pos_izq_actual - pos_izq_anterior
    delta_theta_der = pos_der_actual - pos_der_anterior
    
    avance_lineal = (RADIO_RUEDA * delta_theta_izq + RADIO_RUEDA * delta_theta_der) / 2.0

    pos_izq_anterior = pos_izq_actual
    pos_der_anterior = pos_der_actual

    # --- NUEVO: FILTRO DE KALMAN ---
    
    # 1. Etapa de Predicción (Por movimiento)
    # Si el robot avanza hacia adelante, la distancia al obstáculo DISMINUYE.
    # Por eso restamos el avance lineal a la distancia anterior.
    # (Nota: El PDF muestra suma, pero lógicamente si avanzas, la distancia se acorta).
    d_k_prediccion = d_k - avance_lineal 
    P_k_prediccion = P_k + Q
    
    # 2. Etapa de Corrección (Con medición)
    # Calculamos la Ganancia de Kalman (K_k)
    K_k = P_k_prediccion / (P_k_prediccion + R)
    
    # Actualizamos la estimación fusionando predicción y medición
    d_k = d_k_prediccion + K_k * (z_k - d_k_prediccion)
    
    # Actualizamos la covarianza para el siguiente ciclo
    P_k = (1.0 - K_k) * P_k_prediccion

    # C. Lógica de Movimiento Básica (para probar)
    left_motor.setVelocity(3.0)
    right_motor.setVelocity(3.0)

    # Imprimir para comparar el comportamiento
    print(f"Cruda (z_k): {z_k:7.2f} | Kalman (d_k): {d_k:7.2f} | Ganancia K: {K_k:5.3f}")