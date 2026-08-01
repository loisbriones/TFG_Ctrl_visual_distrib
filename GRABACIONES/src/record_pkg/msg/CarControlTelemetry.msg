float64 dist_derrape
bool estado_derrapando
# Tiempo que pasa desde que recibimos la informacion del coche hasta que enviamos el valor del pwm a RaceController
float64 pipeline_time
# Tiempo que permite almacenar el momento exacto en el que llego el paquete de la camar al CarController 
builtin_interfaces/Time receive_msg_stamp
# Para saber si estamos detectando correctamente el coche 
int32 roi_size
