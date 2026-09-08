float64 dist_derrape
bool estado_derrapando
# Lo que tarda desde que llega la posicion hasta que sale la orden de PWM
float64 pipeline_time
# Instante exacto en el que llego el mensaje de la camara al controlador
builtin_interfaces/Time receive_msg_stamp
# Sin usar: el controlador no lo rellena
int32 roi_size
