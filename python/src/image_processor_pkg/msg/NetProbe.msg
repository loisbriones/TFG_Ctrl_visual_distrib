# Echo que el NetProbeNode manda a una camara para medir la latencia de la red
# La camara lo devuelve como NetProbeEcho

uint32 seq

# Quien lo manda y a que camara va
string origen
string destino
# Instante de salida, solo informativo: el RTT se mide con el reloj monotonico
builtin_interfaces/Time t_send
# Relleno para que el paquete pese lo mismo que un CarLocation
uint8[] padding
