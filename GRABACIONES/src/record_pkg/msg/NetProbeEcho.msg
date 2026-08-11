# Respuesta de la camara a un NetProbe. Es el mismo mensaje devuelto tal cual
# mas proc_ns: la camara no interpreta nada, solo rebota y cronometra lo suyo.
#
# Con este eco el emisor cierra el viaje de ida y vuelta y calcula el RTT
# restando dos lecturas de SU reloj monotonico. Al hacerse las dos medidas con
# el mismo reloj, el desfase entre las maquinas no entra en la cuenta
# (RFC 2681, seccion 2.7.1); por eso no hace falta sincronizar nada con NTP.

# Copiados del NetProbe SIN MODIFICAR: identifican de que sondeo es este eco
uint32 seq
string origen
string destino

# El t_send original, devuelto tal cual. No se usa para calcular el RTT (eso
# lo hace el emisor con su reloj monotonico): queda como rastro de que este
# eco corresponde a ese envio y para poder revisarlo despues en el bag
builtin_interfaces/Time t_send

# Lo que la camara tardo DENTRO del callback, desde que entro hasta justo
# antes de publicar la respuesta. Es una DURACION, no un instante: se mide
# restando dos lecturas del reloj monotonico de la propia camara, asi que el
# desfase de reloj entre maquinas no le afecta y se puede restar del RTT bruto
# sin miedo. Ese tiempo es error de medida (es la camara, no la red), y por eso
# el emisor guarda tambien el RTT neto = RTT bruto - proc_ns
uint64 proc_ns

# El relleno del sondeo, devuelto sin tocar, para que el eco pese lo mismo que
# la ida y los dos sentidos se midan en igualdad de condiciones
uint8[] padding
