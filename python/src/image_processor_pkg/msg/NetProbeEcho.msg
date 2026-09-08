# Respuesta de la camara a un NetProbe
# Es el mismo mensaje devuelto tal cual, mas proc_ns
uint32 seq
string origen
string destino
builtin_interfaces/Time t_send
# Lo que tardo la camara en armar la respuesta, para descontarlo del RTT
uint64 proc_ns
uint8[] padding
