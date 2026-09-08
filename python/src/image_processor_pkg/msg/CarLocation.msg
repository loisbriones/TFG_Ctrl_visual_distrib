string camara_id
string coche
Stiker front
Stiker back
builtin_interfaces/Time stamp
float64 proc_time
BoundingRect bounding_rect_stiker_front
BoundingRect bounding_rect_stiker_back
# Contador de frames procesados de la camara que lo envia, es local a cada camara
uint32 n_frame
# Tiempo desde que se capturo el frame hasta que se publico el mensaje
float64 age_at_publish
