string camara_id 
string coche
Stiker front
Stiker back
builtin_interfaces/Time stamp
float64 proc_time
BoundingRect bounding_rect_stiker_front
BoundingRect bounding_rect_stiker_back
# Contador de frames PROCESADOS de la camara emisora: identifica el fotograma
# del que salen estas coordenadas y es el mismo numero que va quemado en la
# imagen de debug. Es LOCAL a cada camara (ver NUMERACION_FRAMES.md): dos
# camaras distintas no comparten escala, para cruzarlas se usa stamp.
uint32 n_frame
