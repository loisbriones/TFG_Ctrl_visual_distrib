# Sondeo de IDA que el NetProbeNode (en el PC) manda a una camara para medir
# la latencia de red. La camara lo devuelve como NetProbeEcho sin pensarlo.
#
# La medida es de IDA Y VUELTA (RTT) a proposito, y no de ida sola: en un RTT
# las dos marcas de tiempo las toma el MISMO reloj (el del emisor), asi que el
# desfase entre los relojes de las dos maquinas se cancela y desaparece del
# resultado (RFC 2681, seccion 2.7.1). En una medida unidireccional ese
# desfase se suma entero a lo medido, y sobre WiFi es del mismo orden de
# magnitud que la latencia que queremos medir: la cifra no significaria nada.
# Aqui no hay NTP en ninguna parte, y es deliberado.

# Numero de secuencia del sondeo, dentro de la serie de ESTA camara. Sirve
# para casar el eco con su envio y para contar los que no vuelven
uint32 seq

# Quien manda el sondeo (el nodo del PC) y a que camara va dirigido. Van en el
# mensaje para que una fila del CSV se entienda sola, sin mirar el topic
string origen
string destino

# Instante de salida segun el reloj del emisor. Viaja SOLO como informacion y
# la camara lo devuelve SIN TOCAR: nadie lo compara nunca con un reloj de otra
# maquina. El RTT de verdad lo calcula el emisor restando dos lecturas de su
# propio reloj monotonico (ver NetProbeNode.py), que es la unica resta valida
builtin_interfaces/Time t_send

# Relleno para que el paquete de sondeo pese mas o menos lo que un CarLocation
# real y viaje por la red en las mismas condiciones: un datagrama de 40 bytes
# y uno de 200 no se fragmentan igual ni esperan lo mismo en las colas. Su
# tamano se configura en params.yaml (net_probe.padding_bytes)
uint8[] padding
