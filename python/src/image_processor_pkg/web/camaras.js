// ---------------------------------------------------------------------------
// Mosaico de camaras. Mucho mas simple que panel.js: aqui no se pinta nada.
// Cada camara es un <img src="/video/<camara>"> apuntando a un MJPEG, y de
// mostrar el video se encarga el navegador el solo.
//
// Lo unico que hace este fichero es preguntar cada dos segundos que camaras hay
// (/estado_camaras), crear el recuadro de las que aparezcan y reabrir el video
// de las que se hayan quedado sin imagen.
// ---------------------------------------------------------------------------
const mosaico = document.getElementById("mosaico");

// camara -> {caja, img, etiqueta, boton, pausada}
const tiles = {};

const PERIODO_SONDEO = 2000;

// Cada cuanto, como mucho, se reabre el video de una camara que no da imagen.
// Tiene que ser >= el SIN_IMAGEN del nodo (10 s), que es lo que este tarda en
// cerrar por su cuenta un video que no recibe nada: reintentando mas a menudo
// que eso, cada camara apagada iria dejando conexiones a medio morir
const PERIODO_REINTENTO = 10000;

function urlVideo(cam) {
  // El ?t=<ahora> no lo lee nadie en el servidor: esta para que el navegador no
  // reutilice de su cache la conexion anterior, que ya esta cerrada
  return `/video/${cam}?t=${Date.now()}`;
}

function crearTile(cam) {
  const caja = document.createElement("div");
  caja.className = "camara-caja";

  const barra = document.createElement("div");
  barra.className = "camara-barra";

  const titulo = document.createElement("span");
  titulo.className = "camara-nombre";
  titulo.textContent = cam;

  const etiqueta = document.createElement("span");
  etiqueta.className = "camara-estado";

  const boton = document.createElement("button");
  boton.textContent = "pausa";

  const img = document.createElement("img");
  img.className = "camara-img";
  img.alt = cam;
  img.src = urlVideo(cam);

  const tile = { caja, img, etiqueta, boton, pausada: false, reintento: Date.now() };

  boton.onclick = () => {
    tile.pausada = !tile.pausada;
    if (tile.pausada) {
      // Quitar el src CORTA la conexion, y esa es toda la gracia: el nodo ve
      // que ya no queda nadie mirando esta camara y se da de baja de su topic.
      // Con cuatro camaras en una red compartida, esto es lo que evita mandar
      // imagenes que nadie esta viendo
      img.removeAttribute("src");
      boton.textContent = "ver";
      caja.classList.add("pausada");
    } else {
      img.src = urlVideo(cam);
      boton.textContent = "pausa";
      caja.classList.remove("pausada");
    }
    pintarEstado(cam, false);
  };

  barra.append(titulo, etiqueta, boton);
  caja.append(barra, img);
  mosaico.appendChild(caja);
  tiles[cam] = tile;
  return tile;
}

function pintarEstado(cam, hayImagen) {
  const tile = tiles[cam];
  if (tile.pausada) {
    tile.etiqueta.textContent = "en pausa";
    tile.etiqueta.className = "camara-estado";
  } else if (hayImagen) {
    tile.etiqueta.textContent = "en directo";
    tile.etiqueta.className = "camara-estado viva";
  } else {
    // O la camara tiene 'camera.debug: False', o esta arrancando, o se cayo
    tile.etiqueta.textContent = "sin imagen";
    tile.etiqueta.className = "camara-estado muerta";
  }
}

async function sondear() {
  let camaras;
  try {
    camaras = await (await fetch("/estado_camaras")).json();
  } catch (e) {
    document.getElementById("vivo").className = "desconectado";
    return;
  }
  document.getElementById("vivo").className = "punto-vivo";

  for (const c of camaras) {
    if (!tiles[c.id]) crearTile(c.id);
    const tile = tiles[c.id];

    // Sin imagen y no es que este pausada: se reabre el video. Cubre las dos
    // situaciones normales, la camara que aun no habia arrancado cuando se
    // abrio la pagina y el stream que el nodo cerro por no llegarle nada
    if (!c.imagen && !tile.pausada
        && Date.now() - tile.reintento > PERIODO_REINTENTO) {
      tile.reintento = Date.now();
      tile.img.src = urlVideo(c.id);
    }
    if (c.imagen) tile.reintento = Date.now();
    pintarEstado(c.id, c.imagen);
  }

  mosaico.classList.toggle("vacio", camaras.length === 0);
}

sondear();
setInterval(sondear, PERIODO_SONDEO);
